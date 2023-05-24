#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Oct  3 13:38:45 2018

@author: Romeo Orsolino
"""

import copy
import numpy as np
import os

import rospy as ros
import sys
import time
import threading

from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Point

from dls_msgs.msg import StringDoubleArray
from feasible_region.msg import RobotStates, TCrawlStates
from feasible_region.msg import Polygon3D, LegsPolygons, TCrawlFoothold

from std_msgs.msg import Float32MultiArray    # for tcrawl publisher
from std_msgs.msg import MultiArrayDimension  # for tcrawl publisher

from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.maths.math_tools import Math
from jet_leg.maths.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.optimization.tcrawl_foothold_planning_interface import TCrawlFootholdPlanningInterface
from jet_leg.optimization import nonlinear_projection

from jet_leg.optimization.tcrawl_foothold_planning import TCrawlFootHoldPlanning

from jet_leg.plotting.plotting_tools import Plotter
import matplotlib.pyplot as plt

np.set_printoptions(precision=3, linewidth=200, suppress=True)
np.set_printoptions(threshold=np.inf)

stderr = sys.stderr
sys.stderr = open(os.devnull, 'w')
sys.stderr = stderr


class HyQSim(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)

		self.clock_sub_name = 'clock'
		self.hyq_actuation_params_sub_name = "/feasible_region/robot_states"
		self.hyq_actuation_footholds_params_sub_name = "/feasible_region/tcrawl_states"
		self.hyq_wbs = dict()
		self.hyq_debug_msg = RobotStates()
		self.hyq_tcrawl_states_msg = TCrawlStates()
		self.actuation_polygon_topic_name = "/feasible_region/actuation_polygon"
		self.reachable_feasible_topic_name = "/feasible_region/reachble_feasible_region_polygon"
		self.support_region_topic_name = "/feasible_region/support_region"
		self.force_polygons_topic_name = "/feasible_region/force_polygons"
		self.tcrawl_optimal_foothold = "/tcrawl/optimal_footholds"
		print ros.get_namespace()
		self.sim_time = 0.0




	def run(self):
		print "Run!"
		self.sub_clock = ros.Subscriber(self.clock_sub_name, Clock, callback=self._reg_sim_time, queue_size=1000)
		self.sub_actuation_params = ros.Subscriber(self.hyq_actuation_params_sub_name, RobotStates,
												   callback=self.callback_hyq_debug, queue_size=5, buff_size=3000)
		self.sub_actuation_footholds_params = ros.Subscriber(self.hyq_actuation_footholds_params_sub_name, TCrawlStates,
															 callback=self.callback_hyq_footholds, queue_size=5,
															 buff_size=1500)
		self.pub_feasible_polygon = ros.Publisher(self.actuation_polygon_topic_name, Polygon3D, queue_size=10000)

		self.pub_optimal_foothold = ros.Publisher(self.tcrawl_optimal_foothold, TCrawlFoothold, queue_size=10000)

		self.pub_reachable_feasible_polygon = ros.Publisher(self.reachable_feasible_topic_name, Polygon3D,
															queue_size=10000)
		self.pub_support_region = ros.Publisher(self.support_region_topic_name, Polygon3D, queue_size=1000)

	def _reg_sim_time(self, time):
		self.sim_time = time.clock.secs + time.clock.nsecs / 1000000000.0

	#        print("getting time")

	def _reg_sim_wbs(self, msg):
		self.hyq_wbs = copy.deepcopy(msg)

	def callback_hyq_debug(self, msg):
		# print 'new data received'
		self.hyq_debug_msg = copy.deepcopy(msg)

	def callback_hyq_footholds(self, msg):
		self.hyq_tcrawl_states_msg = copy.deepcopy(msg)

	def register_node(self):
		ros.init_node('sub_pub_node_python_tcrawl', anonymous=False)

	def deregister_node(self):
		ros.signal_shutdown("manual kill")

	def get_sim_time(self):
		return self.sim_time

	def get_sim_wbs(self):
		return self.hyq_wbs

	def send_support_region(self, name, vertices):
		output = Polygon3D()
		output.vertices = vertices
		self.pub_support_region.publish(output)

	def send_actuation_polygons(self, name, vertices, option_index, ack_optimization_done):
		output = Polygon3D()
		output.vertices = vertices
		output.option_index = option_index
		output.ack_optimization_done = ack_optimization_done
		self.pub_feasible_polygon.publish(output)

	def send_reachable_feasible_polygons(self, name, vertices, option_index, ack_optimization_done):
		output = Polygon3D()
		output.vertices = vertices
		output.option_index = option_index
		output.ack_optimization_done = ack_optimization_done
		self.pub_reachable_feasible_polygon.publish(output)

	def fillPolygon(self, polygon):
		# print 'polygon ', polygon
		num_actuation_vertices = np.size(polygon, 0)
		vertices = []

		for i in range(0, num_actuation_vertices):
			point = Point()
			point.x = polygon[i][0]
			point.y = polygon[i][1]
			point.z = 0.0  # is the centroidal frame
			vertices = np.hstack([vertices, point])
		return vertices


def talker(robotName):

	footHoldPlanning = TCrawlFootHoldPlanning(robotName)
	# joint_projection = nonlinear_projection.NonlinearProjectionBretl(robot_name)

	# Create a communication thread
	p = HyQSim()
	p.start()  # Start thread
	p.register_node()

	# Create parameter objects
	params = IterativeProjectionParameters()
	foothold_params = TCrawlFootholdPlanningInterface()
	i = 0

	p.get_sim_wbs()
	# Save foothold planning and IP parameters from "debug" topic
	params.getParamsFromRosDebugTopic(p.hyq_debug_msg)
	# foothold_params.getParamsFromRosDebugTopic(p.hyq_debug_msg)
	params.getFutureStanceFeetFlags(p.hyq_debug_msg)

	""" contact points """
	ng = 4
	params.setNumberOfFrictionConesEdges(ng)

	# ''' joint position limits for each leg (this code assumes a hyq-like design, i.e. three joints per leg)
	# HAA = Hip Abduction Adduction
	# HFE = Hip Flextion Extension
	# KFE = Knee Flextion Extension
	# '''
	# LF_q_lim_max = [0.44, 1.2217, -0.3491]  # HAA, HFE, KFE
	# LF_q_lim_min = [-1.22, -0.8727, -2.4435]  # HAA, HFE, KFE
	# RF_q_lim_max = [0.44, 1.2217, -0.3491]  # HAA, HFE, KFE
	# RF_q_lim_min = [-1.22, -0.8727, -2.4435]  # HAA, HFE, KFE
	# LH_q_lim_max = [0.44, 0.8727, 2.4435]  # HAA, HFE, KFE
	# LH_q_lim_min = [-1.22, -1.2217, 0.3491]  # HAA, HFE, KFE
	# RH_q_lim_max = [0.44, 0.8727, 2.4435]  # HAA, HFE, KFE
	# RH_q_lim_min = [-1.22, -1.2217, 0.3491]  # HAA, HFE, KFE
	# joint_limits_max = np.array([LF_q_lim_max, RF_q_lim_max, LH_q_lim_max, RH_q_lim_max])
	# joint_limits_min = np.array([LF_q_lim_min, RF_q_lim_min, LH_q_lim_min, RH_q_lim_min])
	#
	# params.setJointLimsMax(joint_limits_max)
	# params.setJointLimsMin(joint_limits_min)

	inside = 0


	while not ros.is_shutdown():

		opt_start_time = time.time()
		p.get_sim_wbs()

		# Save foothold planning and IP parameters from "debug" topic
		params.getParamsFromRosDebugTopic(p.hyq_debug_msg)
		foothold_params.getParamsFromRosDebugTopic(p.hyq_tcrawl_states_msg)
		foothold_params.getFutureSupportFeetFlags(p.hyq_tcrawl_states_msg)
		# # Remember to set the tcrawl future support feet in iterative projection parameters
		# params.setActiveContacts(foothold_params.getSupportFeet())
		# print "Com: ", params.comPositionWF
		# print "optimization start: ", foothold_params.tcrawl_optimization_start
		# print"self.state_machine: ", params.state_machine
		# print "foothold_params.tcrawl_optimization_start: ", foothold_params.tcrawl_optimization_start
		# print "out.ack_optimization_done: ", foothold_params.ack_optimization_done
		# print "step_direction: ", foothold_params.step_direction_wf
		#
		# # FOOTHOLD PLANNING
		#
		#
		if foothold_params.tcrawl_optimization_start == False:
			foothold_params.ack_optimization_done = False
		#
		# ''' The optimization-done-flag is set by the planner. It is needed to tell the controller whether the optimization
		# is finished or not. When this flag is true the controller will read the result of the optimization that has read
		# from the planner'''
		# ''' The optimization-started-flag is set by the controller. It is needed to tell the planner that a new optimization should start.
		# When this flag is true the planner (in jetleg) will start a new computation of the feasible region.'''
		#

		out = TCrawlFoothold()

		# print "time: ", time.time() - first

		if foothold_params.tcrawl_optimization_start and not foothold_params.ack_optimization_done:
			print '============================================================'
			print 'current swing ', foothold_params.swing_leg_index
			print '============================================================'
		#
			# print "future support feet: ", foothold_params.supportFeet
			# print "swing leg: ", foothold_params.swing_leg_index
			# print "first or second leg: ", foothold_params.leg_sequence_index
			# print "step direciton wf", foothold_params.step_direction_wf
			# print "search step: ", foothold_params.search_step
			# print "while inside::::"
			# print "tcrawl_optimization_start: ", foothold_params.tcrawl_optimization_start
			# print "tcrawl_ack_opt_done: ", foothold_params.ack_optimization_done
			# print "future_region: ", foothold_params.future_support_triangle
			inside = 1

		# 	# Select foothold option with maximum feasible region from among all possible (default is 9) options
			optimal_foothold, optimization_success = footHoldPlanning.compute_optimal_foothold(foothold_params, params)
			print "calculation time_mili: ", (time.time() - opt_start_time) * 1000.0

			foothold_params.ack_optimization_done = True
			# foothold_params.optimization_success = optimization_success

			out.ack_optimization_done = foothold_params.ack_optimization_done
			out.optimization_success = optimization_success
			out.optimal_foothold = optimal_foothold
			p.pub_optimal_foothold.publish(out)
		#
		# 	#         ONLY_ACTUATION, ONLY_FRICTION or FRICTION_AND_ACTUATION
		# 	#        3 - FRICTION REGION
		# 	constraint_mode_IP = 'ONLY_FRICTION'
		# 	params.setConstraintModes([constraint_mode_IP,
		# 							constraint_mode_IP,
		# 							constraint_mode_IP,
		# 							constraint_mode_IP])
		# 	params.setNumberOfFrictionConesEdges(ng)
		#
		# 	params.contactsWF[params.actual_swing] = foothold_params.footOptions[foothold_params.option_index]
		#
		#
		# 	frictionRegion, actuation_polygons, computation_time = compDyn.iterative_projection_bretl(params)
		#
		# 	# this sends the data back to ros that contains the foot hold choice (used for stepping)
		# 	# and the corresponding region (that will be used for com planning TODO update with the real footholds)
		# 	if (actuationRegions is not False) and (np.size(actuationRegions) is not 0):
		# 		print 'sending actuation region'
		# 		p.send_actuation_polygons(name, p.fillPolygon(actuationRegions[-1]), foothold_params.option_index,
		# 								foothold_params.ack_optimization_done)
		# 	else:
		# 		# if it cannot compute anything it will return the frictin region
		# 		p.send_actuation_polygons(name, p.fillPolygon(frictionRegion), foothold_params.option_index,
		# 								foothold_params.ack_optimization_done)
		#
		# 	p.send_support_region(name, p.fillPolygon(frictionRegion))
		# 	initializing multidimensional message
		dimension = 5
		mat = Float32MultiArray()
		mat.layout.dim.append(MultiArrayDimension())
		mat.layout.dim[0].label = "height"
		mat.layout.dim[0].size = dimension
		mat.layout.dim[0].stride = dimension
		mat.layout.data_offset = 0
		mat.data = [0] * dimension
		dummy_output = np.random.rand(5)
		mat.data = dummy_output

		# time.sleep(0.001)
		i += 1

	print 'de registering...'
	p.deregister_node()


if __name__ == '__main__':

	try:
		robot_name = 'hyq'
		talker(robot_name)
	except ros.ROSInterruptException:
		pass

