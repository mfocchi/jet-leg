#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Oct  3 13:38:45 2018

@author: Romeo Orsolino
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import copy
import numpy as np
import os

import rospy as ros
import sys
import time
import threading

from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Point

# from dls_msgs.msg import SimpleDoubleArray, StringDoubleArray, Polygon3D, LegsPolygons
from dls_msgs.msg import StringDoubleArray
from feasible_region.msg import RobotStates, Foothold, OrientationRequest
from feasible_region.msg import Polygon3D, LegsPolygons, OptimalOrientation
from shapely.geometry import Polygon, LineString
from shapely.geos import TopologicalError
import message_filters

from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.optimization.simple_foothold_planning_interface import FootholdPlanningInterface
from jet_leg.optimization.orientation_planning_interface import OrientationPlanningInterface
from jet_leg.optimization import nonlinear_projection

from jet_leg.optimization.foothold_planning import FootHoldPlanning
# from jet_leg.optimization.orientation_planning import OrientationPlanning
from jet_leg.optimization.orientation_planning_multiprocess import OrientationPlanningMultiProcess

from jet_leg.plotting.plotting_tools import Plotter
import matplotlib.pyplot as plt

# to rad yaml from a ros package
import os, rospkg

rospack = rospkg.RosPack()
import yaml

# services
from feasible_region.srv import Config, ConfigResponse

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
		self.hyq_actuation_footholds_params_sub_name = "/feasible_region/foothold"
		self.hyq_orientation_request_params_sub_name = "/feasible_region/orientation_request"
		self.hyq_wbs = dict()
		self.hyq_debug_msg = RobotStates()
		self.hyq_footholds_msg = Foothold()
		self.hyq_orientation_request_msg = OrientationRequest()
		self.actuation_polygon_topic_name = "/feasible_region/actuation_polygon"
		self.reachable_feasible_topic_name = "/feasible_region/reachble_feasible_region_polygon"
		self.support_region_topic_name = "/feasible_region/support_region"
		self.force_polygons_topic_name = "/feasible_region/force_polygons"
		self.optimal_orientation_topic_name = "/feasible_region/optimal_orientation"
		self.robot_name = ros.get_param('/robot_name')
		# self.hyq_wbs_sub_name = "/"+self.robot_name+"/robot_states" not used
		print(ros.get_namespace())
		self.sim_time = 0.0

		self.plotFeasibleRegionFlag = False
		self.plotExtendedRegionFlag = False
		self.plotReachableFeasibleRegionFlag = False
		self.plotFrictionRegion = False
		self.plotForcePolygonsFlag = False

		with open(os.path.join(rospack.get_path("feasible_region"), "config", "feasible_regions_options.yaml"),
				  'rb')         as stream:
			data_loaded = yaml.load(stream)
		visualizationOptions = data_loaded['visualizationOptions']
		self.parse_vis_options(visualizationOptions)

		self.com_optimization = False
		self.foothold_optimization = False
		self.com_optimization_type = 0

		self.orient_ack_optimization_done = False  # Not used

	def run(self):
		print("Started Jetleg!")
		self.sub_clock = ros.Subscriber(self.clock_sub_name, Clock, callback=self._reg_sim_time, queue_size=1000)
		self.sub_actuation_params = ros.Subscriber(self.hyq_actuation_params_sub_name, RobotStates,
												   callback=self.callback_hyq_debug, queue_size=1, buff_size=3000)
		# self.sub_actuation_footholds_params = ros.Subscriber(self.hyq_actuation_footholds_params_sub_name, Foothold,
		#                                            callback=self.callback_hyq_footholds, queue_size=1, buff_size=500)
		# self.sub_orientation_request_params = ros.Subscriber(self.hyq_orientation_request_params_sub_name, OrientationRequest,
		#                                            callback=self.callback_hyq_orientation, queue_size=1, buff_size=500)
		# self.sub_actuation_params = message_filters.Subscriber(self.hyq_actuation_params_sub_name, RobotStates, queue_size=1, buff_size=3000)
		# self.sub_actuation_footholds_params = message_filters.Subscriber(self.hyq_actuation_footholds_params_sub_name, Foothold, queue_size=1, buff_size=1000)
		# self.sub_orientation_request_params = message_filters.Subscriber(self.hyq_orientation_request_params_sub_name,
		#                                                      OrientationRequest, queue_size=1, buff_size=1000)
		#
		# self.ts = message_filters.ApproximateTimeSynchronizer([self.sub_actuation_params, self.sub_actuation_footholds_params, self.sub_orientation_request_params], 10, 1)
		# self.ts.registerCallback(self.callback_synchronizer)

		self.pub_feasible_polygon = ros.Publisher(self.actuation_polygon_topic_name, Polygon3D, queue_size=10000)
		self.pub_optimal_orientation = ros.Publisher(self.optimal_orientation_topic_name, OptimalOrientation,
													 queue_size=1)
		self.pub_reachable_feasible_polygon = ros.Publisher(self.reachable_feasible_topic_name, Polygon3D,
															queue_size=10000)
		self.pub_support_region = ros.Publisher(self.support_region_topic_name, Polygon3D, queue_size=1000)
		self.pub_force_polygons = ros.Publisher(self.force_polygons_topic_name, LegsPolygons, queue_size=1000)

	def config_server(self):

		# advertise service
		service = ros.Service('/feasible_region/set_config', Config, self.handle_set_feasible_region_config)
		print("Feasible Region Config Server initialized")

	def handle_set_feasible_region_config(self, req):
		print("Returning [vis opt: %s , com %s , comtype: %s foothold: %s]" % (
		req.visualization_options, req.com_optimization, req.com_optimization_type, req.foothold_optimization))
		self.com_optimization = req.com_optimization
		self.foothold_optimization = req.foothold_optimization
		self.com_optimization_type = req.com_optimization_type
		if (len(req.visualization_options) < 8):
			print("wrong visualization option size is :", len(req.visualization_options))
			return ConfigResponse(False)
		self.parse_vis_options(req.visualization_options)
		return ConfigResponse(True)

	def parse_vis_options(self, input):
		self.plotFeasibleRegionFlag = input[0] == "1"
		self.plotReachableFeasibleRegionFlag = input[2] == "1"
		self.plotExtendedRegionFlag = input[3] == "1"
		self.plotFrictionRegion = input[5] == "1"
		self.plotForcePolygonsFlag = input[6] == "1"

	def _reg_sim_time(self, time):

		self.sim_time = time.clock.secs + time.clock.nsecs / 1000000000.0

	#        print("getting time")

	def _reg_sim_wbs(self, msg):
		self.hyq_wbs = copy.deepcopy(msg)

	def callback_synchronizer(self, robot_states_msg, foothold_msg, orient_req_msg):
		self.hyq_debug_msg = copy.deepcopy(robot_states_msg)
		self.hyq_footholds_msg = copy.deepcopy(foothold_msg)
		self.hyq_orientation_request_msg = copy.deepcopy(orient_req_msg)

	def callback_hyq_debug(self, msg):
		self.hyq_debug_msg = copy.deepcopy(msg)
		if (self.hyq_debug_msg.orient_optimization_started == False):
			self.orient_ack_optimization_done = False

	def callback_hyq_footholds(self, msg):
		self.hyq_footholds_msg = copy.deepcopy(msg)

	def callback_hyq_orientation(self, msg):
		self.hyq_orientation_request_msg = copy.deepcopy(msg)

	def register_node(self):
		ros.init_node('sub_pub_node_python', anonymous=False)

	def deregister_node(self):
		ros.signal_shutdown("manual kill")

	def get_sim_time(self):
		return self.sim_time

	def get_sim_wbs(self):
		return self.hyq_wbs

	def send_force_polytopes(self, name, polygons):
		output = LegsPolygons()
		output.polygons = polygons
		self.pub_force_polygons.publish(output)

	def send_friction_region(self, name, vertices):
		output = Polygon3D()
		output.vertices = vertices
		self.pub_support_region.publish(output)

	def send_feasible_polygons(self, name, vertices, option_index, ack_optimization_done):
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

	def send_optimal_orientation(self, name, optimal_orientation, optimization_success, ack_optimization_done):
		output = OptimalOrientation()
		output.optimal_orientation[0] = optimal_orientation[0]
		output.optimal_orientation[1] = optimal_orientation[1]
		output.optimal_orientation[2] = optimal_orientation[2]
		output.optimization_success = optimization_success
		output.ack_optimization_done = ack_optimization_done
		self.pub_optimal_orientation.publish(output)

	def fillPolygon(self, polygon):
		# print 'polygon ', polygon
		num_actuation_vertices = np.size(polygon, 0)
		vertices = []

		for i in range(0, num_actuation_vertices):
			point = Point()
			point.x = polygon[i][0]
			point.y = polygon[i][1]
			point.z = polygon[i][2]  # is the centroidal frame
			vertices = np.hstack([vertices, point])
		return vertices

	def computeFeasibleRegion(self, params, ng, compDyn):
		constraint_mode_IP = 'FRICTION_AND_ACTUATION'
		params.setConstraintModes([constraint_mode_IP]*params.getNoOfLegs())
		params.setNumberOfFrictionConesEdges(ng)
		FEASIBLE_REGION, actuation_polygons_array, computation_time = compDyn.iterative_projection_bretl(params)

		return FEASIBLE_REGION, actuation_polygons_array, computation_time


def talker():
	# Create a communication thread
	time.sleep(15)
	p = HyQSim()
	p.start()  # Start thread
	p.register_node()
	# start config node
	p.config_server()

	compDyn = ComputationalDynamics(p.robot_name)
	footHoldPlanning = FootHoldPlanning(p.robot_name)
	# orient_planning = OrientationPlanning(p.robot_name)
	orient_planning_mp = OrientationPlanningMultiProcess(p.robot_name)
	joint_projection = nonlinear_projection.NonlinearProjectionBretl(p.robot_name)
	math = Math()

	name = "Actuation_region"
	force_polytopes_name = "force_polytopes"

	# Create parameter objects
	params = IterativeProjectionParameters()
	foothold_params = FootholdPlanningInterface()
	orient_params = OrientationPlanningInterface()
	i = 0

	p.get_sim_wbs()
	# Save foothold planning and IP parameters from "debug" topic
	first = time.time()
	# print p.hyq_debug_msg.tau_lim.data[0]
	time.sleep(1)
	params.getNoOfLegsFromMsg(p.hyq_debug_msg)
	params.getParamsFromRosDebugTopic(p.hyq_debug_msg)
	# foothold_params.getParamsFromRosDebugTopic(p.hyq_footholds_msg)
	# orient_params.getParamsFromRosDebugTopic(p.hyq_orientation_request_msg)
	foothold_params.getParamsFromRosDebugTopic(p.hyq_debug_msg)
	orient_params.getParamsFromRosDebugTopic(p.hyq_debug_msg)
	params.getFutureStanceFeetFlags(p.hyq_debug_msg)

	old_reachable_feasible_polygon = []
	old_frictionRegion = [[0, 0, 0]]
	old_FEASIBLE_REGION = [0]

	""" contact points """
	ng = 4
	params.setNumberOfFrictionConesEdges(ng)

#	plt.close()
#	plt.figure()
#	plotter = Plotter()
#	plt.ion()
	# plt.show()
	i = 0

	while not ros.is_shutdown():

		# Save foothold planning and IP parameters from "debug" topic
		first = time.time()
		params.getParamsFromRosDebugTopic(p.hyq_debug_msg)
		# foothold_params.getParamsFromRosDebugTopic(p.hyq_footholds_msg)
		# orient_params.getParamsFromRosDebugTopic(p.hyq_orientation_request_msg)
		foothold_params.getParamsFromRosDebugTopic(p.hyq_debug_msg)
		orient_params.getParamsFromRosDebugTopic(p.hyq_debug_msg)

		# print "CoMWF: ", params.getCoMPosWF()
		# print "CoMBF: ", params.getCoMPosBF()
		# print "rpy: ", params.getOrientation()
		# print "contacts: ", params.getContactsPosWF()
		# print "robotMass: ", params.robotMass
		# print "planeNormal: ", params.get_plane_normal()
		# print "normals: ", params.getNormals()

		if p.com_optimization or p.foothold_optimization:
			params.getFutureStanceFeetFlags(p.hyq_debug_msg)
		else:
			params.getCurrentStanceFeetFlags(p.hyq_debug_msg)
		# print "CoM: ", params.getCoMPosWF()
		# print "time: ", time.time() - first
		# print "Stance feet: ", params.stanceFeet

		if (p.plotFrictionRegion):
			constraint_mode_IP = 'ONLY_FRICTION'
			params.setConstraintModes([constraint_mode_IP]*params.getNoOfLegs())
			params.setNumberOfFrictionConesEdges(ng)
			frictionRegion, actuation_polygons, computation_time = compDyn.iterative_projection_bretl(params)
                        #print("frictionRegion: ", frictionRegion)
			if frictionRegion is not False:
				p.send_friction_region(name, p.fillPolygon(frictionRegion))
				old_frictionRegion = frictionRegion
			else:
				p.send_friction_region(name, p.fillPolygon(old_frictionRegion))
		# print "frictionRegion: ", frictionRegion
		# print "friction time: ", computation_time

		if (p.plotFeasibleRegionFlag or p.plotExtendedRegionFlag):
			FEASIBLE_REGION, actuation_polygons_array, computation_time = p.computeFeasibleRegion(params, ng, compDyn)
                        #print("FEASIBLE_REGION: ", FEASIBLE_REGION)
                        # safety measure use old when you cannot compute
			if (p.plotFeasibleRegionFlag):
				if FEASIBLE_REGION is not False:
					p.send_feasible_polygons(name, p.fillPolygon(FEASIBLE_REGION), foothold_params.option_index,
											 foothold_params.ack_optimization_done)
					old_FEASIBLE_REGION = FEASIBLE_REGION
				else:
					print('Could not compute the feasible region')
					p.send_feasible_polygons(name, p.fillPolygon(old_FEASIBLE_REGION), foothold_params.option_index,
											 foothold_params.ack_optimization_done)

		# if (p.plotReachableFeasibleRegionFlag and not p.plotExtendedRegionFlag):
		# 	reachability_polygon, computation_time_joint = joint_projection.project_polytope(params, None,
		# 																					 20. * np.pi / 180, 0.03)
		# 	# print "reachable region computation_time: ", computation_time_joint
		# 	if reachability_polygon.size > 0:
		# 		old_reachable_feasible_polygon = reachability_polygon
		# 		p.send_reachable_feasible_polygons(name, p.fillPolygon(reachability_polygon),
		# 										   foothold_params.option_index,
		# 										   foothold_params.ack_optimization_done)
		# 	else:
		# 		p.send_reachable_feasible_polygons(name, p.fillPolygon([]),
		# 										   foothold_params.option_index,
		# 										   foothold_params.ack_optimization_done)

		# if (p.plotExtendedRegionFlag):
		# 	FEASIBLE_REGION, actuation_polygons_array, computation_time = p.computeFeasibleRegion(params, ng, compDyn) # Why compute again? I think unneeded
		#
		# 	if FEASIBLE_REGION	is not False:
		# 		EXTENDED_FEASIBLE_REGION = Polygon(FEASIBLE_REGION)
		# 	reachable_feasible_polygon = np.array([])
		# 	reachability_polygon, computation_time_joint = joint_projection.project_polytope(params, None,
		# 																					 20. * np.pi / 180, 0.03)
		#
		# 	if reachability_polygon.size > 0:
		# 		preachability_polygon = Polygon(reachability_polygon)
		#
		# 		try:
		# 			reachable_feasible_polygon = EXTENDED_FEASIBLE_REGION.intersection(preachability_polygon)
		# 			reachable_feasible_polygon = np.array(reachable_feasible_polygon.exterior.coords)
		# 		except (AttributeError, TopologicalError) as e:
		# 			print("Shape not a Polygon.")
		# 			p.send_reachable_feasible_polygons(name, p.fillPolygon([]), foothold_params.option_index,
		# 											   foothold_params.ack_optimization_done)
		# 		else:
		# 			old_reachable_feasible_polygon = reachable_feasible_polygon
		# 			p.send_reachable_feasible_polygons(name, p.fillPolygon(reachable_feasible_polygon),
		# 											   foothold_params.option_index,
		# 											   foothold_params.ack_optimization_done)
		# 	else:
		# 		p.send_reachable_feasible_polygons(name, p.fillPolygon(old_reachable_feasible_polygon),
		# 										   foothold_params.option_index,
		# 										   foothold_params.ack_optimization_done)

		# FOOTHOLD PLANNING

		if (p.foothold_optimization):
			# print 'opt started?', foothold_params.optimization_started
			# print 'ack opt done', foothold_params.ack_optimization_done
			#        foothold_params.ack_optimization_done = True
			feasibleRegions = []
			#        print 'robot mass', params.robotMass
			if (foothold_params.foothold_optimization_started == False):
				foothold_params.ack_optimization_done = False

			''' The optimization-done-flag is set by the planner. It is needed to tell the controller whether the optimization 
			is finished or not. When this flag is true the controller will read the result of the optimization that has read 
			from the planner'''
			# print 'optimization done flag',foothold_params.ack_optimization_done
			''' The optimization-started-flag is set by the controller. It is needed to tell the planner that a new optimization should start.
			When this flag is true the planner (in jetleg) will start a new computation of the feasible region.'''
			# print 'optimization started flag', foothold_params.foothold_optimization_started
			if foothold_params.foothold_optimization_started and not foothold_params.ack_optimization_done:
				print('============================================================')
				print('current swing ', params.actual_swing)
				print('============================================================')

				params.getFutureStanceFeetFlags(p.hyq_debug_msg)

				print("FUTURE STANCE LEGS: ", params.stanceFeet)

				# Select foothold option with maximum feasible region from among all possible (default is 9) options
				foothold_params.option_index, stackedResidualRadius, feasibleRegions, mapFootHoldIdxToPolygonIdx = footHoldPlanning.selectMaximumFeasibleArea(
					foothold_params, params)

				if feasibleRegions is False:
					foothold_params.option_index = -1
				else:
					print('min radius ', foothold_params.minRadius, 'residual radius ', stackedResidualRadius)
					# print 'feet options', foothold_params.footOptions
					print('final index', foothold_params.option_index, 'index list', mapFootHoldIdxToPolygonIdx)

				foothold_params.ack_optimization_done = 1

				#         ONLY_ACTUATION, ONLY_FRICTION or FRICTION_AND_ACTUATION
				#        3 - FRICTION REGION
				constraint_mode_IP = 'ONLY_FRICTION'
				params.setConstraintModes([constraint_mode_IP]*params.getNoOfLegs())
				params.setNumberOfFrictionConesEdges(ng)

				params.contactsWF[params.actual_swing] = foothold_params.footOptions[
					foothold_params.option_index]  # variable doesn't change in framework. Needs fix

				#        uncomment this if you dont want to use the vars read in iterative_proJ_params
				#        params.setContactNormals(normals)
				#        params.setFrictionCoefficient(mu)
				#        params.setTrunkMass(trunk_mass)
				#        IP_points, actuation_polygons, comp_time = comp_dyn.support_region_bretl(stanceLegs, contacts, normals, trunk_mass)

				frictionRegion, actuation_polygons, computation_time = compDyn.iterative_projection_bretl(params)

				print('friction region is: ', frictionRegion)

				p.send_friction_region(name, p.fillPolygon(frictionRegion))

				# this sends the data back to ros that contains the foot hold choice (used for stepping) and the corrspondent region (that will be used for com planning TODO update with the real footholds)
				if (feasibleRegions is not False) and (np.size(feasibleRegions) is not 0):
					print('sending actuation region')
					p.send_feasible_polygons(name, p.fillPolygon(feasibleRegions[-1]), foothold_params.option_index,
											 foothold_params.ack_optimization_done)
				# print feasibleRegions[-1]
				else:
					# if it cannot compute anything it will return the frictin region
					p.send_feasible_polygons(name, p.fillPolygon(frictionRegion), foothold_params.option_index,
											 foothold_params.ack_optimization_done)

		# ORIENTATION PLANNING

		''' The optimization-done-flag is set by the planner. It is needed to tell the controller whether the optimization 
							is finished or not. When this flag is true the controller will read the result of the optimization that has read 
							from the planner'''
		''' The optimization-started-flag is set by the controller. It is needed to tell the planner that a new optimization should start.
		When this flag is true the planner (in jetleg) will start a new computation of the feasible region.'''
		if (orient_params.orient_optimization_started == False):
			orient_params.ack_optimization_done = False

		# print "orientation_start: ", orient_params.orient_optimization_started
		# print "optimization_done: ", orient_params.ack_optimization_done
		# print "internal optim_done: ", p.orient_ack_optimization_done

		feasible_regions = []
		if orient_params.orient_optimization_started and not orient_params.ack_optimization_done:
			params.getCurrentStanceFeetFlags(p.hyq_debug_msg)
			print("Stance Feet: ", params.stanceFeet)
			print("Current orientation is: ", params.getOrientation())
			print("Default orientation is: ", orient_params.get_default_orientation())
			print("Current CoM is: ", params.getCoMPosWF())
			# print "Target CoM is: ", orient_params.target_CoM_WF

			first_time = time.time()
			feasible_regions, min_distances, max_areas, optimal_orientation, optimal_index = \
				orient_planning_mp.optimize_orientation(orient_params, params)
			print("Optimization time: ", time.time() - first_time)
			optimization_success = True if optimal_index > -1 else False

			# time.sleep(5)
			orient_params.ack_optimization_done = p.orient_ack_optimization_done = True

			print("Optimal orientation is: ", optimal_orientation)

			print("Sending optimal orientation...")
			p.send_optimal_orientation(name, optimal_orientation, optimization_success,
									   orient_params.ack_optimization_done)

			line = LineString([params.getCoMPosWF(), orient_params.target_CoM_WF])
			line = np.array(list(line.coords))

			stanceFeet = params.getStanceFeet()
			nc = np.sum(stanceFeet)
			stanceID = params.getStanceIndex(stanceFeet)
			contactsWF = params.getContactsPosWF()

		# # Uncomment this section to plot orientation choices
		# plt.clf()
		# for count, feasible_region in enumerate(feasible_regions):
		#
		#     plt.subplot(5, 5, count + 1)
		#     plt.grid()
		#     plt.title("Distance: {} \n  Area: {}".format(min_distances[count], max_areas[count]))
		#
		#     """ contact points """
		#     for j in range(0,nc):  # this will only show the contact positions and normals of the feet that are defined to be in stance
		#         idx = int(stanceID[j])
		#         ''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
		#         h1 = plt.plot(contactsWF[idx, 0], contactsWF[idx, 1], 'ko', markersize=5, label='stance feet')
		#     if min_distances[count] is not False:
		#         try:
		#             polygon = np.array(feasible_regions[count].exterior.coords)
		#         except AttributeError:
		#             print "Shape not a Polygon."
		#         else:
		#             if count == optimal_index:
		#                 h2 = plotter.plot_polygon(np.transpose(polygon), '--g', 'Support Region')
		#             else:
		#                 h2 = plotter.plot_polygon(np.transpose(polygon), '--b', 'Support Region')
		#             plt.plot(np.transpose(line)[:][0], np.transpose(line)[:][1], linestyle='-', linewidth=4)
		#
		# # wm = plt.get_current_fig_manager()
		# # wm.window.state('zoomed')
		# # plt.show(block=False)
		# # plt.tight_layout()
		# # plt.subplots_adjust(left=0.12,bottom=0.05,right=0.9,top=0.95,wspace=0.4,hspace=0.4)
		# plt.draw()
		# plt.pause(0.001)
		# # if i < 2:
		# #     # Since plotting takes time, a reset signal from the framework could be missed.
		# #     orient_params.ack_optimization_done = False
		# #     i += 1
		#
		# if (p.orient_ack_optimization_done == False):
		#     orient_params.ack_optimization_done = False

	print('de registering...')
	p.deregister_node()