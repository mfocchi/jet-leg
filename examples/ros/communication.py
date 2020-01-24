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

#from dls_msgs.msg import SimpleDoubleArray, StringDoubleArray, Polygon3D, LegsPolygons
from dls_msgs.msg import  StringDoubleArray
from feasible_region.msg import RobotStates, Foothold
from feasible_region.msg import  Polygon3D, LegsPolygons
from shapely.geometry import Polygon

from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.maths.math_tools import Math
from jet_leg.maths.simple_iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.optimization.simple_foothold_planning_interface import FootholdPlanningInterface
from jet_leg.optimization import nonlinear_projection

from jet_leg.optimization.foothold_planning import FootHoldPlanning

from jet_leg.plotting.plotting_tools import Plotter
import matplotlib.pyplot as plt

#services
from feasible_region.srv import Config, ConfigResponse

np.set_printoptions(precision = 3, linewidth = 200, suppress = True)
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
        self.hyq_wbs = dict()
        self.hyq_debug_msg = RobotStates()
        self.hyq_footholds_msg = Foothold()
        self.actuation_polygon_topic_name = "/feasible_region/actuation_polygon"
        self.reachable_feasible_topic_name = "/feasible_region/reachble_feasible_region_polygon"
        self.support_region_topic_name = "/feasible_region/support_region"
        self.force_polygons_topic_name = "/feasible_region/force_polygons"
        self.robot_name = ros.get_param('/robot_name')
        # self.hyq_wbs_sub_name = "/"+self.robot_name+"/robot_states" not used
        print ros.get_namespace()
        self.sim_time  = 0.0


        self.plotFeasibleRegionFlag = False
        self.plotExtendedRegionFlag = False
        self.plotShrinkedActuationRegionFlag = False
        self.plotReachableFeasibleRegionFlag = False
        self.plotFrictionRegion = False
        self.plotForcePolygonsFlag = False

        self.com_optimization = False
        self.foothold_optimization = False
        self.com_optimization_type = 0

    def run(self):
        print "Run!"
        self.sub_clock = ros.Subscriber(self.clock_sub_name, Clock, callback=self._reg_sim_time, queue_size=1000)
        self.sub_actuation_params = ros.Subscriber(self.hyq_actuation_params_sub_name, RobotStates, callback=self.callback_hyq_debug, queue_size=5, buff_size=3000)
        self.sub_actuation_footholds_params = ros.Subscriber(self.hyq_actuation_footholds_params_sub_name, Foothold,
                                                   callback=self.callback_hyq_footholds, queue_size=5, buff_size=1500)
        self.pub_feasible_polygon = ros.Publisher(self.actuation_polygon_topic_name, Polygon3D, queue_size=10000)
        self.pub_reachable_feasible_polygon = ros.Publisher(self.reachable_feasible_topic_name, Polygon3D, queue_size=10000)
        self.pub_support_region = ros.Publisher(self.support_region_topic_name, Polygon3D, queue_size=1000)
        self.pub_force_polygons = ros.Publisher(self.force_polygons_topic_name, LegsPolygons, queue_size=1000)

    def config_server(self):

        #advertise service
        service = ros.Service('/feasible_region/set_config', Config, self.handle_set_feasible_region_config)
        print "Feasible Region Config Server initialized"


    def handle_set_feasible_region_config(self, req):
        print "Returning [vis opt: %s , com %s , comtype: %s foothold: %s]" % (req.visualization_options, req.com_optimization, req.com_optimization_type, req.foothold_optimization)
        self.com_optimization = req.com_optimization
        self.foothold_optimization = req.foothold_optimization
        self.com_optimization_type = req.com_optimization_type
        if (len(req.visualization_options) < 6):
            print  "wrong visualization option size is :", len(req.visualization_options)
            return ConfigResponse(False)


        self.plotFeasibleRegionFlag = req.visualization_options[0] == "1"
        self.plotShrinkedActuationRegionFlag = req.visualization_options[1] == "1"
        self.plotReachableFeasibleRegionFlag = req.visualization_options[2] == "1"
        self.plotExtendedRegionFlag = req.visualization_options[3] == "1"
        self.plotFrictionRegion = req.visualization_options[4] == "1"
        self.plotForcePolygonsFlag = req.visualization_options[5] == "1"
        return ConfigResponse(True)

    def _reg_sim_time(self, time):

        self.sim_time = time.clock.secs + time.clock.nsecs/1000000000.0
#        print("getting time")

    def _reg_sim_wbs(self, msg):
        self.hyq_wbs = copy.deepcopy(msg)

    def callback_hyq_debug(self, msg):
       self.hyq_debug_msg = copy.deepcopy(msg)

    def callback_hyq_footholds(self, msg):
       self.hyq_footholds_msg = copy.deepcopy(msg)

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

    def fillPolygon(self, polygon):
        # print 'polygon ', polygon
        num_actuation_vertices = np.size(polygon, 0)
        vertices = []

        for i in range(0, num_actuation_vertices):
            point = Point()
            point.x = polygon[i][0]
            point.y = polygon[i][1]
            point.z = polygon[i][2] #is the centroidal frame
            vertices = np.hstack([vertices, point])
        return vertices

def talker():
    # Create a communication thread
    p = HyQSim()
    p.start()  # Start thread
    p.register_node()
    # start config node
    p.config_server()

    compDyn = ComputationalDynamics(p.robot_name)
    footHoldPlanning = FootHoldPlanning(p.robot_name)
    joint_projection = nonlinear_projection.NonlinearProjectionBretl(p.robot_name)
    math = Math()



    name = "Actuation_region"
    force_polytopes_name = "force_polytopes"

    # Create parameter objects
    params = IterativeProjectionParameters()
    foothold_params = FootholdPlanningInterface()
    i = 0

    p.get_sim_wbs()
    # Save foothold planning and IP parameters from "debug" topic
    first = time.time()
    print p.hyq_debug_msg.tau_lim.data[0]
    params.getParamsFromRosDebugTopic(p.hyq_debug_msg)
    foothold_params.getParamsFromRosDebugTopic(p.hyq_footholds_msg)
    params.getFutureStanceFeetFlags(p.hyq_debug_msg)
    print "time: ", time.time() - first

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


    while not ros.is_shutdown():

        # Save foothold planning and IP parameters from "debug" topic
        first = time.time()
        params.getParamsFromRosDebugTopic(p.hyq_debug_msg)
        foothold_params.getParamsFromRosDebugTopic(p.hyq_footholds_msg)

        if p.com_optimization or p.foothold_optimization:
            params.getFutureStanceFeetFlags(p.hyq_debug_msg)
        else:
            params.getCurrentStanceFeetFlags(p.hyq_debug_msg)
        # print "CoM: ", params.getCoMPosWF()
        # print "time: ", time.time() - first

        if (p.plotFrictionRegion):
            constraint_mode_IP = 'ONLY_FRICTION'
            params.setConstraintModes([constraint_mode_IP,
                                           constraint_mode_IP,
                                           constraint_mode_IP,
                                           constraint_mode_IP])
            params.setNumberOfFrictionConesEdges(ng)
            frictionRegion, actuation_polygons, computation_time = compDyn.iterative_projection_bretl(params)
            p.send_friction_region(name, p.fillPolygon(frictionRegion))
            # print "frictionRegion: ", frictionRegion
            # print "friction time: ", computation_time

        if (p.plotFeasibleRegionFlag):
               # USE THIS ONLY TO PLOT THE ACTUAL REGION FOR A VIDEO FOR THE PAPER DO NOT USE FOR COM PLANNING
            constraint_mode_IP = 'FRICTION_AND_ACTUATION'
            # constraint_mode_IP = 'ONLY_ACTUATION'
            params.setConstraintModes([constraint_mode_IP,
                               constraint_mode_IP,
                               constraint_mode_IP,
                               constraint_mode_IP])
            params.setNumberOfFrictionConesEdges(ng)
            FEASIBLE_REGION, actuation_polygons_array, computation_time = compDyn.iterative_projection_bretl(params)
            # print "FEASIBLE_REGION: ", FEASIBLE_REGION
            # print"FEASIBLE_REGION computation_time", computation_time
            #safety measure use old when you cannot compute
            if FEASIBLE_REGION is not False:
               p.send_feasible_polygons(name, p.fillPolygon(FEASIBLE_REGION), foothold_params.option_index, foothold_params.ack_optimization_done)
               old_FEASIBLE_REGION = FEASIBLE_REGION
            else:
               print 'Could not compute the feasible region'
               p.send_feasible_polygons(name, p.fillPolygon(old_FEASIBLE_REGION), foothold_params.option_index,  foothold_params.ack_optimization_done)

        if (p.plotReachableFeasibleRegionFlag and not p.plotExtendedRegionFlag):
            reachability_polygon, computation_time_joint = joint_projection.project_polytope(params, None, 20. * np.pi / 180, 0.03)
            p.send_reachable_feasible_polygons(name, p.fillPolygon(reachability_polygon), foothold_params.option_index,
                                   foothold_params.ack_optimization_done)

        if (p.plotExtendedRegionFlag):
            EXTENDED_FEASIBLE_REGION = Polygon(FEASIBLE_REGION)
            reachable_feasible_polygon = np.array([])
            reachability_polygon, computation_time_joint = joint_projection.project_polytope(params, None, 20. * np.pi / 180, 0.03)
            if reachability_polygon.size > 0:
                preachability_polygon = Polygon(reachability_polygon)
                reachable_feasible_polygon = EXTENDED_FEASIBLE_REGION.intersection(preachability_polygon)

                try:
                    reachable_feasible_polygon = np.array(reachable_feasible_polygon.exterior.coords)

                except AttributeError:
                    print "Shape not a Polygon."
                    p.send_reachable_feasible_polygons(name, p.fillPolygon(old_reachable_feasible_polygon), foothold_params.option_index,
                                                   foothold_params.ack_optimization_done)
                else:
                    old_reachable_feasible_polygon = reachable_feasible_polygon
                    p.send_reachable_feasible_polygons(name, p.fillPolygon(reachable_feasible_polygon), foothold_params.option_index,
                                                   foothold_params.ack_optimization_done)

        # FOOTHOLD PLANNING
   
        if (p.foothold_optimization):
            #print 'opt started?', foothold_params.optimization_started
            #print 'ack opt done', foothold_params.ack_optimization_done
    #        foothold_params.ack_optimization_done = True
            feasibleRegions = []
    #        print 'robot mass', params.robotMass
            if (foothold_params.optimization_started == False):
                foothold_params.ack_optimization_done = False

            ''' The optimization-done-flag is set by the planner. It is needed to tell the controller whether the optimization 
            is finished or not. When this flag is true the controller will read the result of the optimization that has read 
            from the planner'''
            # print 'optimization done flag',foothold_params.ack_optimization_done
            ''' The optimization-started-flag is set by the controller. It is needed to tell the planner that a new optimization should start.
            When this flag is true the planner (in jetleg) will start a new computation of the feasible region.'''
            # print 'optimization started flag', foothold_params.optimization_started
            if foothold_params.optimization_started and not foothold_params.ack_optimization_done:
                print '============================================================'
                print 'current swing ', params.actual_swing
                print '============================================================'

                params.getFutureStanceFeetFlags(p.hyq_debug_msg)

                print "FUTURE STANCE LEGS: ", params.stanceFeet

                # Select foothold option with maximum feasible region from among all possible (default is 9) options
                foothold_params.option_index, stackedResidualRadius, feasibleRegions, mapFootHoldIdxToPolygonIdx = footHoldPlanning.selectMaximumFeasibleArea( foothold_params, params)

                if feasibleRegions is False:
                    foothold_params.option_index = -1
                else:
                    print 'min radius ', foothold_params.minRadius, 'residual radius ', stackedResidualRadius
                    #print 'feet options', foothold_params.footOptions
                    print 'final index', foothold_params.option_index, 'index list', mapFootHoldIdxToPolygonIdx

                foothold_params.ack_optimization_done = 1

                #         ONLY_ACTUATION, ONLY_FRICTION or FRICTION_AND_ACTUATION
                #        3 - FRICTION REGION
                constraint_mode_IP = 'ONLY_FRICTION'
                params.setConstraintModes([constraint_mode_IP,
                                           constraint_mode_IP,
                                           constraint_mode_IP,
                                           constraint_mode_IP])
                params.setNumberOfFrictionConesEdges(ng)

                params.contactsWF[params.actual_swing] = foothold_params.footOptions[foothold_params.option_index] # variable doesn't change in framework. Needs fix

                #        uncomment this if you dont want to use the vars read in iterative_proJ_params
                #        params.setContactNormals(normals)
                #        params.setFrictionCoefficient(mu)
                #        params.setTrunkMass(trunk_mass)
                #        IP_points, actuation_polygons, comp_time = comp_dyn.support_region_bretl(stanceLegs, contacts, normals, trunk_mass)

                frictionRegion, actuation_polygons, computation_time = compDyn.iterative_projection_bretl(params)

                print 'friction region is: ',frictionRegion

                p.send_friction_region(name, p.fillPolygon(frictionRegion))

                #this sends the data back to ros that contains the foot hold choice (used for stepping) and the corrspondent region (that will be used for com planning TODO update with the real footholds)
                if (feasibleRegions is not False) and (np.size(feasibleRegions) is not 0):
                    print 'sending actuation region'
                    p.send_feasible_polygons(name, p.fillPolygon(feasibleRegions[-1]), foothold_params.option_index, foothold_params.ack_optimization_done)
                    # print feasibleRegions[-1]
                else:
                    #if it cannot compute anything it will return the frictin region
                    p.send_feasible_polygons(name, p.fillPolygon(frictionRegion), foothold_params.option_index, foothold_params.ack_optimization_done)


        time.sleep(0.001)
        i+=1
        
    print 'de registering...'
    p.deregister_node()
        

if __name__ == '__main__':
    
    try:
        talker()

    except ros.ROSInterruptException:
        pass
    
        