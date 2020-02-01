#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Oct  3 13:38:45 2018

@author: Abdelrahman Abdalla
"""

# import copy
import numpy as np
import os

# import rospy as ros
import sys
# import time
# import threading
#
from copy import deepcopy

from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.optimization.nonlinear_projection import NonlinearProjectionBretl
from jet_leg.maths.computational_geometry import ComputationalGeometry
from jet_leg.optimization.foothold_planning_interface import FootholdPlanningInterface
from jet_leg.maths.math_tools import Math
from jet_leg.robots.dog_interface import DogInterface
from jet_leg.dynamics.rigid_body_dynamics import RigidBodyDynamics
from shapely.geometry import Polygon, LineString, LinearRing

stderr = sys.stderr
sys.stderr = open(os.devnull, 'w')
sys.stderr = stderr


class Orientation_Planning:
	def __init__(self, robot_name):
		self.robotName = robot_name
		self.compGeo = ComputationalGeometry()
		self.compDyn = ComputationalDynamics(self.robotName)
		self.kinProj = NonlinearProjectionBretl(self.robotName)
		self.footPlanning = FootholdPlanningInterface()
		self.math = Math()
		self.dog = DogInterface()
		self.rbd = RigidBodyDynamics()

	def optimize_orientation(self, orient_plan_params, params):
		ng = 4
		params.setConstraintModes(['FRICTION_AND_ACTUATION',
								   'FRICTION_AND_ACTUATION',
								   'FRICTION_AND_ACTUATION',
								   'FRICTION_AND_ACTUATION'])

		params.setNumberOfFrictionConesEdges(ng)
		#
		# params.setCoMPosWF(footPlanningParams.com_position_to_validateW)

		#        print numberOfFeetOptions

		alpha = 0.0872665 # Increment of 5 degrees
		target_CoM_WF = orient_plan_params.get_target_CoM_WF()

		# Initialize return variables
		optimal_orientation = orient_plan_params.get_default_orientation()
		optimal_distance = old_distance = -1
		feasible_regions = []
		min_distances = []

		delta_roll = -0.174533  # -10 degrees
		for roll_index in range(orient_plan_params.no_of_angle_choices+1):
			delta_pitch = -0.174533  # -10 degrees
			for pitch_index in range(orient_plan_params.no_of_angle_choices+1):

				print "loop: roll ", roll_index, " pitch ", pitch_index
				new_orientation = orient_plan_params.get_default_orientation() + [delta_roll, delta_pitch, 0]
				print "roll value: ", new_orientation[0], " pitch value: ", new_orientation[1], "yaw value: ", new_orientation[2]
				# This is all to get the contactsBF and use it then in joint projection (to find contactsWF)
				params.setOrientation(new_orientation)

				# Compute feasible region, reachable region, and intersection

				feasible_region, actuation_polygons, computation_time = self.compDyn.iterative_projection_bretl(params)
				if feasible_region is False:
					feasible_regions.append(False)
					min_distances.append(False)
					print "Feasible region is false"
					continue

				reachability_polygon, computation_time_joint = self.kinProj.project_polytope(params, target_CoM_WF, 20. * np.pi / 180, 0.03)
				if reachability_polygon.size <= 0:
					feasible_regions.append(False)
					min_distances.append(False)
					print "Kinematic reach region is false"
					continue

				extended_feasible_region = Polygon(feasible_region)
				p_reachability_polygon = Polygon(reachability_polygon)
				reachable_feasible_polygon = extended_feasible_region.intersection(p_reachability_polygon)

				# Exclude if path from current CoM to target one lies outside region
				l_CoM_path = LineString([params.getCoMPosWF(), target_CoM_WF])
				if not l_CoM_path.within(reachable_feasible_polygon):
					feasible_regions.append(reachable_feasible_polygon)
					min_distances.append(-1)
					print "Path outside of region"
					continue


				# Compute minimum distance between path and polygon
				r_reachable_feasible_polygon = LinearRing(reachable_feasible_polygon.exterior.coords)
				distance = r_reachable_feasible_polygon.distance(l_CoM_path)

				feasible_regions.append(reachable_feasible_polygon)
				min_distances.append(distance)

				# Update optimal distance
				if distance >= old_distance:
					optimal_orientation, optimal_distance = new_orientation, distance
					print "better_orientation: ", new_orientation, ", better_distance: ", distance
					print "better_region: ", np.array(reachable_feasible_polygon.exterior.coords)
					old_distance = distance

				delta_pitch += alpha
			delta_roll += alpha

		return optimal_orientation, optimal_distance, feasible_regions, min_distances
		# newArea = self.compGeo.computePolygonArea(IAR)