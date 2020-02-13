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
# import multiprocessing as mp
import pathos.multiprocessing as mp # Need to use Pathos to pass a class method to the pool map

from copy import deepcopy
import itertools
from functools import partial

from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.optimization.nonlinear_projection import NonlinearProjectionBretl
from jet_leg.maths.computational_geometry import ComputationalGeometry
from jet_leg.optimization.foothold_planning_interface import FootholdPlanningInterface
from jet_leg.maths.math_tools import Math
from jet_leg.robots.dog_interface import DogInterface
from jet_leg.dynamics.rigid_body_dynamics import RigidBodyDynamics
from shapely.geometry import Polygon, LineString, LinearRing
from shapely.geos import TopologicalError

stderr = sys.stderr
sys.stderr = open(os.devnull, 'w')
sys.stderr = stderr


class OrientationPlanningMultiProcess:
	def __init__(self, robot_name):
		self.robotName = robot_name
		# self.compGeo = ComputationalGeometry()
		self.compDyn = ComputationalDynamics(self.robotName)
		self.kinProj = NonlinearProjectionBretl(self.robotName)

		self.pool = mp.ProcessingPool(mp.cpu_count()-7) # 5 cores seems to be the fastest
		# self.math = Math()
		# self.dog = DogInterface()
		# self.rbd = RigidBodyDynamics()

	def optimize_orientation(self, orient_plan_params, params):

		ng = 4
		params.setConstraintModes(['FRICTION_AND_ACTUATION',
								   'FRICTION_AND_ACTUATION',
								   'FRICTION_AND_ACTUATION',
								   'FRICTION_AND_ACTUATION'])

		params.setNumberOfFrictionConesEdges(ng)

		delta_angle = 0.0872665 # Increment of 5 degrees
		first_roll = -0.174533  # -10 degrees
		first_pitch = -0.174533  # -10 degrees

		roll_list = [first_roll + i*delta_angle for i in range(orient_plan_params.no_of_angle_choices+1)]
		pitch_list = [first_pitch + i*delta_angle for i in range(orient_plan_params.no_of_angle_choices+1)]

		# processes = []
		# for i in range(10):
		# 	t = multiprocessing.Process(target=self.single_optimize_orientation, args=(i,))
		# 	processes.append(t)
		# 	t.start()

		# optimal_orientation = mp.Value('i', orient_plan_params.get_default_orientation())
		# optimal_distance = mp.Value('f', -1)
		# old_area = mp.Value('f', -1)
		# optimal_index = mp.Value('i', -1)
		new_angles = itertools.product(roll_list,pitch_list)


		func = partial(self.single_optimize_orientation, params, orient_plan_params)
		results_zipped  = self.pool.map(func, new_angles)
		# self.pool.join()
		# self.pool.close()
		reachable_regions, min_distances, max_areas = zip(*results_zipped)
		print "reachable_regions: ", reachable_regions
		print "min_distances: ", min_distances
		print "max_areas: ", max_areas

		# for one_process in processes:
		# 	one_process.join()

		default_orientation = optimal_orientation = orient_plan_params.get_default_orientation()
		old_distance = -1
		old_area = -1
		optimal_index = -1
		new_angles = itertools.product(roll_list, pitch_list)

		# Compute optimal orientation and its index
		# for index, angles in enumerate(new_angles):
		for index, angle in enumerate(new_angles):

			print "index: ", index
			reachable_region = reachable_regions[index]
			distance = min_distances[index]
			area = max_areas[index]

			if reachable_region is False:
				continue
			if distance <= -1:
				continue

			new_orientation = default_orientation + np.array([angle[0], angle[1], 0])

			if distance - old_distance > 0.02 or (abs(distance - old_distance) < 0.02 and area > old_area):
				optimal_orientation = new_orientation
				optimal_index = index
				old_distance = distance
				old_area = area

		return reachable_regions, min_distances, max_areas, optimal_orientation, optimal_index


	def single_optimize_orientation(self, params, orient_plan_params, new_angle):

		new_roll = new_angle[0]
		new_pitch = new_angle[1]

		# Initialize return variables
		default_orientation = orient_plan_params.get_default_orientation()
		target_CoM_WF = orient_plan_params.get_target_CoM_WF()

		new_orientation = default_orientation + np.array([new_roll, new_pitch, 0])
		# This is all to get the contactsBF and use it then in joint projection (to find contactsWF)
		params.setOrientation(new_orientation)


		# Compute feasible region, reachable region, and intersection

		feasible_region, actuation_polygons, computation_time = self.compDyn.try_iterative_projection_bretl(params)
		if feasible_region is False:
			return False, False, "FR False"
			# feasible_regions.append(False)
			# min_distances.append(False)
			# max_areas.append("FR False")
			# # print "Feasible region is false"
			# continue

		reachability_polygon, computation_time_joint = self.kinProj.project_polytope(params, target_CoM_WF, 25 * np.pi / 180, 0.03)
		if reachability_polygon.size <= 0:
			return False, False, "Reach False"
			# feasible_regions.append(False)
			# min_distances.append(False)
			# max_areas.append("Reach False")
			# # print "Kinematic reach region is false"
			# continue

		extended_feasible_region = Polygon(feasible_region)
		p_reachability_polygon = Polygon(reachability_polygon)
		try:
			reachable_feasible_polygon = extended_feasible_region.intersection(p_reachability_polygon)
		except TopologicalError:
			return False, False, "Intersec Fail"
			# feasible_regions.append(False)
			# min_distances.append(False)
			# max_areas.append("Intersec Fail")
			# continue

		area = reachable_feasible_polygon.area

		# Exclude if path from current CoM to target one lies outside region
		l_CoM_path = LineString([params.getCoMPosWF(), target_CoM_WF])
		try:
			if not l_CoM_path.within(reachable_feasible_polygon):
				return reachable_feasible_polygon, -1, area
				# feasible_regions.append(reachable_feasible_polygon)
				# min_distances.append(-1)
				# max_areas.append(area)
				# # print "Path outside of region"
				# continue
		except TopologicalError:
			return False, False, "Extended !=Polygon"
			# feasible_regions.append(False)
			# min_distances.append(False)
			# max_areas.append("Extended !=Polygon")
			# continue


		# Compute minimum distance between path and polygon
		r_reachable_feasible_polygon = LinearRing(reachable_feasible_polygon.exterior.coords)
		distance = r_reachable_feasible_polygon.distance(l_CoM_path)

		return reachable_feasible_polygon, distance, area

		# feasible_regions.append(reachable_feasible_polygon)
		# min_distances.append(distance)
		# max_areas.append(area)


			# optimal_orientation = new_orientation
			# optimal_index = len(feasible_regions) - 1
			# print "optimal_index: ", optimal_index
			# print "better_orientation: ", new_orientation, ", better_distance: ", distance
			# print "better_region: ", np.array(reachable_feasible_polygon.exterior.coords)