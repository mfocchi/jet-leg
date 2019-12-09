# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Abdelrahman Abdalla
"""

from numpy import array, cos, sin, cross, pi
from scipy.linalg import norm
from scipy.spatial import ConvexHull
from jet_leg.maths.geometry import Geometry
import time
import numpy as np

from jet_leg.maths.math_tools import Math
from jet_leg.kinematics.kinematics_interface import KinematicsInterface


class Vertex:

	def __init__(self, p):
		self.x = p[0]
		self.y = p[1]


class Polygon:

	def from_vertices(self, v1, v2, v3):
		v1.next = v2
		v2.next = v3
		v3.next = v1
		self.vertices = [v1, v2, v3]

	def sort_vertices(self):
		"""
		Export vertices starting from the left-most and going clockwise.
		Assumes all vertices are on the positive halfplane.
		"""
		minsd = 1e10
		ibottom = 0
		for i in range(len(self.vertices)):
			v = self.vertices[i]
			if (v.y + v.next.y) < minsd:
				ibottom = i
				minsd = v.y + v.next.y
		for v in self.vertices:
			v.checked = False
		vcur = self.vertices[ibottom]
		newvertices = []
		while not vcur.checked:
			vcur.checked = True
			newvertices.append(vcur)
			vcur = vcur.next
		newvertices.reverse()
		vfirst = newvertices.pop(-1)
		newvertices.insert(0, vfirst)
		self.vertices = newvertices

	def export_vertices(self, threshold=1e-2):
		export_list = [self.vertices[0]]
		for i in range(1, len(self.vertices) - 1):
			vcur = self.vertices[i]
			vlast = export_list[-1]
			if norm([vcur.x - vlast.x, vcur.y - vlast.y]) > threshold:
				export_list.append(vcur)
		export_list.append(self.vertices[-1])  # always add last vertex
		return export_list


class NonlinearProjectionBretl:
	def __init__(self, robot_name):
		self.robotName = robot_name
		self.kin = KinematicsInterface(self.robotName)
		self.geom = Geometry()
		self.math = Math()

	# @property
	# def params(self):
	# 	return self.params
	#
	# @params.setter
	# def params(self, _params):
	# 	self.params = _params

	def getcontactsBF(self, params, comPositionWF):

		comPositionBF = params.getCoMPosBF()
		contactsWF = params.getContactsPosWF()
		contactsBF = np.zeros((4, 3))
		rpy = params.getOrientation()

		for j in np.arange(0, 4):
			j = int(j)
			contactsBF[j, :] = np.add(np.dot(self.math.rpyToRot(rpy[0], rpy[1], rpy[2]),
															(contactsWF[j, :] - comPositionWF)),
															comPositionBF)

		return contactsBF

	def compute_vertix(self, com_pos_x, com_pos_y, com_pos_z, params, theta, min_dir_step, max_iter):
		"""
		Compute vertix of projected polygon in vdir direction.

		Solves nonlinear optimization problem by iteratively testing
		over beta-spaced possible CoM positions along vdir direction

		Returns
		-------
		poly: Polygon Output polygon.
		"""

		# Search for a CoM position in direction vdir

		vdir = array([cos(theta), sin(theta), 0])

		c_t = [com_pos_x, com_pos_y, com_pos_z]  # CoM to be tested
		cxy_opt = c_t[:2]  # Optimal CoM so far
		foot_vel = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])

		i = 0

		# Find closest feasible and non-feasible points to start bisection algorithm

		# Initial step is large to speed up initial search
		# However, region is not convex so too large of a step can be non-conservative and skip close vertices
		dir_step = 0.1
		c_t_feasible = True  # Flag for out of limits search

		while c_t_feasible and i < max_iter:

			c_t += dir_step * vdir  # iterate along direction vector by step cm
			contactsBF = self.getcontactsBF(params, c_t)
			q = self.kin.inverse_kin(contactsBF, foot_vel)

			if self.kin.isOutOfJointLims(q, params.getJointLimsMax(), params.getJointLimsMin()):
				c_t_feasible = False
			else:
				cxy_opt = [c_t[0], c_t[1]]
				dir_step += dir_step / 2

			i += 1

		# Perform bisection algorithm using two points from previous step
		# Switch direction to go back to feasible region
		dir_step = -dir_step / 2

		while abs(dir_step) >= min_dir_step and i < max_iter:

			old_c_t_feasible = c_t_feasible
			c_t += dir_step * vdir
			contactsBF = self.getcontactsBF(params, c_t)
			q = self.kin.inverse_kin(contactsBF, foot_vel)

			# If new point is on the same side (feasible or infeasible region) as last point, continue in same direction
			if not self.kin.isOutOfJointLims(q, params.getJointLimsMax(), params.getJointLimsMin()):
				c_t_feasible = True
				cxy_opt = [c_t[0], c_t[1]]
			else:
				c_t_feasible = False

			if c_t_feasible == old_c_t_feasible:
				dir_step = dir_step/2
			else:
				dir_step = -dir_step / 2

			i += 1
			# print "new dir_step: ", dir_step

		return cxy_opt

	def compute_polygon(self, params, theta_step, dir_step, max_iter):
		"""
		Compute projected Polytope.

		Returns
		-------
		vertices: list of arrays List of vertices of the
				projected polygon.
		"""

		comPosWF_0 = params.getCoMPosWF()
		polygon = []

		# Check if current configuration is already feasible
		contactsBF = self.getcontactsBF(params, comPosWF_0)
		foot_vel = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])
		if self.kin.isOutOfWorkSpace(contactsBF, params.getJointLimsMax(), params.getJointLimsMin(), foot_vel):
			print "Couldn't compute a reachable region! Current configuration is already out of joint limits!"
		else:
			# polygon = Polygon()
			theta = 0

			while theta < 360. * pi /180.:
				# print "theta: ", theta
				
				# Compute region for the current CoM position (in world frame)
				v = Vertex(self.compute_vertix(comPosWF_0[0], comPosWF_0[1], comPosWF_0[2], params, theta, dir_step, max_iter))
				polygon.append(v)

				theta += theta_step  # increase search angle by step rad

		return polygon

	def project_polytope(self, params, com_wf_check=None, theta_step=20. * pi / 180, dir_step=0.03, max_iter=1000):
		"""
		Project a polytope into a 2D polygon using the incremental projection
		algorithm from [Bretl08].

		Returns
		-------
		vertices: list of arrays List of vertices of the
				projected polygon.
		"""

		ip_start = time.time()
		polygon = self.compute_polygon(params, theta_step, dir_step, max_iter)
		# polygon.sort_vertices()
		# vertices_list = polygon.export_vertices()
		vertices_list = polygon
		vertices = [array([v.x, v.y]) for v in vertices_list]
		compressed_vertices = np.compress([True, True], vertices, axis=1)
		# print compressed_vertices
		# try:
		# 	hull = ConvexHull(compressed_vertices)
		# except Exception as err:
		# 	print("QHull type error: " + str(err))
		# 	print("matrix to compute qhull:", compressed_vertices)
		# 	return False, False, False
		#
		# compressed_hull = compressed_vertices[hull.vertices]
		# compressed_hull = self.geom.clockwise_sort(compressed_hull)
		# vertices = compressed_hull
		# compressed_vertices = self.geom.clockwise_sort(compressed_vertices)
		vertices = compressed_vertices
		computation_time = (time.time() - ip_start)

		if com_wf_check is not None:
			foot_vel = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])
			contactsBF_check = self.getcontactsBF(params, com_wf_check)
			# print "q: ", self.kin.inverse_kin(contactsBF_check, foot_vel)
			if self.kin.isOutOfWorkSpace(contactsBF_check, params.getJointLimsMax(), params.getJointLimsMin(), foot_vel):
				print "Ouch!"
			else:
				print "Feasible!"
		# print "size: ", len(vertices)
		return vertices, computation_time
