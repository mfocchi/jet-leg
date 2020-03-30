# -*- coding: utf-8 -*-
"""
Created on Thu Oct 25 12:06:27 2018

@author: rorsolino
"""

import numpy as np
import time
from numpy import array
from shapely.geometry import Polygon, LineString

import matplotlib.pyplot as plt
from jet_leg.plotting.plotting_tools import Plotter
from jet_leg.maths.math_tools import Math
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.optimization.nonlinear_projection import NonlinearProjectionBretl
# from jet_leg.height_map import HeightMap

# from jet_leg.path_sequential_iterative_projection import PathIterativeProjection
from jet_leg.maths.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.optimization.orientation_planning_interface import OrientationPlanningInterface
from jet_leg.optimization.orientation_planning import OrientationPlanning

''' MAIN '''

plt.close('all')
start_t_IPVC = time.time()

robot_name = 'hyq'
math = Math()
comp_dyn = ComputationalDynamics(robot_name)
nonlinear_proj = NonlinearProjectionBretl(robot_name)
# pathIP = PathIterativeProjection()
params = IterativeProjectionParameters()

# ONLY_ACTUATION, ONLY_FRICTION or FRICTION_AND_ACTUATION
constraint_mode = ['FRICTION_AND_ACTUATION',
				   'FRICTION_AND_ACTUATION',
				   'FRICTION_AND_ACTUATION',
				   'FRICTION_AND_ACTUATION']

comWF = np.array([1.069126240, -0.059787876, 0.58205050562])
comBF = np.array([0.009007794, -0.0037249425, -0.0436662037])
rpy = np.array([0.176160568, -0.119195807, -0.007672352])

""" contact points in the World Frame"""
LF_foot = np.array([1.518, 0.291, 0.279])
RF_foot = np.array([1.428, -0.342, 0.022])
LH_foot = np.array([0.791, 0.308, 0.084])
RH_foot = np.array([0.702, -0.339, 0.021])

contactsWF = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))

''' parameters to be tuned'''
trunk_mass = 86.574
mu = 0.5
plane_normal = (-0.127544936, -0.226259757, 0.96568049)

''' stanceFeet vector contains 1 is the foot is on the ground and 0 if it is in the air'''
stanceFeet = [1,1,1,1]

number_of_contacts = 4
stanceID = params.getStanceIndex(stanceFeet)

''' now I define the normals to the surface of the contact points. By default they are all vertical now'''
# axisZ= array([[0.0], [0.0], [1.0]])
#
# # n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))  # LF
# # n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))  # RF
# # n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))  # LH
# # n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))  # RH
# # normals = np.vstack([n1, n2, n3, n4])
n1 = [-0.2587954, 0.001343170, 0.9659312]
n2 = [0., 0.0, 1.]
n3 = [-0.2587954, 0.001343170, 0.9659312]
n4 = [0., 0., 1.]
normals = np.array([n1, n2, n3, n4])

''' joint position limits for each leg (this code assumes a hyq-like design, i.e. three joints per leg)
HAA = Hip Abduction Adduction
HFE = Hip Flextion Extension
KFE = Knee Flextion Extension
'''
LF_q_lim_max = [0.44, 1.2217, -0.3491]  # HAA, HFE, KFE
LF_q_lim_min = [-1.22, -0.8727, -2.4435]  # HAA, HFE, KFE
RF_q_lim_max = [0.44, 1.2217, -0.3491]  # HAA, HFE, KFE
RF_q_lim_min = [-1.22, -0.8727, -2.4435]  # HAA, HFE, KFE
LH_q_lim_max = [0.44, 0.8727, 2.4435]  # HAA, HFE, KFE
LH_q_lim_min = [-1.22, -1.2217, 0.3491]  # HAA, HFE, KFE
RH_q_lim_max = [0.44, 0.8727, 2.4435]  # HAA, HFE, KFE
RH_q_lim_min = [-1.22, -1.2217, 0.3491]  # HAA, HFE, KFE
joint_limits_max = np.array([LF_q_lim_max, RF_q_lim_max, LH_q_lim_max, RH_q_lim_max])
joint_limits_min = np.array([LF_q_lim_min, RF_q_lim_min, LH_q_lim_min, RH_q_lim_min])

''' torque limits for each leg (this code assumes a hyq-like design, i.e. three joints per leg)
HAA = Hip Abduction Adduction
HFE = Hip Flextion Extension
KFE = Knee Flextion Extension
'''
LF_tau_lim = [130, 140, 160] # HAA, HFE, KFE
RF_tau_lim = [130, 140, 170] # HAA, HFE, KFE
LH_tau_lim = [130, 140, 170] # HAA, HFE, KFE
RH_tau_lim = [130, 140, 170] # HAA, HFE, KFE
torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim])

''' extForceW is an optional external pure force (no external torque for now) applied on the CoM of the robot.'''
extForceW = np.array([0.0, 0.0, 0.0]) # units are Nm
extTorqueW = np.array([0.0, 0.0, 0.0]) # units are Nm

'''You now need to fill the 'params' object with all the relevant
    informations needed for the computation of the IP'''

params.setContactsPosWF(contactsWF)
params.setCoMPosWF(comWF)
params.setCoMPosBF(comBF)
params.setOrientation(rpy)
params.setTorqueLims(torque_limits)
params.setJointLimsMax(joint_limits_max)
params.setJointLimsMin(joint_limits_min)
params.setActiveContacts(stanceFeet)
params.setConstraintModes(constraint_mode)
params.setContactNormals(normals)
params.setFrictionCoefficient(mu)
params.setNumberOfFrictionConesEdges(4)
params.setTotalMass(trunk_mass)
params.externalForceWF = extForceW
params.externalTorqueWF = extTorqueW# params.externalForceWF is actually used anywhere at the moment


#-----------------------------------------------------------------------------------------------------
# Single Projection

com_check = np.array([1.22566129, -0.06287036, 0.76374874])
com_check = None

polygon, computation_time = nonlinear_proj.project_polytope(params, None, 20. * np.pi / 180, 0.03)

plt.figure()
plotter = Plotter()
for j in range(0, number_of_contacts):  # this will only show the contact positions and normals of the feet that are defined to be in stance
    idx = int(stanceID[j])
    ''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
    h1 = plt.plot(contactsWF[idx, 0], contactsWF[idx, 1], 'ko', markersize=15, label='stance feet')
h2 = plotter.plot_polygon(np.transpose(polygon), '--b', 'Iterative Projection')

#-----------------------------------------------------------------------------------------------------
# Orientation Planning Parameters

# default_orientation = np.array([(0.2117821, -0.10354096, -0.01358345)])
default_orientation = np.array([0.208109788, -0.10347280000, -0.01423768505])
# target_CoM_WF = np.array([1.1642021, 0.0314649, 0.61342885])
target_CoM_WF = np.array([1.16566950, 0.0289022953, 0.608773453])
no_of_angle_choices = 4

orient_params = OrientationPlanningInterface()

orient_params.default_orientation = default_orientation
orient_params.target_CoM_WF = target_CoM_WF
orient_params.no_of_angle_choices = no_of_angle_choices

planning = OrientationPlanning(robot_name)

#-----------------------------------------------------------------------------------------------------
# Optimization

first_time = time.time()
feasible_regions, min_distances, optimal_orientation, optimal_index = planning.optimize_orientation(orient_params, params)
print "total time: ", time.time() - first_time

line = LineString([params.getCoMPosWF(), orient_params.target_CoM_WF])
line = np.array(list(line.coords))

#-----------------------------------------------------------------------------------------------------
#Plotting Optimization

plt.figure()
plotter = Plotter()

for count, feasible_region in enumerate(feasible_regions):

	plt.subplot(5, 5, count+1)
	plt.grid()
	plt.title("Distance: {}".format(min_distances[count]))
	plt.legend()

	""" contact points """
	for j in range(0,number_of_contacts):  # this will only show the contact positions and normals of the feet that are defined to be in stance
		idx = int(stanceID[j])
		''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
		h1 = plt.plot(contactsWF[idx, 0], contactsWF[idx, 1], 'ko', markersize=5, label='stance feet')
	if min_distances[count]:
		try:
			polygon = np.array(feasible_regions[count].exterior.coords)
		except AttributeError:
			print "Shape not a Polygon."
		else:
			if count == optimal_index:
				h2 = plotter.plot_polygon(np.transpose(polygon), '--g', 'Support Region')
			else:
				h2 = plotter.plot_polygon(np.transpose(polygon), '--b', 'Support Region')
			plt.plot(np.transpose(line)[:][0], np.transpose(line)[:][1], linestyle='-', linewidth=4)

print "Current orientation is: ", [ang*180./np.pi for ang in rpy]
print "Default orientation is: ", [ang*180./np.pi for ang in default_orientation]
print "Optimal orientation is: ", [ang*180./np.pi for ang in optimal_orientation], " with distance of: ", min_distances[optimal_index]

plt.show()