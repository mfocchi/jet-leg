# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""

import numpy as np

from numpy import array
from jet_leg.plotting.plotting_tools import Plotter
import random
from jet_leg.maths.math_tools import Math
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.maths.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.optimization import nonlinear_projection

import matplotlib.pyplot as plt
from jet_leg.plotting.arrow3D import Arrow3D

plt.close('all')
math = Math()

''' Set the robot's name (either 'hyq', 'hyqreal' or 'anymal')'''
robot_name = 'hyq'

''' number of generators, i.e. rays/edges used to linearize the friction cone '''
ng = 4

'''
possible constraints for each foot:
 ONLY_ACTUATION = only joint-torque limits are enforces
 ONLY_FRICTION = only friction cone constraints are enforced
 FRICTION_AND_ACTUATION = both friction cone constraints and joint-torque limits
'''
constraint_mode_IP = ['FRICTION_AND_ACTUATION',
					  'FRICTION_AND_ACTUATION',
					  'FRICTION_AND_ACTUATION',
					  'FRICTION_AND_ACTUATION']

# number of decision variables of the problem
# n = nc*6
''' VALUES FOR FIRST PLOT'''

comWF = np.array([-0.03730262638362422, 0.0005479786142273393, 1.0897889225484119])  # pos of COM in world frame w. trunk controller
comBF = np.array([0.010018654610937781, 0.00022151741616971491, -0.04438858039131427])  # pos of COM in body frame w. trunk controller
rpy = np.array([-0.0006119917928183459, -0.27603816940394355, -0.0003406566641193891])  # orientation of body frame w. trunk controller

""" contact points in the World Frame"""
LF_foot = np.array([0.434,  0.327,  0.682])  # Starting configuration w.o. trunk controller
RF_foot = np.array([0.434, -0.328,  0.683])
LH_foot = np.array([-0.232,  0.331,  0.504])
RH_foot = np.array([-0.233, -0.33,   0.504])

contactsWF = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))

''' parameters to be tuned'''
trunk_mass = 86.574
mu = 0.8

''' stanceFeet vector contains 1 is the foot is on the ground and 0 if it is in the air'''
stanceFeet = [1, 1, 1, 1]

randomSwingLeg = random.randint(0, 3)
tripleStance = False  # if you want you can define a swing leg using this variable
if tripleStance:
	print 'Swing leg', randomSwingLeg
	stanceFeet[randomSwingLeg] = 0
print 'stanceLegs ', stanceFeet

''' now I define the normals to the surface of the contact points. By default they are all vertical now'''
axisZ = array([[0.0], [0.0], [1.0]])

# n1 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # LF
# n2 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # RF
# n3 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # LH
# n4 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # RH
n1 = np.array([-0.25881867,  0.        ,  0.96592593])  # Starting configuration w.o. trunk controller
n2 = np.array([-0.25881867,  0.        ,  0.96592593])
n3 = np.array([-0.25881867,  0.        ,  0.96592593])
n4 = np.array([-0.25881867,  0.        ,  0.96592593])
normals = np.vstack([n1, n2, n3, n4])

com_vertical_shift =  0.533541922849

planeNormal =  array([-0.25881867,  0.        ,  0.96592593]) # -15 degrees ramp

''' torque limits for each leg (this code assumes a hyq-like design, i.e. three joints per leg)
HAA = Hip Abduction Adduction
HFE = Hip Flextion Extension
KFE = Knee Flextion Extension
'''
LF_tau_lim = [160.0, 160.0, 160.0]  # HAA, HFE, KFE
RF_tau_lim = [160.0, 160.0, 160.0]  # HAA, HFE, KFE
LH_tau_lim = [160.0, 160.0, 160.0]  # HAA, HFE, KFE
RH_tau_lim = [160.0, 160.0, 160.0]  # HAA, HFE, KFE
torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim])

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

comp_dyn = ComputationalDynamics(robot_name)

'''You now need to fill the 'params' object with all the relevant 
    informations needed for the computation of the IP'''
params = IterativeProjectionParameters()
projection = nonlinear_projection.NonlinearProjectionBretl(robot_name)

params.setContactsPosWF(contactsWF)
params.setCoMPosWF(comWF)
params.setCoMPosBF(comBF)
params.setOrientation(rpy)
params.setTorqueLims(torque_limits)
params.setJointLimsMax(joint_limits_max)
params.setJointLimsMin(joint_limits_min)
params.setActiveContacts(stanceFeet)
params.setConstraintModes(constraint_mode_IP)
params.setContactNormals(normals)
params.setFrictionCoefficient(mu)
params.setNumberOfFrictionConesEdges(ng)
params.setTotalMass(trunk_mass)
params.set_plane_normal(planeNormal)
params.com_vertical_shift = com_vertical_shift

''' compute iterative projection 
Outputs of "iterative_projection_bretl" are:
IP_points = resulting 2D vertices
actuation_polygons = these are the vertices of the 3D force polytopes (one per leg)
computation_time = how long it took to compute the iterative projection
'''
polygon, computation_time = projection.project_polytope(params, None, 10. * np.pi / 180, 0.02)

ax = plt.subplot(111)

nc = np.sum(stanceFeet)
stanceID = params.getStanceIndex(stanceFeet)
# plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15)

plotter = Plotter()

''' 2D figure '''
# plt.figure()
plt.plot(contactsWF[0:nc, 0], contactsWF[0:nc, 1], 'ko', markersize=15, label=r'$\mathrm{Feet}$')
h4 = plotter.plot_polygon(np.transpose(polygon), 'b')

plt.grid()
plt.xlim([-0.5,0.5])
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.legend(bbox_to_anchor=(1.0, 0.87))
plt.savefig("foo1.pdf")
#------------------------------------------------------------------

''' CHANGE VALUES FOR SECOND PLOT'''
comWF = np.array([-0.0617166764298456, 0.000579159140332129, 1.0819874730160357])  # pos of COM in world frame w. trunk controller
comBF = np.array([0.015209617968225717, 0.00022532648579280272, -0.04494004792017108])  # pos of COM in body frame w. trunk controller
rpy = np.array([-0.0007251391263700892, 0.0009821278594368248, -0.0005346730627743704])  # orientation of body frame w. trunk controller

""" contact points in the World Frame"""
LF_foot = np.array([0.428,  0.324,  0.681])  # Starting configuration w.o. trunk controller
RF_foot = np.array([0.428, -0.325,  0.681])
LH_foot = np.array([-0.238,  0.332,  0.502])
RH_foot = np.array([-0.238, -0.332,  0.503])

contactsWF = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))

# planeNormal =  array([0.0, 0.0, 1.0])
com_vertical_shift =  0.53227402184

params.setContactsPosWF(contactsWF)
params.setCoMPosWF(comWF)
params.setCoMPosBF(comBF)
params.setOrientation(rpy)
params.setContactNormals(normals)
params.set_plane_normal(planeNormal)
params.com_vertical_shift = com_vertical_shift

''' compute iterative projection 
Outputs of "iterative_projection_bretl" are:
IP_points = resulting 2D vertices
actuation_polygons = these are the vertices of the 3D force polytopes (one per leg)
computation_time = how long it took to compute the iterative projection
'''
polygon, computation_time = projection.project_polytope(params, None, 10. * np.pi / 180, 0.02)

ax = plt.subplot(111)

nc = np.sum(stanceFeet)
stanceID = params.getStanceIndex(stanceFeet)
# plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15)

fig2 = plt.figure()

''' 2D figure '''
# plt.figure()
for j in range(0,
			   nc):  # this will only show the contact positions and normals of the feet that are defined to be in stance
	idx = int(stanceID[j])
	''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
plt.plot(contactsWF[0:nc,0],contactsWF[0:nc,1],'ko',markersize=15, label=r'$\mathrm{Feet}$')
h4 = plotter.plot_polygon(np.transpose(polygon), 'b')

plt.grid()
plt.xlim([-0.5,0.5])
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.legend(bbox_to_anchor=(1.0, 0.87))
plt.savefig("foo2.pdf")
plt.show()