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

import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cmx
from jet_leg.plotting.arrow3D import Arrow3D
from matplotlib import rc
# rc('text', usetex=True)
# rc('font', size=14)
# rc('legend', fontsize=16)
# rc('text.latex', preamble=r'\usepackage{amsfonts}')

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
constraint_mode_IP = ['ONLY_FRICTION',
					  'ONLY_FRICTION',
					  'ONLY_FRICTION',
					  'ONLY_FRICTION']

# number of decision variables of the problem
# n = nc*6
comWF = np.array([-0.009, 0.0001, 0.549])  # pos of COM in world frame w. trunk controller
comBF = np.array([-0.0094, 0.0002, -0.0458])  # pos of COM in body frame w. trunk controller
rpy = np.array([0.00001589, -0.00000726, -0.00000854])  # orientation of body frame w. trunk controller

""" contact points in the World Frame"""
LF_foot = np.array([0.36, 0.32, 0.02])  # Starting configuration w.o. trunk controller
RF_foot = np.array([0.36, -0.32, 0.02])
LH_foot = np.array([-0.36, 0.32, 0.02])
RH_foot = np.array([-0.36, -0.32, 0.02])

contactsWF = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))

''' parameters to be tuned'''
trunk_mass = 85.
mu = 0.6

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

n1 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # LF
n2 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # RF
n3 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # LH
n4 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # RH
normals = np.vstack([n1, n2, n3, n4])

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

''' extForceW is an optional external pure force (no external torque for now) applied on the CoM of the robot.'''
extForceW4 = [-0000.0, 00.0, +000.0]  # units are Nm
extTorqueW4 = [00.0, -00000.0, -00000.0]  # units are Nm
extForceW1 = [-100.0, 0.0, 0.0]  # units are Nm
extTorqueW1 = [0.0, -100.0, 0.0]  # units are Nm
extForceW2 = [-000.0, -100.0, 0.0]  # units are Nm
extTorqueW2 = [000.0, -100.0, 0.0]  # units are Nm
extForceW3 = [0.0, 0.0, 0.0]  # units are Nm
extTorqueW3 = [0.0, 0.0, 100.0]  # units are Nm

extWrenchesW = np.array([[extForceW1, extTorqueW1],
					   [extForceW2, extTorqueW2],
					   [extForceW3, extTorqueW3],
					   [extForceW4, extTorqueW4]])

externalWrenchesIter = iter(extWrenchesW)


comp_dyn = ComputationalDynamics(robot_name)

'''You now need to fill the 'params' object with all the relevant
    informations needed for the computation of the IP'''
params = IterativeProjectionParameters()

params.setContactsPosWF(contactsWF)
params.setCoMPosWF(comWF)
params.setCoMPosBF(comBF)
params.setOrientation(rpy)
params.setTorqueLims(torque_limits)
params.setActiveContacts(stanceFeet)
params.setConstraintModes(constraint_mode_IP)
params.setContactNormals(normals)
params.setFrictionCoefficient(mu)
params.setNumberOfFrictionConesEdges(ng)
params.setTotalMass(trunk_mass)
# params.externalForceWF = extForceW  # params.externalForceWF is actually used anywhere at the moment
# params.externalTorqueWF = extTorqueW


# '''Plotting the contact points in the 3D figure'''
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.set_xlabel('X axis')
# ax.set_ylabel('Y axis')
# ax.set_zlabel('Z axis')

nc = np.sum(stanceFeet)
stanceID = params.getStanceIndex(stanceFeet)
force_scaling_factor = 1500
# plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15)
fz_tot = 0.0
# for j in range(0,
# 			   nc):  # this will only show the contact positions and normals of the feet that are defined to be in stance
# 	idx = int(stanceID[j])
# 	ax.scatter(contactsWF[idx, 0], contactsWF[idx, 1], contactsWF[idx, 2], c='b', s=100)
# 	'''CoM will be plotted in green if it is stable (i.e., if it is inside the feasible region'''
# 	if isConfigurationStable:
# 		ax.scatter(comWF[0], comWF[1], comWF[2], c='g', s=100)
# 		grf = contactForces[j * 3:j * 3 + 3]
# 		fz_tot += grf[2]
#
# 		''' draw the set contact forces that respects the constraints'''
# 		b = Arrow3D([contactsWF[idx, 0], contactsWF[idx, 0] + grf[0] / force_scaling_factor],
# 					[contactsWF[idx, 1], contactsWF[idx, 1] + grf[1] / force_scaling_factor],
# 					[contactsWF[idx, 2], contactsWF[idx, 2] + grf[2] / force_scaling_factor], mutation_scale=20, lw=3,
# 					arrowstyle="-|>",
# 					color="b")
# 		ax.add_artist(b)
# 	else:
# 		ax.scatter(comWF[0], comWF[1], comWF[2], c='r', s=100)
#
# 	''' draw 3D arrows corresponding to contact normals'''
# 	a = Arrow3D([contactsWF[idx, 0], contactsWF[idx, 0] + normals[idx, 0] / 10],
# 				[contactsWF[idx, 1], contactsWF[idx, 1] + normals[idx, 1] / 10],
# 				[contactsWF[idx, 2], contactsWF[idx, 2] + normals[idx, 2] / 10], mutation_scale=20, lw=3,
# 				arrowstyle="-|>", color="r")
#
# 	''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
# 	ax.scatter(contactsWF[idx, 0], contactsWF[idx, 1], 0.0, c='k', s=100)
# 	ax.add_artist(a)

print 'sum of vertical forces is', fz_tot

scale = np.linspace(50, 150, 5)
jet = cm = plt.get_cmap('Set1')
cNorm  = colors.Normalize(vmin=50, vmax=150)
scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=jet)
idx = 0
# ''' plotting Iterative Projection points '''
plotter = Plotter()
# for j in range(0, nc):  # this will only show the force polytopes of the feet that are defined to be in stance
# 	idx = int(stanceID[j])
# 	plotter.plot_polygon(np.transpose(IP_points))
# 	if (constraint_mode_IP[idx] == 'ONLY_ACTUATION') or (constraint_mode_IP[idx] == 'FRICTION_AND_ACTUATION'):
# 		plotter.plot_actuation_polygon(ax, forcePolytopes[idx], contactsWF[idx, :], force_scaling_factor)
f = plt.figure()
ax = plt.subplot(111)
lines = []

colorVal_list = [(0.7,0,0),(0,0.4,0),(1,0.5,0),(0,0,0.3)]
for wrench in externalWrenchesIter:
	''' 2D figure '''
	params.externalForceWF = wrench[0]
	params.externalTorqueWF = wrench[1]

	IP_points, force_polytopes, IP_computation_time = comp_dyn.iterative_projection_bretl(params)
	point = np.vstack([IP_points])
	# colorVal = scalarMap.to_rgba(scale[idx])
	colorVal = colorVal_list[idx]
	colorText = ('color: (%4.2f,%4.2f,%4.2f)' % (colorVal[0], colorVal[1], colorVal[2]))
	x = np.hstack([point[:, 0], point[0, 0]])
	y = np.hstack([point[:, 1], point[0, 1]])
	if idx==3 :
		style = '--'
	else:
		style = '-'
	h = ax.plot(x, y, style, color=colorVal, linewidth=5.)
	idx += 1
	# lines.append(h)

# h2 = plotter.plot_polygon(np.transpose(IP_points), '--b', 'Support Region')
ax.plot(contactsWF[0:nc,0],contactsWF[0:nc,1],'ko',markersize=15)

box = ax.get_position()
ax.set_position([box.x0, box.y0, box.width * 0.92, box.height])
ax.legend([r'$F_x = -100 \, \mathrm{N,}$' + '\n' + r'$\tau_y = -100 \, \mathrm{Nm}$',
			r'$F_y = -100 \, \mathrm{N,}$' + '\n' + r'$\tau_y = -100 \, \mathrm{Nm}$',
			r'$\tau_\mathrm{z} = 100 \, \mathrm{Nm}$',
			'$\mathrm{No}$ $\mathrm{ext}$ $\mathrm{wrench}$'], # mathrm so the text font and latex symbols look the same
		   loc="lower center",
		   	bbox_to_anchor=(1.02, 0.4), fontsize=16, framealpha=0.8)

# plt.legend([lines[0][0], lines[1][0], lines[2][0]],[r'$F_x = -100, \tau_y = -100$',
# 		   	r'$F_x = -100, F_y = 50, \tau_x = 50, \tau_y = -100$',
# 			r'$\tau_z = 100$'],
# 			 fontsize='x-large')
# plt.legend([lines[3][0]], [r'$No external wrench$'],
# 		   	loc='lower center', fontsize='small')

plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
# plt.xlim(-0.5, 0.7)
# plt.legend()
f.savefig("foo.pdf")
plt.show()