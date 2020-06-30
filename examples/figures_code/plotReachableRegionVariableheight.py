# -*- coding: utf-8 -*-
"""
Created on Fri Aug 10 16:08:43 2018

@author: romeoorsolino
"""

import numpy as np

from context import jet_leg

from numpy import array, cross, dot, eye, hstack, vstack, zeros, matrix
from numpy.linalg import norm

from jet_leg.maths.math_tools import Math
from jet_leg.maths.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.optimization import nonlinear_projection

import matplotlib as mpl
import matplotlib.colors as colors
import matplotlib.cm as cmx
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Wedge, Polygon
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
from matplotlib.collections import PatchCollection


def set_axes_radius(ax, origin, radius):
	ax.set_xlim3d([origin[0] - radius, origin[0] + radius])
	ax.set_ylim3d([origin[1] - radius, origin[1] + radius])
	ax.set_zlim3d([origin[2] - radius, origin[2] + radius])


def set_axes_equal(ax):
	'''Make axes of 3D plot have equal scale so that spheres appear as spheres,
	cubes as cubes, etc..  This is one possible solution to Matplotlib's
	ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

	Input
	  ax: a matplotlib axis, e.g., as output from plt.gca().
	'''

	limits = np.array([
		ax.get_xlim3d(),
		ax.get_ylim3d(),
		ax.get_zlim3d(),
	])

	origin = np.mean(limits, axis=1)
	radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
	set_axes_radius(ax, origin, radius)


plt.close('all')

robot_name = 'hyq'
projection = nonlinear_projection.NonlinearProjectionBretl(robot_name)

math = Math()
# number of contacts

# number of generators, i.e. rays used to linearize the friction cone
ng = 4

# ONLY_ACTUATION or ONLY_FRICTION or FRICTION_AND_ACTUATION
# constraint_mode = 'ONLY_ACTUATION'
constraint_mode = ['FRICTION_AND_ACTUATION',
				   'FRICTION_AND_ACTUATION',
				   'FRICTION_AND_ACTUATION',
				   'FRICTION_AND_ACTUATION']

useVariableJacobian = False

comWF = np.array([0.009, 0.0001, 0.549])  # pos of COM in world frame w. trunk controller
# comBF = np.array([0.0094,  0.0002, -0.0433])  # pos of COM in body frame w.o. trunk controller
comBF = np.array([0.0094, 0.0002, -0.0458])  # pos of COM in body frame w. trunk controller
# rpy = np.array([0.00012, 0.00601, 3.6e-05])  # orientation of body frame w.o. trunk controller
rpy = np.array([0.00001589, -0.00000726, -0.00000854])  # orientation of body frame w. trunk controller

# contact positions
""" contact points """
# LF_foot = np.array([0.3, 0.2, -.9])
# RF_foot = np.array([0.3, -0.2, -0.5])
# LH_foot = np.array([-0.3, 0.2, -0.5])
# RH_foot = np.array([-0.3, -0.2, -0.5])

LF_foot = np.array([0.36, 0.32, 0.02])  # Starting configuration w.o. trunk controller
RF_foot = np.array([0.36, -0.32, 0.02])
LH_foot = np.array([-0.36, 0.32, 0.02])
RH_foot = np.array([-0.36, -0.32, 0.02])

contacts = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))
stanceLegs = [1, 1, 1, 1]
nc = np.sum(stanceLegs)
stanceIndex = []
swingIndex = []
print 'stance', stanceLegs
for iter in range(0, 4):
	if stanceLegs[iter] == 1:
		#               print 'new poly', stanceIndex, iter
		stanceIndex = np.hstack([stanceIndex, iter])
	else:
		swingIndex = iter

''' parameters to be tuned'''
g = 9.81
mu = 0.8
axisZ = array([[0.0], [0.0], [1.0]])

n1 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))
n2 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))
n3 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))
n4 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))
# %% Cell 2

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

normals = np.vstack([n1, n2, n3, n4])

''' Add 2D figure '''
mpl.rcParams['text.usetex'] = True
mpl.rcParams['text.latex.unicode'] = True
fig = plt.figure(1)

scale = np.linspace(50, 150, 6)
jet = cm = plt.get_cmap('RdBu')
cNorm = colors.Normalize(vmin=55, vmax=145)
scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=jet)
idx = 5

params = IterativeProjectionParameters()
params.setContactsPosWF(contacts)
params.setCoMPosWF(comWF)
params.setCoMPosBF(comBF)
params.setOrientation(rpy)
params.setActiveContacts(stanceLegs)
params.setConstraintModes(constraint_mode)
params.setContactNormals(normals)
params.setFrictionCoefficient(mu)
params.setNumberOfFrictionConesEdges(ng)
params.setJointLimsMax(joint_limits_max)
params.setJointLimsMin(joint_limits_min)


ax = plt.subplot(111)

for height in range(65, 35, -5):
	comWF[2] = height/100.
	params.setCoMPosWF(comWF)
	polygon, computation_time = projection.project_polytope(params, None, 10. * np.pi / 180, 0.02)
	point = np.vstack([polygon])
	colorVal = scalarMap.to_rgba(scale[idx])
	colorText = ('color: (%4.2f,%4.2f,%4.2f)' % (colorVal[0], colorVal[1], colorVal[2]))
	idx -= 1
	# plotter.plot_polygon(np.transpose(IP_points), x[0],'trunk mass ' + str(trunk_mass*10) + ' N')
	x = np.hstack([point[:, 0], point[0, 0]])
	y = np.hstack([point[:, 1], point[0, 1]])
	h1 = ax.plot(x, y, color=colorVal, linewidth=5., label=r'${}$'.format(height/100.) + r' $\mathrm{m}$')
ax.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15, label=r'$\mathrm{Feet}$')
# handles, labels = ax.get_legend_handles_labels()
# labels.append(Patch('ko','$\mathrm{Feet}$')
# constraint_mode = ['ONLY_FRICTION',
# 				   'ONLY_FRICTION',
# 				   'ONLY_FRICTION',
# 				   'ONLY_FRICTION']
# params.setConstraintModes(constraint_mode)
# IP_points, actuation_polygons, comp_time = comp_dyn.iterative_projection_bretl(params)
# point = np.vstack([IP_points])
# x = np.hstack([point[:, 0], point[0, 0]])
# y = np.hstack([point[:, 1], point[0, 1]])
# h2 = plt.plot(x, y, color='blue', linestyle='dashed', linewidth=5., label='only friction')
#
# plt.rc('font', family='serif', size=20)
# plt.grid()
# plt.xlabel("x [m]")
# plt.ylabel("y [m]")
# plt.legend(prop={'size': 20}, bbox_to_anchor=(1.1, 1.1))
# # plt.axis('equal')
# plt.axis([-1.25, 1.75, -1.45, 1.55])
# plt.show()

box = ax.get_position()
ax.set_position([box.x0, box.y0, box.width * 0.95, box.height])
plt.xlim([-0.5,0.5])
plt.rc('font', family='serif', size=20)
plt.grid()
plt.xlabel("x [m]")
plt.ylabel("y [m]")
ax.legend(bbox_to_anchor=(1.17, 0.83), fontsize=18, framealpha=0.6)

plt.savefig("foo.pdf")
plt.show()
