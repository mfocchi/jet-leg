# -*- coding: utf-8 -*-
"""
Created on Thu Oct 18 11:05:06 2018

@author: rorsolino
"""

# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""

import time
import pylab
import pypoman
import numpy as np

from context import jet_leg 

from numpy import array, cross, dot, eye, hstack, vstack, zeros, matrix
from numpy.linalg import norm
from jet_leg.plotting_tools import Plotter

from jet_leg.math_tools import Math
from jet_leg.computational_dynamics import ComputationalDynamics
from jet_leg.vertex_based_projection import VertexBasedProjection

import matplotlib.pyplot as plt
from jet_leg.arrow3D import Arrow3D
        
plt.close('all')
math = Math()
# number of contacts
nc = 3
# number of generators, i.e. rays used to linearize the friction cone
ng = 4

# ONLY_ACTUATION or ONLY_FRICTION
constraint_mode = 'ONLY_ACTUATION'
constraint_mode_IP = 'ONLY_ACTUATION'
useVariableJacobian = False
# number of decision variables of the problem
n = nc*6

# contact positions
""" contact points """

LF_foot = np.array([0.3, 0.3, -0.5])
RF_foot = np.array([0.3, -0.2, -0.5])
LH_foot = np.array([-0.2, 0.0, -0.5])
RH_foot = np.array([-0.3, -0.2, -0.5])

contactsToStack = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
contacts = contactsToStack[0:nc, :]

''' parameters to be tuned'''
g = 9.81
trunk_mass = 85.
mu = 0.8

axisZ= array([[0.0], [0.0], [1.0]])

n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
# %% Cell 2

normals = np.vstack([n1, n2, n3, n4])

#
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')

plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15)
for j in range(0,nc):
    ax.scatter(contacts[j,0], contacts[j,1], contacts[j,2],c='b',s=100)
    a = Arrow3D([contacts[j,0], contacts[j,0]+normals[j,0]/10], [contacts[j,1], contacts[j,1]+normals[j,1]/10],[contacts[j,2], contacts[j,2]+normals[j,2]/10], mutation_scale=20, lw=3, arrowstyle="-|>", color="r")
    ax.add_artist(a)

comp_dyn = ComputationalDynamics()
''' compute iterative projection '''
IP_points, actuation_polygons = comp_dyn.iterative_projection_bretl(constraint_mode_IP, contacts, normals, trunk_mass, ng, mu)

''' plotting Iterative Projection points '''
plotter = Plotter()
scaling_factor = 2000
if constraint_mode_IP == 'ONLY_ACTUATION':
    plotter.plot_polygon(np.transpose(IP_points))
    for j in range(0,nc):
        plotter.plot_actuation_polygon(ax, actuation_polygons[j], contacts[j,:], scaling_factor)

''' 2D figure '''
plt.figure()
h1 = plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15, label='feet')
h2 = plotter.plot_polygon(np.transpose(IP_points), '--b','Iterative Projection')
    
plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.legend()
plt.show()