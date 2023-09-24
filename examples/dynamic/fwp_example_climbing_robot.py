# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""

import numpy as np

from numpy import array
from jet_leg.plotting.plotting_tools import Plotter
import random
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.dynamics.feasible_wrench_polytope import FeasibleWrenchPolytope
from jet_leg.dynamics.rigid_body_dynamics import RigidBodyDynamics
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
import time
import matplotlib.pyplot as plt


plt.close('all')
math = Math()

'''You now need to fill the 'params' object with all the relevant 
    informations needed for the computation of the IP'''
params = IterativeProjectionParameters()

# if you have more than 2 feet in contact, just set pointContacts to True
# since the feasible region is a region, when you have only two feet on the ground, the region would degenerate, and the
# algorithm cannot find a solution. so what we did is that we introduced an infinitismal contact torque for each feet,
# even if the foot is a point foot just to give a small degree of freedom for the region to be a polygon and not a line
# params.pointContacts = True

# params.setContactsPosWF(contactsWF)
# params.externalCentroidalWrench = extCentroidalWrench
# params.setCoMPosWF(comWF)
# params.setCoMLinAcc(comWF_lin_acc)
# params.setTorqueLims(comp_dyn.robotModel.robotModel.joint_torque_limits)
# params.setActiveContacts(stanceFeet)
# params.setConstraintModes(constraint_mode_IP)
# params.setContactNormals(normals)
# params.setFrictionCoefficient(mu)
# params.setNumberOfFrictionConesEdges(ng)
# params.setTotalMass(comp_dyn.robotModel.robotModel.trunkMass)

# '''I now check whether the given CoM configuration is stable or not'''
# C, d, isIKoutOfWorkSpace, forcePolytopes = comp_dyn.constr.getInequalities(params)

# print("Force Polytopes", C, d, isIKoutOfWorkSpace, forcePolytopes.getHalfspaces())
fwp = FeasibleWrenchPolytope(params)

FC1 = np.array([[0, 100, 100, -100, -100],
[0, 500, 500, 500, 500],
[0, -100, 100, -100, 100]])

line1 = np.array([[100, -100],
[500, 500],
[100, -100]])

line2 = np.array([[100, -100],
[500, 500],
[-100, 100]])

friction_cone_v = []
friction_cone_v.append(FC1)
friction_cone_v.append(line1)
friction_cone_v.append(line2)

FWP = fwp.minkowskySum(len(friction_cone_v), friction_cone_v)
print("Vertices", FWP)

from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
ax.scatter(FWP[0, :], FWP[1, :], FWP[2, :], marker='o', color='purple')


points = FWP[:3, :].T
hull = ConvexHull(points)
for i in hull.simplices:
    plt.plot(points[i, 0], points[i, 1], points[i, 2], 'r-')

plt.show()




plt.show()