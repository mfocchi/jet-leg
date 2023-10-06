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
robot_name="hyq"
comp_dyn = ComputationalDynamics(robot_name)
fwp = FeasibleWrenchPolytope(params)

num_generators = 4
max_normal_force = 500
mu = 0.2

n1 =  np.array([0, 1, 0])
FC1 = comp_dyn.constr.frictionConeConstr.linearized_cone_vertices(num_generators, mu, cone_height=max_normal_force, normal=n1).T

n2 =  np.array([0, 1, 0])
FC2 = comp_dyn.constr.frictionConeConstr.linearized_cone_vertices(num_generators, mu, cone_height=max_normal_force, normal=n2).T

line1 = np.array([[100, -100],
[500, 500],
[100, -100]])

line2 = np.array([[100, -100],
[500, 500],
[-100, 100]])

friction_cone_v = []
friction_cone_v.append(FC1)
friction_cone_v.append(FC2)
# friction_cone_v.append(line1)
# friction_cone_v.append(line2)

FWP = fwp.minkowskySum(friction_cone_v)
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