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

from scipy.optimize import linprog
from scipy.spatial import ConvexHull
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 10000, suppress = True)

plt.close('all')
math = Math()


# Functions from @Mateen Ulhaq and @karlo
def set_axes_equal(ax: plt.Axes):
    """Set 3D plot axes to equal scale.

    Make axes of 3D plot have equal scale so that spheres appear as
    spheres and cubes as cubes.  Required since `ax.axis('equal')`
    and `ax.set_aspect('equal')` don't work on 3D.
    """
    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])
    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    _set_axes_radius(ax, origin, radius)

def _set_axes_radius(ax, origin, radius):
    x, y, z = origin
    ax.set_xlim3d([x - radius, x + radius])
    ax.set_ylim3d([y - radius, y + radius])
    ax.set_zlim3d([z - radius, z + radius])


def plot_Robot(p_base,  anchor_1, anchor_2, w_R_b, contactsW):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    foot_sx = contactsW[0,:]
    foot_dx = contactsW[1, :]
    hoist_sx = contactsW[2, :]
    hoist_dx = contactsW[3, :]

    plt.title("robot")
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    #base
    ax.scatter(p_base[0], p_base[1], p_base[2], marker='o', color='g', s=200)
    #anchors
    ax.scatter(anchor_1[0], anchor_1[1], anchor_1[2], marker='o', color='b', s=50)
    ax.scatter(anchor_2[0], anchor_2[1], anchor_2[2], marker='o', color='b', s=50)
    #plot ropes
    ax.plot([hoist_sx[0] , anchor_1[0]], [hoist_sx[1] , anchor_1[1]], [hoist_sx[2] , anchor_1[2]],  color='r')
    ax.plot([hoist_dx[0] , anchor_2[0]], [hoist_dx[1] , anchor_2[1]], [hoist_dx[2] , anchor_2[2]],  color='r')
    # plot base  xaxis
    x_axis = w_R_b[:,0]
    y_axis = w_R_b[:, 1]
    z_axis = w_R_b[:, 2]
    ax.plot([p_base[0], p_base[0] + x_axis[0]], [p_base[1], p_base[1] + x_axis[1]], [p_base[2], p_base[2] + x_axis[2]], color='r')
    ax.plot([p_base[0], p_base[0] + y_axis[0]], [p_base[1], p_base[1] + y_axis[1]], [p_base[2], p_base[2] + y_axis[2]], color='g')
    ax.plot([p_base[0], p_base[0] + z_axis[0]], [p_base[1], p_base[1] + z_axis[1]], [p_base[2], p_base[2] + z_axis[2]], color='b')
    # feet
    ax.scatter(foot_sx[0], foot_sx[1], foot_sx[2], marker='o', color='b', s=50)
    ax.scatter(foot_dx[0], foot_dx[1], foot_dx[2], marker='o', color='b', s=50)

    #hoists
    ax.scatter(hoist_sx[0], hoist_sx[1], hoist_sx[2], marker='o', color='r', s=50)
    ax.scatter(hoist_dx[0], hoist_dx[1], hoist_dx[2], marker='o', color='r', s=50)

    ax.set_box_aspect([1, 1, 1])  # IMPORTANT - this is the new, key line
    # ax.set_proj_type('ortho') # OPTIONAL - default is perspective (shown in image above)
    set_axes_equal(ax)  # IMPORTANT - this is also required

    plt.show()

# This function only plots the linear part of the points!
def plot_FWP(FWP, title="FWP", static_wrench = None, margin = None, direction_of_max_wrench = np.array([0,0,1])):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    plt.title(title)
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    # plot all points also internal ones
    ax.scatter(FWP[0, :], FWP[1, :], FWP[2, :], marker='o', color='royalblue', alpha=0.2)

    if static_wrench is not None:
        ax.scatter(static_wrench[0], static_wrench[1], static_wrench[2], marker='o', color='g', s=200)


    if margin is not None:
        max_wrench = static_wrench[:3] + margin.x[:3]
        ax.scatter(max_wrench[0], max_wrench[1], max_wrench[2], marker='o', color='r', s=200)
        if direction_of_max_wrench is not None:
            # plot also the line
            t = np.linspace(0, 1500, 50)
            points_of_line = np.zeros((3, len(t)))
            for i in range(len(t)):
                points_of_line[:,i] = static_wrench[:3] + direction_of_max_wrench[:3] *t[i]
            ax.scatter(points_of_line[0,:], points_of_line[1,:], points_of_line[2,:],  color='g')

    # get boundary in 3D
    points = FWP[:3, :].T
    FWP3d_hull = ConvexHull(points)

    for i in FWP3d_hull.simplices:
        # plot edges of external points
        plt.plot(points[i, 0], points[i, 1], points[i, 2], color='r')
        #  plot only external points
        ax.scatter(points[i, 0], points[i, 1], points[i, 2], marker='o', color='royalblue', alpha=1.)

    # an alternative way to get the vertices
    # vertices = points[FWP3d_hull.vertices]
    # ax.scatter(vertices[:, 0], vertices[:, 1], vertices[:, 2], marker='o', color='b')

    plt.show()

def computeMargin(FWP, direction_v =np.array([0,0,1, 0,0,0]), static_wrench =  np.array([0,0,0, 0,0,0]), type_of_margin='6D')  :
    res = None

    if type_of_margin == '6D':
        FWP_hull = ConvexHull(FWP.T)#, qhull_options="Q0")
        # flag to see if static wrench is inside
        inside = np.all(FWP_hull.equations[:, :-1].dot(static_wrench) + FWP_hull.equations[:, -1] < 0)

        if inside:
            # be sure is a unit vector
            direction_v_unit= direction_v/ np.linalg.norm(direction_v)
            # maximizes the cost
            c = -direction_v_unit.reshape(1,6)
            # point (x + w_gi) should be inside polytope A (x+wgi)<=b -> A x < b - Aw_gi
            A_ub = FWP_hull.equations[:, :-1]
            b_ub = -FWP_hull.equations[:, -1] -A_ub.dot(static_wrench)
            # nullspace constraints (remove everything else is orthogonal to direction_v_unit)
            A_eq = np.eye(6) -np.outer(direction_v_unit,direction_v_unit.T)
            b_eq = np.zeros(6)
            bound = (-np.inf, np.inf)
            res = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq, bounds=[bound] * len(c), method='highs-ds',
                          callback=None, options=None, x0=None)
        else:
            print('static wrench is out of polytope, static equilibrum is not possible')

    elif type_of_margin == '3D':
        # for debugging use 3d version
        FWP_hull3d = ConvexHull(FWP[:3, :].T)#, qhull_options="QJ")
        # flag to see if static wrench is inside
        inside = np.all(FWP_hull3d.equations[:, :-1].dot(static_wrench[:3]) + FWP_hull3d.equations[:, -1] <0)

        if inside:
            # be sure is a unit vector
            direction_v_unit = direction_v[:3] / np.linalg.norm(direction_v[:3])
            # maximizes the cost
            c = -direction_v_unit[:3].reshape(1,3)
            A_ub = FWP_hull3d.equations[:, :-1]
            b_ub = -FWP_hull3d.equations[:, -1] - A_ub.dot(static_wrench[:3])
            # nullspace constraints (remove everything else is orthogonal to direction_v_unit)
            A_eq = np.eye(3) -np.outer(direction_v_unit,direction_v_unit.T)
            b_eq = np.zeros(3)
            bound = (-np.inf, np.inf)
            res = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq, bounds=[bound] * len(c), method='highs-ds',
                          callback=None, options=None, x0=None)
        else:
            print('static wrench is out of polytope, static equilibrum is not possible')
    else:
        print("Wrong type")


    #debug
    # res = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=None, b_eq=None, bounds=None, method='highs-ds',
    #               callback=None, options=None, x0=None)
    #print("Max force Fz", res.x[2])
    return res

def computeOrientation(p_base, p_anchor1, p_anchor2):

    l1 = np.linalg.norm(p_base - p_anchor1)
    l2 =  np.linalg.norm(p_base - p_anchor2)
    psi = np.arctan2(p_base[0], -p_base[2])
    print("PSI: ", psi)
    w_R_b=np.array([[np.cos(psi), 0, -np.sin(psi)],
                    [0, 1,                    0   ],
                    [ np.sin(psi), 0, np.cos(psi)]])
    return w_R_b

'''You now need to fill the 'params' object with all the relevant 
    informations needed for the computation of the IP'''
params = IterativeProjectionParameters()
comp_dyn = ComputationalDynamics()
fwp = FeasibleWrenchPolytope(params)
# number of contacts
fwp.no_of_legs = 4

# Set number of edges of the friction cones and value of the cap plane
num_generators = 4
# Set active contacts
mu = 0.8
activeContacts = [1,1,1,1]
activeContactsIndex = params.getStanceIndex(activeContacts)

#inputs
comWF = np.array([1.5, 2.5, -6.0])
p_anchor1 = np.array([0,0,0])
p_anchor2 = np.array([0,5,0])
wall_normal =  np.array([1, 0, 0])
max_rope_force = 600.
max_leg_force = 300.

# compute relevant kin quantities
w_R_b = computeOrientation(comWF, p_anchor1, p_anchor2)

landing_joint = 0.7 # assumed always positive
lower_landing_leg = 0.3
offset_base_y = 0.08
contact_foot_sx = np.array([-lower_landing_leg*np.sin(landing_joint), - offset_base_y - lower_landing_leg*np.cos(landing_joint), 0.025]) # left foot
contact_foot_dx = np.array([-lower_landing_leg*np.sin(landing_joint), offset_base_y + lower_landing_leg*np.cos(landing_joint), 0.025]) # right foot
contact_hoist_sx = np.array([0, -0.05, 0.05]) # rope (attachment)
contact_hoist_dx = np.array([0, 0.05, 0.05]) # rope (attachment)

contact_foot_sxW = w_R_b.dot(contact_foot_sx) +comWF
contact_foot_dxW = w_R_b.dot(contact_foot_dx) +comWF
contact_hoist_sxW = w_R_b.dot(contact_hoist_sx) +comWF
contact_hoist_dxW = w_R_b.dot(contact_hoist_dx) +comWF
contactsWF = np.vstack((contact_foot_sxW, contact_foot_dxW, contact_hoist_sxW, contact_hoist_dxW))
print("Contacts position in WF (row-wise)\n", contactsWF)
# comment this if you want to run debug
#plot_Robot(comWF, p_anchor1, p_anchor2, w_R_b, contactsWF)


# line of actions of the anchor forces (rope axis univ vectors)
W_rope_axis_sx  = (contact_hoist_sxW-p_anchor1)/np.linalg.norm(contact_hoist_sxW-p_anchor1)
W_rope_axis_dx  =  (contact_hoist_dxW-p_anchor2)/np.linalg.norm(contact_hoist_dxW-p_anchor2)

# min/max anchor forces manifolds
W_rope_force_sx = np.hstack(( -W_rope_axis_sx.reshape(3,1)*max_rope_force, np.zeros((3,1))) )
W_rope_force_dx = np.hstack(( -W_rope_axis_dx.reshape(3,1)*max_rope_force, np.zeros((3,1))) )
print("Rope force manifold sx in WF (colunmn wise)\n", W_rope_force_sx)
print("Rope force manifold dx in WF(colunmn wise)\n", W_rope_force_dx)


#debug
# W_rope_force_sx = np.array([  [0, -10],
#                            [0, -100],
#                            [0, 500]])
#
# W_rope_force_dx = np.array([[0, -10],
#                            [0, 100],
#                            [0, 500]])


#friction cones at feet
FC1 = comp_dyn.constr.frictionConeConstr.linearized_cone_vertices(num_generators, mu, cone_height=max_leg_force, normal=wall_normal).T
FC2 = comp_dyn.constr.frictionConeConstr.linearized_cone_vertices(num_generators, mu, cone_height=max_leg_force, normal=wall_normal).T



# The order you use to append the feasible sets should match the order of the contacts
friction_cone_v = []
friction_cone_v.append(FC1)
friction_cone_v.append(FC2)
friction_cone_v.append(W_rope_force_sx)
friction_cone_v.append(W_rope_force_dx)

feasible_sets_6D = fwp.computeAngularPart(contactsWF.T, activeContacts, activeContactsIndex, friction_cone_v)

print("6-D force sets:\n")
print(feasible_sets_6D)

FWP = fwp.minkowskySum(feasible_sets_6D)
print("Number of vertices", np.shape(FWP)[1])

# Compute centroidal wrench
mass = 15.07
external_wrench = [0]*6
w_gi = comp_dyn.rbd.computeCentroidalWrench(mass, comWF, external_wrench)

# '''I now check whether the given CoM configuration is having any operation margin'''
direction_of_max_wrench = np.array([-1,  0,  0, 0, 0, 0])
#w_gi = np.array([0,0,147,400,-320,0])
res = computeMargin(FWP, direction_v=direction_of_max_wrench , static_wrench = w_gi, type_of_margin='3D')
plot_FWP(FWP, "FWP", static_wrench=w_gi, margin = res, direction_of_max_wrench=direction_of_max_wrench)

if res is not None:
    print(f"max wrench in {direction_of_max_wrench} direction is: {res.x}")
# https://www.sandvik.coromant.com/it-it/knowledge/machining-formulas-definitions/drilling-formulas-definitions
# https://www.albertobarbisan.it/didattica/FORATURA.pdf


# Debug
# direction_of_max_wrench = np.array([0.,  0,  1.])
# outside
# static_wrench = np.array([0,0,2000])
# res = computeMargin(FWP, direction_v=direction_of_max_wrench , static_wrench = static_wrench, type_of_margin='3D')
# plot_FWP(FWP, "FWP", static_wrench = static_wrench, margin = res, direction_of_max_wrench=direction_of_max_wrench)
# #inside
# static_wrench = np.array([500,0,147])
# res = computeMargin(FWP, direction_v=direction_of_max_wrench , static_wrench = static_wrench, type_of_margin='3D')
# plot_FWP(FWP, "FWP", static_wrench = static_wrench, margin = res, direction_of_max_wrench=direction_of_max_wrench)