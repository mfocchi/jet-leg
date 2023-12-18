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
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull
import unittest
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 10000, suppress = True)

#to run: python3 -m unittest test_climbing_robot

plt.close('all')
math = Math()

class TestClimbingRobot(unittest.TestCase):

    def computeMargin(self, FWP, direction_v=np.array([0, 0, 1, 0, 0, 0]), static_wrench=np.array([0, 0, 0, 0, 0, 0]),
                      type_of_margin='6D'):
        res = None

        if type_of_margin == '6D':
            FWP_hull = ConvexHull(FWP.T)#, qhull_options="QJ")
            # flag to see if static wrench is inside
            inside = np.all(FWP_hull.equations[:, :-1].dot(static_wrench) + FWP_hull.equations[:, -1] < 0)

            if inside:
                # be sure is a unit vector
                direction_v_unit = direction_v / np.linalg.norm(direction_v)
                # maximizes the cost
                c = -direction_v_unit.reshape(1, 6)
                # point (x + w_gi) should be inside polytope A (x+wgi)<=b -> A x < b - Aw_gi
                A_ub = FWP_hull.equations[:, :-1]
                b_ub = -FWP_hull.equations[:, -1] - A_ub.dot(static_wrench)
                # nullspace constraints (remove everything else is orthogonal to direction_v_unit)
                A_eq = np.eye(6) - np.outer(direction_v_unit, direction_v_unit.T)
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
            inside = np.all(FWP_hull3d.equations[:, :-1].dot(static_wrench[:3]) + FWP_hull3d.equations[:, -1] < 0)

            if inside:
                # be sure is a unit vector
                direction_v_unit = direction_v[:3] / np.linalg.norm(direction_v[:3])
                # maximizes the cost
                c = -direction_v_unit[:3].reshape(1, 3)
                A_ub = FWP_hull3d.equations[:, :-1]
                b_ub = -FWP_hull3d.equations[:, -1] - A_ub.dot(static_wrench[:3])
                # nullspace constraints (remove everything else is orthogonal to direction_v_unit)
                A_eq = np.eye(3) - np.outer(direction_v_unit, direction_v_unit.T)
                b_eq = np.zeros(3)
                bound = (-np.inf, np.inf)
                res = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq, bounds=[bound] * len(c), method='highs-ds',
                              callback=None, options=None, x0=None)
            else:
                print('static wrench is out of polytope, static equilibrum is not possible')
        else:
            print("Wrong type")

        return res

    def computeOrientation(self, p_base, p_anchor1, p_anchor2):

        l1 = np.linalg.norm(p_base - p_anchor1)
        l2 = np.linalg.norm(p_base - p_anchor2)
        psi = np.arctan2(p_base[0], -p_base[2])
        print("PSI: ", psi)
        w_R_b = np.array([[np.cos(psi), 0, -np.sin(psi)],
                          [0, 1, 0],
                          [np.sin(psi), 0, np.cos(psi)]])
        return w_R_b
#TODO redo this
    # def test_rectangle(self):
    #     '''You now need to fill the 'params' object with all the relevant
    #         informations needed for the computation of the IP'''
    #     params = IterativeProjectionParameters()
    #     fwp = FeasibleWrenchPolytope(params)
    #     robot_name="hyq" # The robot name is only needed in the case you want to compute the force polytopes (using the leg Jacobians)
    #     comp_dyn = ComputationalDynamics(robot_name)
    #     fwp.no_of_legs = 4
    #
    #     # Set number of edges of the friction cones and value of the cap plane
    #     num_generators = 4
    #     max_normal_force = 500
    #
    #     # Set active contacts
    #     mu = 0.2
    #     stanceLegs = [1,1,1,1]
    #     stanceIndex = params.getStanceIndex(stanceLegs)
    #     comWF = np.array([0., 0., 0.0])
    #
    #
    #     FWP = np.array([[150, -150, 150, -150, 150, -150, 150, -150],
    #     [-100, -100, 100, 100, -100, -100, 100, 100],
    #     [-200, -200, -200, -200, 200, 200, 200, 200]])
    #     print("vertices", FWP)
    #
    #     '''I now check whether the given CoM configuration is dynamically stable or not (see "Feasible Wrench Polytope")'''
    #     w_gi = np.array([0., 0., 0.0])
    #     isFWPStable = fwp.checkDynamicStability(FWP, w_gi)
    #     # self.assertEqual(isFWPStable, True)
    #
    #     res = self.computeMargin(FWP, np.array([1, 1, 1, 0, 0, 0]), static_wrench = np.array([100,0,0,0,0,0]),type_of_margin='3D')
    #     print("Result", res.x)
    #     self.assertEqual(res.x[0], 150)
    #     self.assertEqual(res.x[1], 100)
    #     self.assertEqual(res.x[2], 200)
    #
    #     res = self.computeMargin(FWP, np.array([1, -1, 1, 0, 0, 0]), static_wrench = np.array([100,0,0,0,0,0]),type_of_margin='3D')
    #     print("Result", res.x)
    #     self.assertEqual(res.x[0], 150)
    #     self.assertEqual(res.x[1], -100)
    #     self.assertEqual(res.x[2], 200)
    #
    #     res = self.computeMargin(FWP, np.array([1, 1, -1, 0, 0, 0]), static_wrench = np.array([100,0,0,0,0,0]),type_of_margin='3D')
    #     print("Result", res.x)
    #     self.assertEqual(res.x[0], 150)
    #     self.assertEqual(res.x[1], 100)
    #     self.assertEqual(res.x[2], -200)
    #
    #     res = self.computeMargin(FWP, np.array([-1, -1, -1, 0, 0, 0]), static_wrench = np.array([100,0,0,0,0,0]),type_of_margin='3D')
    #     print("Result", res.x)
    #     self.assertEqual(res.x[0], -150)
    #     self.assertEqual(res.x[1], -100)
    #     self.assertEqual(res.x[2], -200)
    #
    #     points = FWP[:3, :].T
    #     hull = ConvexHull(points)
    #
    #     print("Hull", hull.vertices)
    #     self.assertEqual(len(hull.vertices), 8)

    def setup_linearized_cone_3d(self, normal, direction):
        '''You now need to fill the 'params' object with all the relevant 
            informations needed for the computation of the IP'''
        params = IterativeProjectionParameters()
        fwp = FeasibleWrenchPolytope(params)
        robot_name="hyq" # The robot name is only needed in the case you want to compute the force polytopes (using the leg Jacobians)
        comp_dyn = ComputationalDynamics(robot_name)
        fwp.no_of_legs = 4

        # Set number of edges of the friction cones and value of the cap plane
        num_generators = 4
        max_normal_force = 500

        # Set active contacts
        mu = 0.2
        stanceLegs = [1,1,1,1]
        stanceIndex = params.getStanceIndex(stanceLegs)
        comWF = np.array([0., 0., 0.0])

        n1 =  normal
        FC1 = comp_dyn.constr.frictionConeConstr.linearized_cone_vertices(num_generators, mu, cone_height=max_normal_force, normal=n1).T
        print("FC", FC1)
        FWP = FC1 #fwp.minkowskySum(feasible_sets_6D)
        print("Number of vertices", np.shape(FWP))

        '''I now check whether the given CoM configuration is dynamically stable or not (see "Feasible Wrench Polytope")'''
        w_gi = np.array([0., 0., 0.])
        isFWPStable = fwp.checkDynamicStability(FWP, w_gi)
        self.assertEqual(isFWPStable, True)

        points = FWP[:3, :].T
        hull = ConvexHull(points)

        print("Hull", hull.vertices)
        self.assertEqual(len(hull.vertices), 5)

        return self.computeMargin(FWP, np.array([direction[0], direction[1], direction[2], 0, 0, 0]), static_wrench = np.array([0,1600,0,0,0,0]))


#TODO check
    # def test_linearized_cone_3d_x(self):
    #
    #     direction = n =  np.array([1, 0, 0])
    #     res = self.setup_linearized_cone_3d(n, direction)
    #     self.assertEqual(res.x[0], 500)
    #
    #     direction =  np.array([-1, 0, 0])
    #     res = self.setup_linearized_cone_3d(n, direction)
    #     print("Result", res.x)
    #     self.assertEqual(res.x[0], 0)
    #     self.assertEqual(res.x[1], 0)
    #     self.assertEqual(res.x[2], 0)
        
    def test_climbing(self):

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
        activeContacts = [1, 1, 1, 1]
        activeContactsIndex = params.getStanceIndex(activeContacts)

        gazeboWF_offset = np.array([10, 10, -10])

        # inputs
        comWF = gazeboWF_offset+ np.array([1.5, 2.5, -6.0])
        p_anchor1 = gazeboWF_offset + np.array([0, 0, 0])
        p_anchor2 = gazeboWF_offset + np.array([0, 5, 0])
        wall_normal = np.array([1, 0, 0])
        max_rope_force = 600.
        max_leg_force = 300.

        # compute relevant kin quantities
        w_R_b = self.computeOrientation(comWF, p_anchor1, p_anchor2)

        landing_joint = 0.7  # assumed always positive
        lower_landing_leg = 0.3
        offset_base_y = 0.08
        contact_foot_sx = np.array(
            [-lower_landing_leg * np.sin(landing_joint), - offset_base_y - lower_landing_leg * np.cos(landing_joint),
             0.025])  # left foot
        contact_foot_dx = np.array(
            [-lower_landing_leg * np.sin(landing_joint), offset_base_y + lower_landing_leg * np.cos(landing_joint),
             0.025])  # right foot
        contact_hoist_sx = np.array([0, -0.05, 0.05])  # rope (attachment)
        contact_hoist_dx = np.array([0, 0.05, 0.05])  # rope (attachment)

        contact_foot_sxW = w_R_b.dot(contact_foot_sx) + comWF
        contact_foot_dxW = w_R_b.dot(contact_foot_dx) + comWF
        contact_hoist_sxW = w_R_b.dot(contact_hoist_sx) + comWF
        contact_hoist_dxW = w_R_b.dot(contact_hoist_dx) + comWF
        contactsWF = np.vstack((contact_foot_sxW, contact_foot_dxW, contact_hoist_sxW, contact_hoist_dxW))
        print("Contacts position in WF", contactsWF)

        # line of actions of the anchor forces (rope axis univ vectors)
        W_rope_axis_sx = (contact_hoist_sxW - p_anchor1) / np.linalg.norm(contact_hoist_sxW - p_anchor1)
        W_rope_axis_dx = (contact_hoist_dxW - p_anchor2) / np.linalg.norm(contact_hoist_dxW - p_anchor2)

        # min/max anchor forces
        W_rope_force_sx = np.hstack((-W_rope_axis_sx.reshape(3, 1) * max_rope_force, np.zeros((3, 1))))
        W_rope_force_dx = np.hstack((-W_rope_axis_dx.reshape(3, 1) * max_rope_force, np.zeros((3, 1))))

        print("Rope force manifold sx in WF (colunmn wise)\n", W_rope_force_sx)
        print("Rope force manifold dx in WF(colunmn wise)\n", W_rope_force_dx)

        FC1 = comp_dyn.constr.frictionConeConstr.linearized_cone_vertices(num_generators, mu, cone_height=max_leg_force,
                                                                          normal=wall_normal).T
        FC2 = comp_dyn.constr.frictionConeConstr.linearized_cone_vertices(num_generators, mu, cone_height=max_leg_force,
                                                                          normal=wall_normal).T

        # The order you use to append the feasible sets should match the order of the contacts
        friction_cone_v = []
        friction_cone_v.append(FC1)
        friction_cone_v.append(FC2)
        friction_cone_v.append(W_rope_force_sx)
        friction_cone_v.append(W_rope_force_dx)

        feasible_sets_6D = fwp.computeAngularPart(contactsWF.T, activeContacts, activeContactsIndex, friction_cone_v)

        print("6-D force sets:", feasible_sets_6D)
        FWP = fwp.minkowskySum(feasible_sets_6D)
        print("Number of vertices", np.shape(FWP)[1])

        # Compute centroidal wrench
        mass = 15.07
        external_wrench = [0] * 6
        w_gi = comp_dyn.rbd.computeCentroidalWrench(mass, comWF, external_wrench)
        isFWPStable = fwp.checkDynamicStability(FWP, w_gi)
        self.assertEqual(isFWPStable, True)

        # '''I now check whether the given CoM configuration is having any operation margin'''
        direction_of_max_wrench = np.array([-1, 0, 0, 0, 0, 0])
        res = self.computeMargin(FWP, direction_v=direction_of_max_wrench, static_wrench=w_gi, type_of_margin='3D')

        print("Result", res.x)
        self.assertEqual(res.x[0], -36.487051829840745)
        self.assertEqual(res.x[1], 0)
        self.assertEqual(res.x[2], 0)

        points = FWP[:3, :].T
        hull = ConvexHull(points)
        print("Hull", hull.vertices)
        self.assertEqual(len(hull.vertices), 14)

    