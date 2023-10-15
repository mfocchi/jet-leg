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


plt.close('all')
math = Math()

class TestClimbingRobot(unittest.TestCase):
    def plot_FWP(self, FWP, max_force = np.array([0,0,0, 0,0,0]), title="FWP"):
        fig = plt.figure(1)
        ax = fig.add_subplot(111, projection="3d")
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        ax.scatter(FWP[0, :], FWP[1, :], FWP[2, :], marker='o', color='royalblue', alpha=0.2)
        ax.scatter(max_force[0], max_force[1], max_force[2], marker='o', color='r', s=1000)
        plt.title(title)
        points = FWP[:3, :].T
        FWP3d_hull = ConvexHull(points)
        for i in FWP3d_hull.simplices:
            plt.plot(points[i, 0], points[i, 1], points[i, 2], color='b')

        vertices = points[FWP3d_hull.vertices]
        ax.scatter(vertices[:, 0], vertices[:, 1], vertices[:, 2], marker='o', color='b')

        plt.show()

    def computeMargin(self, FWP, direction_v =np.array([1,0,0,0,0,0]), offset =  np.array([0,0,0,0,0,0])):

        # # be sure is a unit vector
        # direction_v_unit= direction_v/ np.linalg.norm(direction_v)
        # # FWP_hull = ConvexHull(FWP.T, qhull_options="QJ")
        # c = -direction_v_unit
        # A_ub = FWP_hull.equations[:, :-1]
        # b_ub = -FWP_hull.equations[:, -1]
        # A_eq = np.eye(6) -np.outer(direction_v_unit,direction_v_unit.T)
        # b_eq = offset

        # for debugging use 3d version
        FWP_hull3d = ConvexHull(FWP[:3, :].T)#, qhull_options="QJ")
        # be sure is a unit vector
        direction_v_unit = direction_v[:3] / np.linalg.norm(direction_v[:3])
        c = -direction_v_unit
        print("Equations", FWP_hull3d.equations)
        print("Direction", c)
        A_ub = FWP_hull3d.equations[:, :-1]
        b_ub = -FWP_hull3d.equations[:, -1]
        # A_eq = np.eye(3) -np.outer(direction_v_unit,direction_v_unit.T)
        # b_eq = np.array([0,1600,0])

        A_eq = None
        b_eq = None

        bound = (-np.inf, np.inf)

        res = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq, bounds=[bound]*len(c),
                    callback=None, options=None, x0=None)
        #debug
        # res = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=None, b_eq=None, bounds=None, method='highs-ds',
        #               callback=None, options=None, x0=None)
        print(A_eq)
        #print("Max force Fz", res.x[2])
        return res
    
    def test_rectangle(self):
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


        FWP = np.array([[150, -150, 150, -150, 150, -150, 150, -150],
        [-100, -100, 100, 100, -100, -100, 100, 100],
        [-200, -200, -200, -200, 200, 200, 200, 200]])
        print("vertices", FWP)

        '''I now check whether the given CoM configuration is dynamically stable or not (see "Feasible Wrench Polytope")'''
        w_gi = np.array([0., 0., 0.0])
        isFWPStable = fwp.checkDynamicStability(FWP, w_gi)
        # self.assertEqual(isFWPStable, True)

        res = self.computeMargin(FWP, np.array([1, 1, 1, 0, 0, 0]), offset = np.array([0,1600,0,0,0,0]))
        print("Result", res.x)
        self.assertEqual(res.x[0], 150)
        self.assertEqual(res.x[1], 100)
        self.assertEqual(res.x[2], 200)

        res = self.computeMargin(FWP, np.array([1, -1, 1, 0, 0, 0]), offset = np.array([0,1600,0,0,0,0]))
        print("Result", res.x)
        self.assertEqual(res.x[0], 150)
        self.assertEqual(res.x[1], -100)
        self.assertEqual(res.x[2], 200)

        res = self.computeMargin(FWP, np.array([1, 1, -1, 0, 0, 0]), offset = np.array([0,1600,0,0,0,0]))
        print("Result", res.x)
        self.assertEqual(res.x[0], 150)
        self.assertEqual(res.x[1], 100)
        self.assertEqual(res.x[2], -200)

        res = self.computeMargin(FWP, np.array([-1, -1, -1, 0, 0, 0]), offset = np.array([0,1600,0,0,0,0]))
        print("Result", res.x)
        self.assertEqual(res.x[0], -150)
        self.assertEqual(res.x[1], -100)
        self.assertEqual(res.x[2], -200)
        # self.plot_FWP(FWP, res.x)


        # fig = plt.figure(2)
        # ax = fig.add_subplot(111, projection="3d")
        # ax.set_xlabel('X axis')
        # ax.set_ylabel('Y axis')
        # ax.set_zlabel('Z axis')
        # # ax.scatter(FWP[0, :], FWP[1, :], FWP[2, :], marker='o', color='purple')

        points = FWP[:3, :].T
        hull = ConvexHull(points)
        # for i in hull.simplices:
        #     plt.plot(points[i, 0], points[i, 1], points[i, 2], 'r-')
        # plt.plot(points[hull.vertices,0], points[hull.vertices,1], points[hull.vertices,2], 'k-')

        print("Hull", hull.vertices)
        self.assertEqual(len(hull.vertices), 8)

        # plt.show()

    def test_linearized_cone(self):
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

        n1 =  np.array([1, 0, 0])
        FC1 = comp_dyn.constr.frictionConeConstr.linearized_cone_vertices(num_generators, mu, cone_height=max_normal_force, normal=n1).T

        FWP = FC1 #fwp.minkowskySum(feasible_sets_6D)
        print("Number of vertices", np.shape(FWP))

        '''I now check whether the given CoM configuration is dynamically stable or not (see "Feasible Wrench Polytope")'''
        w_gi = np.array([0., 0., 0.])
        isFWPStable = fwp.checkDynamicStability(FWP, w_gi)
        self.assertEqual(isFWPStable, True)

        res = self.computeMargin(FWP, np.array([1, 0, 0, 0, 0, 0]), offset = np.array([0,1600,0,0,0,0]))
        print("Result", res.x)
        self.assertEqual(res.x[0], 500)

        res = self.computeMargin(FWP, np.array([-1, 0, 0, 0, 0, 0]), offset = np.array([0,1600,0,0,0,0]))
        print("Result", res.x)
        self.assertEqual(res.x[0], 0)
        self.assertEqual(res.x[1], 0)
        self.assertEqual(res.x[2], 0)
        # self.plot_FWP(FWP, res.x)


        # fig = plt.figure(2)
        # ax = fig.add_subplot(111, projection="3d")
        # ax.set_xlabel('X axis')
        # ax.set_ylabel('Y axis')
        # ax.set_zlabel('Z axis')
        # # ax.scatter(FWP[0, :], FWP[1, :], FWP[2, :], marker='o', color='purple')

        points = FWP[:3, :].T
        hull = ConvexHull(points)
        # for i in hull.simplices:
        #     plt.plot(points[i, 0], points[i, 1], points[i, 2], 'r-')
        # plt.plot(points[hull.vertices,0], points[hull.vertices,1], points[hull.vertices,2], 'k-')

        print("Hull", hull.vertices)
        self.assertEqual(len(hull.vertices), 5)

        # plt.show()

    def test_climbing(self):
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

        # Set contact points
        comWF = np.array([0., 0., 0.0])

        lower_landing_leg = 0.3
        offset_base_y = 0.08
        contact1 = np.array([-lower_landing_leg*np.sin(np.pi/3), - offset_base_y - lower_landing_leg*np.cos(np.pi/3), 0.025]) # left foot
        contact2 = np.array([-lower_landing_leg*np.sin(np.pi/3), offset_base_y + lower_landing_leg*np.cos(np.pi/3), 0.025]) # right foot
        contact3 = np.array([0, -0.05, 0.05]) # rope (attachment)
        contact4 = np.array([0, 0.05, 0.05]) # rope (attachment)

        contactsWF = np.vstack((contact1+comWF, contact2+comWF, contact3+comWF, contact4+comWF))
        # print("Contacts", contactsWF)

        n1 =  np.array([1, 0, 0])
        FC1 = comp_dyn.constr.frictionConeConstr.linearized_cone_vertices(num_generators, mu, cone_height=max_normal_force, normal=n1).T

        n2 =  np.array([1, 0, 0])
        FC2 = comp_dyn.constr.frictionConeConstr.linearized_cone_vertices(num_generators, mu, cone_height=max_normal_force, normal=n2).T

        # TO DO: compute the segments as a function of the rope angle
        line1 = np.array([[0, 100],
                            [0, -500],
                            [0, 100]])

        line2 = np.array([[0, 100],
                            [0, 500],
                            [0, 100]])

        # The order you use to append the feasible sets should match the order of the contacts
        friction_cone_v = []
        friction_cone_v.append(FC1)
        friction_cone_v.append(FC2)
        friction_cone_v.append(line1)
        friction_cone_v.append(line2) 

        # print("Friction cove v", friction_cone_v)
        feasible_sets_6D = fwp.computeAngularPart(contactsWF.T, stanceLegs, stanceIndex, friction_cone_v)
        # feasible_sets_6D = friction_cone_v

        # print("6-D force sets:", feasible_sets_6D)

        FWP = fwp.minkowskySum(feasible_sets_6D)
        # print("Number of vertices", np.shape(FWP)[1])

        '''I now check whether the given CoM configuration is dynamically stable or not (see "Feasible Wrench Polytope")'''
        start = time.time()
        mass = 0
        external_wrench = [0]*6
        w_gi = comp_dyn.rbd.computeCentroidalWrench(mass, comWF, external_wrench)
        isFWPStable = fwp.checkDynamicStability(FWP, w_gi)
        self.assertEqual(isFWPStable, True)

        res = self.computeMargin(FWP, np.array([0.,  0.5,  1. ,0,0,0]), offset = np.array([0,1600,0,0,0,0]))
        print("Result", res.x)
        # self.plot_FWP(FWP, res.x)


        # fig = plt.figure(2)
        # ax = fig.add_subplot(111, projection="3d")
        # ax.set_xlabel('X axis')
        # ax.set_ylabel('Y axis')
        # ax.set_zlabel('Z axis')
        # ax.scatter(FWP[0, :], FWP[1, :], FWP[2, :], marker='o', color='purple')

        points = FWP[:3, :].T
        hull = ConvexHull(points)
        # for i in hull.simplices:
        #     plt.plot(points[i, 0], points[i, 1], points[i, 2], 'r-')
        # plt.plot(points[hull.vertices,0], points[hull.vertices,1], points[hull.vertices,2], 'k-')
        # plt.show()

        print("Hull", hull.vertices)
        self.assertEqual(len(hull.vertices), 14)

