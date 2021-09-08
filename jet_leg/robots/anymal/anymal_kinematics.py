
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os

import pinocchio
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import *
import yaml


class anymalKinematics():
    def __init__(self):
        self.PKG = os.path.dirname(os.path.abspath(__file__)) + '/../../../resources/urdfs/anymal/'
        self.URDF = self.PKG + 'urdf/anymal_boxy.urdf'
        self.FEET = self.PKG + 'feet_names_ordered.yaml'
        if self.PKG is None:
            self.robot = RobotWrapper.BuildFromURDF(self.URDF)
        else:
            self.robot = RobotWrapper.BuildFromURDF(self.URDF, [self.PKG])
        self.robot = RobotWrapper.BuildFromURDF(self.URDF, [self.PKG])
        self.model = self.robot.model
        self.data = self.robot.data
        self.feet_jac = None
        self.ik_success = False
    
        ## Can be used to compute q in an feet order similar to feet variables
        # Get feet frame names in a similar order to feet variables (position, etc...)
        with open(self.FEET, 'r') as stream:
            try:
                self.urdf_feet_names = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
        # Get feet frame names in an alphabatical order to match pinocchio kinematics
        self.urdf_feet_names_pinocchio = []
        for frame in self.model.frames:
            if frame.name in self.urdf_feet_names:
                self.urdf_feet_names_pinocchio.append(frame.name)
    

    def getBlockIndex(self, frame_name):
        for i in range(len(self.urdf_feet_names_pinocchio)):
            if frame_name == self.urdf_feet_names_pinocchio[i]:
                print(frame_name)
                idx = i * 3
                print(idx)
                break

        return idx

    def footInverseKinematicsFixedBase(self, foot_pos_des, frame_name):
        frame_id = self.model.getFrameId(frame_name)
        blockIdx = self.getBlockIndex(frame_name)
        anymal_q0 = np.vstack([-0.1, 0.7, -1., -0.1, -0.7, 1., 0.1, 0.7, -1., 0.1, -0.7, 1.])
        q = anymal_q0
        eps = 0.005
        IT_MAX = 200
        DT = 1e-1
        err = np.zeros((3, 1))
        e = np.zeros((3, 1))

        i = 0
        while True:
            pinocchio.forwardKinematics(self.model, self.data, q)
            pinocchio.framesForwardKinematics(self.model, self.data, q)
            foot_pos = self.data.oMf[frame_id].translation
            # err = np.hstack([err, (foot_pos - foot_pos_des)])
            # e = err[:,-1]
            # print foot_pos_des[0], foot_pos[[0]], foot_pos[[0]] - foot_pos_des[0]
            # e = foot_pos - foot_pos_des
            e[0] = foot_pos[[0]] - foot_pos_des[0]
            e[1] = foot_pos[[1]] - foot_pos_des[1]
            e[2] = foot_pos[[2]] - foot_pos_des[2]

            J = pinocchio.frameJacobian(self.model, self.data, q, frame_id)
            J_lin = J[:3, :]

            if np.linalg.norm(e) < eps:
                # print("IK Convergence achieved!")
                IKsuccess = True
                break
            if i >= IT_MAX:
                print((
                    "\n Warning: the iterative algorithm has not reached convergence to the desired precision. Error is: ",
                    np.linalg.norm(e)))
                IKsuccess = False
                break
            # print J_lin
            v = np.matmul(- np.linalg.pinv(J_lin), e)
            q = pinocchio.integrate(self.model, q, v * DT)
            i += 1
            # print i

        q_leg = q[blockIdx:blockIdx + 3]
        J_leg = J_lin[:, blockIdx:blockIdx + 3]
        return q_leg, J_leg, err, IKsuccess

    def fixedBaseInverseKinematics(self, feetPosDes):

        no_of_feet = len(self.urdf_feet_names)
        self.feet_jac = []
        q = np.zeros((no_of_feet,3))
        leg_ik_success = np.zeros((no_of_feet))

        for leg in range(no_of_feet):
            '''Compute IK in similar order to feet location variable'''
            f_p_des = np.array(feetPosDes[leg, :]).T
            q[leg], foot_jac, err, leg_ik_success[leg] = self.footInverseKinematicsFixedBase(f_p_des,
                                                                                                self.urdf_feet_names[leg])
            self.feet_jac.append(foot_jac)


        self.ik_success = all(leg_ik_success)

        if self.ik_success is False:
            print('Warning, IK failed. Jacobian is singular')
        return q

    def getLegJacobians(self):
        isOutOfWS = not self.ik_success
        return *self.feet_jac, isOutOfWS
