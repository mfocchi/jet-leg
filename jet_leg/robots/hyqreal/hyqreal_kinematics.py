import pinocchio
from pinocchio.utils import *
from pinocchio.robot_wrapper import RobotWrapper
import matplotlib.pyplot as plt
import os


class hyqrealKinematics():
    def     __init__(self):

        self.PKG = os.path.dirname(os.path.abspath(__file__)) + '/../../../resources/urdfs/hyqreal/'
        self.URDF = self.PKG + 'urdf/hyqreal.urdf'
        if self.PKG is None:
            self.robot = RobotWrapper.BuildFromURDF(self.URDF)
        else:
            self.robot = RobotWrapper.BuildFromURDF(self.URDF, [self.PKG])

        self.model = self.robot.model
        self.data = self.robot.data
        self.LF_foot_jac = None
        self.LH_foot_jac = None
        self.RF_foot_jac = None
        self.RH_foot_jac = None
        self.urdf_foot_name_lf = 'lf_foot'
        self.urdf_foot_name_lh = 'lh_foot'
        self.urdf_foot_name_rf = 'rf_foot'
        self.urdf_foot_name_rh = 'rh_foot'

        self.q0 = np.vstack([-0.18, 0.87, -1.43, -0.18, 0.87, -1.43, -0.24, 0.66, -1.52, -0.24, 0.63, -1.52])

    def getBlockIndex(self, frame_name):
        if frame_name == self.urdf_foot_name_lf:
            idx = 0
        elif frame_name == self.urdf_foot_name_lh:
            idx = 3
        elif frame_name == self.urdf_foot_name_rf:
            idx = 6
        elif frame_name == self.urdf_foot_name_rh:
            idx = 9

        #print ('index is',idx)
        return idx

    def footInverseKinematicsFixedBase(self, foot_pos_des, frame_name):
        frame_id = self.model.getFrameId(frame_name)
        blockIdx = self.getBlockIndex(frame_name)
        eps = 0.005
        IT_MAX = 200
        DT = 1e-1
        err = np.zeros((3, 1))
        e = np.zeros((3, 1))
        q = self.q0

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
                # print(
                #     "\n Warning: the iterative algorithm has not reached convergence to the desired precision. Error is: ",
                #     np.linalg.norm(e))
                IKsuccess = False
                break
            # print J_lin
            v = - np.linalg.pinv(J_lin).dot(e)
            q = pinocchio.integrate(self.model, q, v * DT)
            i += 1
            # print i

        q_leg = q[blockIdx:blockIdx + 3]
        J_leg = J_lin[:, blockIdx:blockIdx + 3]
        return q_leg, J_leg, err, IKsuccess

    def fixedBaseInverseKinematics(self, feetPosDes):

        self.LF_foot_jac = []
        self.LH_foot_jac = []
        self.RF_foot_jac = []
        self.RH_foot_jac = []
        leg_ik_success = np.zeros((4))

        f_p_des = np.array(feetPosDes[0, :]).T
        q_LF, self.LF_foot_jac, err, leg_ik_success[0] = self.footInverseKinematicsFixedBase(f_p_des,
                                                                                             self.urdf_foot_name_lf)

        f_p_des = np.array(feetPosDes[2, :]).T
        q_LH, self.LH_foot_jac, err, leg_ik_success[2] = self.footInverseKinematicsFixedBase(f_p_des,
                                                                                             self.urdf_foot_name_lh)

        f_p_des = np.array(feetPosDes[1, :]).T
        q_RF, self.RF_foot_jac, err, leg_ik_success[1] = self.footInverseKinematicsFixedBase(f_p_des,
                                                                                             self.urdf_foot_name_rf)

        f_p_des = np.array(feetPosDes[3, :]).T
        q_RH, self.RH_foot_jac, err, leg_ik_success[3] = self.footInverseKinematicsFixedBase(f_p_des,
                                                                                             self.urdf_foot_name_rh)

        self.ik_success = bool(leg_ik_success[0] and leg_ik_success[1] and leg_ik_success[2] and leg_ik_success[3])

        # if self.ik_success is False:
        #     print('Warning, IK failed. Jacobian is singular')

        '''please NOTICE here the alphabetical order of the legs in the vector q: LF -> LH -> RF -> RH '''
        q = np.vstack([q_LF, q_RF, q_LH, q_RH])
        return q, self.ik_success

    def getLegJacobians(self):
        isOutOfWS = False
        return self.LF_foot_jac, self.RF_foot_jac, self.LH_foot_jac, self.RH_foot_jac, isOutOfWS

    def isOutOfJointLims(self, joint_positions, joint_limits_max, joint_limits_min):

        no_of_legs_to_check = joint_positions.size/3
        q = joint_positions.reshape((no_of_legs_to_check, 3))
        # print "q: ", q
        # print "leq than max ", np.all(np.less_equal(q, joint_limits_max))
        # print "geq than min ", np.all(np.greater_equal(q, joint_limits_min))
        return not np.all(np.less_equal(q, joint_limits_max)) \
               or not np.all(np.greater_equal(q, joint_limits_min))

    def isOutOfWorkSpace(self, contactsBF_check, joint_limits_max, joint_limits_min, stance_index, foot_vel):
        [q, success] = self.fixedBaseInverseKinematics(contactsBF_check)
        # print "q:", q

        return self.isOutOfJointLims(q, joint_limits_max, joint_limits_min)
