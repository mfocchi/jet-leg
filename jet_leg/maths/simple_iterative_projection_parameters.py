# -*- coding: utf-8 -*-
"""
Created on Wed Nov 14 15:07:45 2018

@author: Romeo Orsolino
"""
import numpy as np
from math_tools import Math


class IterativeProjectionParameters:
	def __init__(self):

		self.no_of_legs = 4
		self.stride = 3

		self.math = Math()
		self.q = [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
		self.comPositionBF = [0., 0., 0.]  # var used only for IK inside constraints.py
		self.comPositionWF = [0., 0., 0.]
		self.footPosWLF = [0.3, 0.2, -.0]
		self.footPosWRF = [0.3, -0.2, -.0]
		self.footPosWLH = [-0.3, 0.2, -.0]
		self.footPosWRH = [-0.3, -0.2, -.0]
		self.externalForceWF = np.array([0., 0., 0.])
		self.externalTorqueWF = np.array([0., 0., 0.])

		self.roll = 0.0
		self.pitch = 0.0
		self.yaw = 0.0

		self.LF_tau_lim = [50.0, 50.0, 50.0]
		self.RF_tau_lim = [50.0, 50.0, 50.0]
		self.LH_tau_lim = [50.0, 50.0, 50.0]
		self.RH_tau_lim = [50.0, 50.0, 50.0]

		self.LF_q_lim_max = [0., 0., 0.]
		self.LF_q_lim_min = [0., 0., 0.]
		self.RF_q_lim_max = [0., 0., 0.]
		self.RF_q_lim_min = [0., 0., 0.]
		self.LH_q_lim_max = [0., 0., 0.]
		self.LH_q_lim_min = [0., 0., 0.]
		self.RH_q_lim_max = [0., 0., 0.]
		self.RH_q_lim_min = [0., 0., 0.]

		self.torque_limits = np.array([self.LF_tau_lim, self.RF_tau_lim, self.LH_tau_lim, self.RH_tau_lim])
		self.leg_self_weight = np.array([[0., 0., 0.], [0., 0., 0.], [0., 0., 0.], [0., 0., 0.]])
		self.joint_limits_max = np.array([self.LF_q_lim_max, self.RF_q_lim_max, self.LH_q_lim_max, self.RH_q_lim_max])
		self.joint_limits_min = np.array([self.LF_q_lim_min, self.RF_q_lim_min, self.LH_q_lim_min, self.RH_q_lim_min])

		self.state_machineLF = True
		self.state_machineRF = True
		self.state_machineLH = True
		self.state_machineRH = True
		self.state_machine = True
		self.stanceFeet = [0, 0, 0, 0]
		self.numberOfContacts = 0
		#        self.contactsHF = np.zeros((4,3))
		self.contactsBF = np.zeros((4, 3))
		self.contactsWF = np.zeros((4, 3))

		axisZ = np.array([[0.0], [0.0], [1.0]])
		n1 = np.transpose(np.transpose(self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))
		n2 = np.transpose(np.transpose(self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))
		n3 = np.transpose(np.transpose(self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))
		n4 = np.transpose(np.transpose(self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))
		# %% Cell 2
		self.normals = np.vstack([n1, n2, n3, n4])
		self.constraintMode = ['FRICTION_AND_ACTUATION',
							   'FRICTION_AND_ACTUATION',
							   'FRICTION_AND_ACTUATION',
							   'FRICTION_AND_ACTUATION']

		self.friction = 0.8
		self.robotMass = 85  # Kg
		self.numberOfGenerators = 4

		self.actual_swing = 0

		self.plane_normal = [0, 0, 1]

		# CoM height (projected along z axis)
		self.com_vertical_shift = 0

		self.desired_acceleration = np.array([0,0,0])

	def setContactsPosBF(self, contactsBF):
		self.contactsBF = contactsBF

	def setContactsPosWF(self, contactsWF):
		self.contactsWF = contactsWF

	def setCoMPosWF(self, comWF):
		self.comPositionWF = comWF

	def setCoMPosBF(self, comBF):
		self.comPositionBF = comBF

	def setOrientation(self, rpy):
		self.roll = rpy[0]
		self.pitch = rpy[1]
		self.yaw = rpy[2]

	def setTorqueLims(self, torqueLims):
		self.torque_limits = torqueLims

	def setJointLimsMax(self, jointLims_max):
		self.joint_limits_max = jointLims_max

	def setJointLimsMin(self, jointLims_min):
		self.joint_limits_min = jointLims_min

	def setActiveContacts(self, activeContacts):
		self.stanceFeet = activeContacts

	# print self.stanceFeet

	def setContactNormals(self, normals):
		self.normals = normals

	def setConstraintModes(self, constraintMode):
		self.constraintMode = constraintMode

	def setFrictionCoefficient(self, mu):
		self.friction = mu

	def setNumberOfFrictionConesEdges(self, ng):
		self.numberOfGenerators = ng

	def setTotalMass(self, mass):
		self.robotMass = mass

	def set_plane_normal(self, plane_normal):
		self.plane_normal = plane_normal

	def getContactsPosWF(self):
		return self.contactsWF

	def getContactsPosBF(self):  # used only for IK inside constraints.py
		return self.contactsBF

	def getCoMPosWF(self):
		return self.comPositionWF

	def getCoMPosBF(self):
		return self.comPositionBF

	def getTorqueLims(self):
		return self.torque_limits
		
	def getLegSelfWeight(self):
		return self.leg_self_weight

	def getJointLimsMax(self):
		return self.joint_limits_max

	def getJointLimsMin(self):
		return self.joint_limits_min

	def get_external_force(self):
		return self.externalForceWF

	def get_external_torque(self):
		return self.externalTorqueWF

	def getStanceFeet(self):
		return self.stanceFeet

	def getNormals(self):
		return self.normals

	def getOrientation(self):
		return self.roll, self.pitch, self.yaw

	def getConstraintModes(self):
		return self.constraintMode

	def getFrictionCoefficient(self):
		return self.friction

	def getNumberOfFrictionConesEdges(self):
		return self.numberOfGenerators

	def getTotalMass(self):
		return self.robotMass

	def get_plane_normal(self):
		return self.plane_normal

	def get_com_vertical_shift(self):
		return self.com_vertical_shift

	def get_CoM_plane_z_intercept(self):
		point_on_plane = self.comPositionWF
		return self.math.plane_z_intercept(point_on_plane, self.plane_normal)

	def get_terrain_plane_z_intercept(self):
		point_on_plane = self.comPositionWF - np.array([0,0,self.com_vertical_shift])
		return self.math.plane_z_intercept(point_on_plane, self.plane_normal)

	def getStanceIndex(self, stanceLegs):
		stanceIdx = []
		#        print 'stance', stanceLegs
		for iter in range(0, 4):
			if stanceLegs[iter] == 1:
				#                print 'new poly', stanceIndex, iter
				stanceIdx = np.hstack([stanceIdx, int(iter)])
		stanceIdx = stanceIdx.astype(np.int)

		return stanceIdx

	def getParamsFromRosDebugTopic(self, received_data):
		# print 'number of elements: ', num_of_elements
			#            print j, received_data.name[j], str(received_data.name[j]), str("footPosLFx")

		# Torque Limits
		# TO-DO in framework:
		# Changing torque limits in urdf still doesn't change
		# value used by trunk controller and Jet-leg.
		# For now, change manually here.
		for leg in range(0,self.no_of_legs):
			self.torque_limits[leg] = received_data.tau_lim.data[self.stride * leg : self.stride * leg + 3]
			self.leg_self_weight[leg] = received_data.leg_self_weight.data[self.stride * leg : self.stride * leg + 3]


		# Joint Limits
		for leg in range(0,self.no_of_legs):
			self.joint_limits_max[leg] = received_data.q_lim_max.data[self.stride * leg : self.stride * leg + 3]
		for leg in range(0,self.no_of_legs):
			self.joint_limits_min[leg] = received_data.q_lim_min.data[self.stride * leg : self.stride * leg + 3]
#
# 		# the inputs are all in the WF this way we can compute generic regions for generic contact sets and generic com position
		for leg in range(0,self.no_of_legs):
			self.contactsWF[leg] = received_data.contactsWF.data[self.stride * leg : self.stride * leg + 3]

		self.comPositionWF = received_data.actual_CoM

		self.comPositionBF = received_data.off_CoM

		# External wrench
		self.externalForceWF = np.array(received_data.ext_wrench[0:3])
		self.externalTorqueWF = np.array(received_data.ext_wrench[3:6])

# 		# print 'ext force ',self.externalForceWF

# 		# they are in WF
		for leg in range(0,self.no_of_legs):
			self.normals[leg] = received_data.normals.data[self.stride * leg : self.stride * leg + 3]

		self.robotMass = received_data.robot_mass

		self.friction = received_data.mu_estimate

		self.plane_normal = received_data.plane_normal


          #what it matters is the relative position of act com wrt the region, however for visualization purposes we 
		self.com_vertical_shift = received_data.com_vertical_shift
#
		self.roll = received_data.roll
		self.pitch = received_data.pitch
		self.yaw = received_data.yaw
#
		self.actual_swing = received_data.actual_swing # variable doesn't change in framework. Needs fix

		self.desired_acceleration = np.array(received_data.desired_acceleration)

	def getFutureStanceFeetFlags(self, received_data):

		self.stanceFeet = received_data.future_stance_legs

		self.numberOfContacts = np.sum(self.stanceFeet)

	def getCurrentStanceFeetFlags(self, received_data):

		self.state_machine = received_data.current_state_machine_leg

		for leg in range(0, self.no_of_legs):
			if self.state_machine[leg] < 4.0:
				self.stanceFeet[leg] = 1
			else:
				self.stanceFeet[leg] = 0

		self.numberOfContacts = np.sum(self.stanceFeet)
