# -*- coding: utf-8 -*-
"""
Created on Mon May 28 13:00:59 2018

@author: Romeo Orsolino
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.computational_geometry.leg_force_polytopes import LegForcePolytopes
from scipy.linalg import block_diag
from jet_leg.robots.dog_interface import DogInterface
from jet_leg.dynamics.rigid_body_dynamics import RigidBodyDynamics
from jet_leg.computational_geometry.polytopes import Polytope
from jet_leg.constraints.friction_cone_constraint import FrictionConeConstraint
from jet_leg.constraints.force_polytope_constraint import ForcePolytopeConstraint

class Constraints:    
    def __init__(self, robot_kinematics=None, robot_model=None):
        #self.robotName = robot_name
        self.kin = robot_kinematics
        self.math = Math()

        self.dog = DogInterface()
        self.rbd = RigidBodyDynamics()
        self.frictionConeConstr = FrictionConeConstraint()
        if robot_kinematics is not None:
            self.forcePolytopeConstr = ForcePolytopeConstraint(robot_kinematics)
        if robot_model is not None:
            self.model = robot_model
    
    def getInequalities(self, params, saturate_normal_force = False):

        stanceLegs = params.getStanceFeet()
        
        #print 'stance legs', stanceLegs
        contactsNumber = np.sum(stanceLegs)
        contactsWF = params.getContactsPosWF()
        comPositionWF = params.getCoMPosWF()
        comPositionBF = params.getCoMPosBF()
        rpy = params.getOrientation()
        #compute the contacs in the base frame for the inv kineamtics
        contactsBF = np.zeros((params.getNoOfLegs(),3))

        for j in np.arange(0, params.getNoOfLegs()):
            j = int(j)
            contactsBF[j,:]= np.add( np.dot(self.math.rpyToRot(rpy[0], rpy[1], rpy[2]), (contactsWF[j,:] - comPositionWF)), comPositionBF)

        #print 'WF ',contactsWF
        #print contactsBF
        #print 'stance legs ', stanceLegs
        
        constraint_mode = params.getConstraintModes()

        tau_lim = params.getTorqueLims()
        contact_torque_lims = self.model.contact_torque_limits
        leg_self_weight = params.getLegSelfWeight()
        ng = params.getNumberOfFrictionConesEdges()
        friction_coeff = params.getFrictionCoefficient()
        normals = params.getNormals()

        C = np.zeros((0,0))
        d = np.zeros((0))

        stanceIndex = params.getStanceIndex(stanceLegs)
        #we are static so we set to zero
        foot_vel = np.array([[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]])

        self.kin.inverse_kin(contactsBF, foot_vel)

        forcePolytopes = LegForcePolytopes(params.getNoOfLegs())
        for j in stanceIndex:
            j = int(j)
            if constraint_mode[j] == 'ONLY_FRICTION':
                #            print contactsNumber
                Ctemp, d_cone = self.frictionConeConstr.linearized_cone_halfspaces_world(params.pointContacts, friction_coeff, normals[j, :], contact_torque_lims)
                isIKoutOfWorkSpace = False
                leg_actuation_polygon = np.zeros((3, 8))
                # n = self.math.normalize(normals[j,:])
                # rotationMatrix = self.math.rotation_matrix_from_normal(n)
                # Ctemp = np.dot(constraints_local_frame, rotationMatrix.T)
            
            if constraint_mode[j] == 'ONLY_ACTUATION':
                Ctemp, d_cone, leg_actuation_polygon, isIKoutOfWorkSpace = self.forcePolytopeConstr.compute_actuation_constraints(j, tau_lim, leg_self_weight, rpy , params.pointContacts, contact_torque_lims)

                if isIKoutOfWorkSpace is False:
                    if params.pointContacts:
                        d_cone = d_cone.reshape(6)
                    else:
                        d_cone = d_cone.reshape(10)
                else:
                    Ctemp = np.zeros((0,0))
                    d_cone = np.zeros((0))
            
            if constraint_mode[j] == 'FRICTION_AND_ACTUATION':
                C1, d1, leg_actuation_polygon, isIKoutOfWorkSpace = self.forcePolytopeConstr.compute_actuation_constraints(j, tau_lim, leg_self_weight, rpy, params.pointContacts, contact_torque_lims)
                C2, d2 = self.frictionConeConstr.linearized_cone_halfspaces_world(params.pointContacts, friction_coeff, normals[j, :], contact_torque_lims)

                if isIKoutOfWorkSpace is False:
                    #                print d1
                    Ctemp = np.vstack([C1, C2])
                    #               print np.size(C,0), np.size(C,1), C
                    d_cone = np.hstack([d1[0], d2])
                    #                print d
                    if params.pointContacts:
                        d_cone = d_cone.reshape((6 + ng))
                    else:
                        d_cone = d_cone.reshape((10 + ng + 4))
                else:
                    Ctemp = np.zeros((0,0))
                    d_cone = np.zeros((0))

            currentLegForcePolytope = Polytope()
            currentLegForcePolytope.setHalfSpaces(Ctemp, d_cone)

            currentLegForcePolytope.setVertices(leg_actuation_polygon)
            # print np.shape(currentLegForcePolytope.getVertices())
            forcePolytopes.forcePolytope[j] = currentLegForcePolytope
                
            C = block_diag(C, Ctemp)
            d = np.hstack([d, d_cone])

        
        if contactsNumber == 0:
            print('contactsNumber is zero, there are no stance legs set! This might be because Gazebo is in pause.')
            
        return C, d, isIKoutOfWorkSpace, forcePolytopes
    