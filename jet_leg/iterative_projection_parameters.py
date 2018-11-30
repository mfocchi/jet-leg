# -*- coding: utf-8 -*-
"""
Created on Wed Nov 14 15:07:45 2018

@author: Romeo Orsolino
"""
import numpy as np
from math_tools import Math

class IterativeProjectionParameters:
    def __init__(self):
        self.math = Math()
        self.comPositionBF = [0., 0., 0.]
        self.footPosLF = [0.3, 0.25, -.5]
        self.footPosRF = [-0.3, 0.25, -.5]
        self.footPosLH = [0.3, -0.25, -.5]
        self.footPosRH = [-0.3, -0.25, -.5]

        self.roll = 0.0
        self.pitch= 0.0
    
        self.LF_tau_lim = [50.0, 50.0, 50.0]
        self.RF_tau_lim = [50.0, 50.0, 50.0]
        self.LH_tau_lim = [50.0, 50.0, 50.0]
        self.RH_tau_lim = [50.0, 50.0, 50.0]
        self.torque_limits = np.array([self.LF_tau_lim, self.RF_tau_lim, self.LH_tau_lim, self.RH_tau_lim])
        
        self.state_machineLF = True
        self.state_machineRF = True
        self.state_machineLH = True
        self.state_machineRH = True
        self.stanceFeet = [0, 0, 0, 0]
        self.numberOfContacts = 0
        self.contactsHF = np.zeros((4,3))
        self.contactsBF = np.zeros((4,3))


        self.math = Math()        
        axisZ= np.array([[0.0], [0.0], [1.0]])
        n1 = np.transpose(np.transpose(self.math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n2 = np.transpose(np.transpose(self.math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n3 = np.transpose(np.transpose(self.math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n4 = np.transpose(np.transpose(self.math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        # %% Cell 2
        self.normals = np.vstack([n1, n2, n3, n4])
        self.constraintMode = ['FRICTION_AND_ACTUATION',
                               'FRICTION_AND_ACTUATION',
                               'FRICTION_AND_ACTUATION',
                               'FRICTION_AND_ACTUATION']
                               
        self.friction = 0.8
        self.robotMass = 85 #Kg
        self.numberOfGenerators = 4
            
    
        #foothold planning 
        self.footOption0 = [0., 0., 0.]
        self.footOption1 = [0., 0., 0.]
        self.footOption2 = [0., 0., 0.]
        self.footOption3 = [0., 0., 0.]
        self.footOption4 = [0., 0., 0.]
        self.footOptions = np.array([self.footOption0,
                                     self.footOption1,
                                     self.footOption2,
                                     self.footOption3,
                                     self.footOption4])
                
        self.actual_swing = 0
        self.optimization_started = False

    def setContactsPosBF(self, contactsBF):
        self.contactsBF = contactsBF

    def setCoMPos(self, comWF):
        self.comPositionBF = comWF
    
    def setTorqueLims(self, torqueLims):
        self.torque_limits = torqueLims
        
    def setActiveContacts(self, activeContacts):
        self.stanceFeet = activeContacts
        #print self.stanceFeet
        
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
        
    def getContactsPosBF(self):
        return self.contactsBF
        
    def getCoMPos(self):
        return self.comPositionBF
        
    def getTorqueLims(self):
        return self.torque_limits
        
    def getStanceFeet(self):
        return self.stanceFeet
        
    def getNormals(self):

        return self.normals
        
    def getConstraintModes(self):
        return self.constraintMode
        
    def getFrictionCoefficient(self):
        return self.friction
        
    def getNumberOfFrictionConesEdges(self):
        return self.numberOfGenerators
        
    def getTotalMass(self):
        return self.robotMass
        
    def getParamsFromRosDebugTopic(self, received_data):
        
        num_of_elements = np.size(received_data.data)
#        print 'number of elements: ', num_of_elements
        for j in range(0,num_of_elements):
#            print j, received_data.name[j], str(received_data.name[j]), str("footPosLFx")
            if str(received_data.name[j]) == str("LF_HAAminVar"):
                self.LF_tau_lim[0] = received_data.data[j]           
            if str(received_data.name[j]) == str("LF_HFEminVar"):
                self.LF_tau_lim[1] = received_data.data[j]
            if str(received_data.name[j]) == str("LF_KFEminVar"):
                self.LF_tau_lim[2] = received_data.data[j]
            if str(received_data.name[j]) == str("RF_HAAminVar"):
                self.RF_tau_lim[0] = received_data.data[j]           
            if str(received_data.name[j]) == str("RF_HFEminVar"):
                self.RF_tau_lim[1] = received_data.data[j]
            if str(received_data.name[j]) == str("RF_KFEminVar"):
                self.RF_tau_lim[2] = received_data.data[j]
            if str(received_data.name[j]) == str("LH_HAAminVar"):
                self.LH_tau_lim[0] = received_data.data[j]           
            if str(received_data.name[j]) == str("LH_HFEminVar"):
                self.LH_tau_lim[1] = received_data.data[j]
            if str(received_data.name[j]) == str("LH_KFEminVar"):
                self.LH_tau_lim[2] = received_data.data[j]
            if str(received_data.name[j]) == str("RH_HAAminVar"):
                self.RH_tau_lim[0] = received_data.data[j]           
            if str(received_data.name[j]) == str("RH_HFEminVar"):
                self.RH_tau_lim[1] = received_data.data[j]
            if str(received_data.name[j]) == str("RH_KFEminVar"):
                self.RH_tau_lim[2] = received_data.data[j]
                
            self.torque_limits = np.array([self.LF_tau_lim, 
                                           self.RF_tau_lim, 
                                           self.LH_tau_lim, 
                                           self.RH_tau_lim, ])
                                                                                    
#            print 'torque lims',self.torque_limits
            
            if str(received_data.name[j]) == str("footPosLFx"):
                self.footPosLF[0] = int(received_data.data[j]*100.0)/100.0
            if str(received_data.name[j]) == str("footPosLFy"):
                self.footPosLF[1] = int(received_data.data[j]*100.0)/100.0
            if str(received_data.name[j]) == str("footPosLFz"):
                self.footPosLF[2] = int(received_data.data[j]*100.0)/100.0
            if str(received_data.name[j]) == str("footPosRFx"):
                self.footPosRF[0] = int(received_data.data[j]*100.0)/100.0
            if str(received_data.name[j]) == str("footPosRFy"):
                self.footPosRF[1] = int(received_data.data[j]*100.0)/100.0
            if str(received_data.name[j]) == str("footPosRFz"):
                self.footPosRF[2] = int(received_data.data[j]*100.0)/100.0
            if str(received_data.name[j]) == str("footPosLHx"):
                self.footPosLH[0] = int(received_data.data[j]*100.0)/100.0
            if str(received_data.name[j]) == str("footPosLHy"):
                self.footPosLH[1] = int(received_data.data[j]*100.0)/100.0
            if str(received_data.name[j]) == str("footPosLHz"):
                self.footPosLH[2] = int(received_data.data[j]*100.0)/100.0
            if str(received_data.name[j]) == str("footPosRHx"):
                self.footPosRH[0] = int(received_data.data[j]*100.0)/100.0
            if str(received_data.name[j]) == str("footPosRHy"):
                self.footPosRH[1] = int(received_data.data[j]*100.0)/100.0
            if str(received_data.name[j]) == str("footPosRHz"):
                self.footPosRH[2] = int(received_data.data[j]*100.0)/100.0

            if str(received_data.name[j]) == str("offCoMX"):
                self.comPositionBF[0] = received_data.data[j]
            if str(received_data.name[j]) == str("offCoMY"):
                self.comPositionBF[1] = received_data.data[j]                       
            if str(received_data.name[j]) == str("offCoMZ"):
                self.comPositionBF[2] = received_data.data[j]        
                
                
#            define feet in centroidal frame
            self.contactsHF = np.array([[np.dot( np.transpose(self.math.rpyToRot(self.roll,self.pitch,0.0)), np.subtract(self.footPosLF, self.comPositionBF))],
                                       [np.dot( np.transpose(self.math.rpyToRot(self.roll,self.pitch,0.0)), np.subtract(self.footPosRF, self.comPositionBF))],
                                        [np.dot( np.transpose(self.math.rpyToRot(self.roll,self.pitch,0.0)), np.subtract(self.footPosLH, self.comPositionBF))],
                                            [np.dot( np.transpose(self.math.rpyToRot(self.roll,self.pitch,0.0)),np.subtract(self.footPosRH, self.comPositionBF))]])

            self.contactsBF = np.array([ self.footPosLF,self.footPosRF,self.footPosLH, self.footPosRH]) 
                

            if str(received_data.name[j]) == str("normalLFx"):
                self.normals[0,0] = received_data.data[j]  
            if str(received_data.name[j]) == str("normalLFy"):
                self.normals[0,1] = received_data.data[j] 
            if str(received_data.name[j]) == str("normalLFz"):
                self.normals[0,2] = received_data.data[j]                  
                                                 
            if str(received_data.name[j]) == str("normalRFx"):
                self.normals[1,0] = received_data.data[j]
            if str(received_data.name[j]) == str("normalRFy"):
                self.normals[1,1] = received_data.data[j] 
            if str(received_data.name[j]) == str("normalRFz"):
                self.normals[1,2] = received_data.data[j]                 
                                                 
            if str(received_data.name[j]) == str("normalLHx"):
                self.normals[2,0] = received_data.data[j] 
            if str(received_data.name[j]) == str("normalLHy"):
                self.normals[2,1] = received_data.data[j] 
            if str(received_data.name[j]) == str("normalLHz"):
                self.normals[2,2] = received_data.data[j]                
                                                 
            if str(received_data.name[j]) == str("normalRHx"):
                self.normals[3,0] = received_data.data[j]  
            if str(received_data.name[j]) == str("normalRHy"):
                self.normals[3,1] = received_data.data[j]  
            if str(received_data.name[j]) == str("normalRHz"):
                self.normals[3,2] = received_data.data[j]                  
                
    
            if str(received_data.name[j]) == str("robotMass"):
                self.robotMass = received_data.data[j] 
           
            if str(received_data.name[j]) == str("muEstimate"):
                self.friction = received_data.data[j]

            if str(received_data.name[j]) == str("roll"):
                self.roll = received_data.data[j]             
            if str(received_data.name[j]) == str("pitch"):
                self.pitch = received_data.data[j]    
                
            #foothold planning
                
            if str(received_data.name[j]) == str("foothold_option0x"):  
                self.footOption0[0] = received_data.data[j]
            if str(received_data.name[j]) == str("foothold_option0y"):  
                self.footOption0[1] = received_data.data[j]                   
            if str(received_data.name[j]) == str("foothold_option0z"):  
                self.footOption0[2] = received_data.data[j]    
                
            if str(received_data.name[j]) == str("foothold_option1x"):  
                self.footOption1[0] = received_data.data[j]
            if str(received_data.name[j]) == str("foothold_option1y"):  
                self.footOption1[1] = received_data.data[j]                   
            if str(received_data.name[j]) == str("foothold_option1z"):  
                self.footOption1[2] = received_data.data[j]   

            if str(received_data.name[j]) == str("foothold_option2x"):  
                self.footOption2[0] = received_data.data[j]
            if str(received_data.name[j]) == str("foothold_option2y"):  
                self.footOption2[1] = received_data.data[j]                   
            if str(received_data.name[j]) == str("foothold_option2z"):  
                self.footOption2[2] = received_data.data[j]   

            if str(received_data.name[j]) == str("foothold_option3x"):  
                self.footOption3[0] = received_data.data[j]
            if str(received_data.name[j]) == str("foothold_option3y"):  
                self.footOption3[1] = received_data.data[j]                   
            if str(received_data.name[j]) == str("foothold_option3z"):  
                self.footOption3[2] = received_data.data[j]   

            if str(received_data.name[j]) == str("foothold_option4x"):  
                self.footOption4[0] = received_data.data[j]
            if str(received_data.name[j]) == str("foothold_option4y"):  
                self.footOption4[1] = received_data.data[j]                   
            if str(received_data.name[j]) == str("foothold_option4z"):  
                self.footOption4[2] = received_data.data[j]  
                
            self.footOptions = np.array([self.footOption0,
                                     self.footOption1,
                                     self.footOption2,
                                     self.footOption3,
                                     self.footOption4])
#            print self.footOptions

            if str(received_data.name[j]) == str("actual_swing"):  
                self.actual_swing = int(received_data.data[j])
                
            if str(received_data.name[j]) == str("optimization_started"):  
                self.optimization_started = bool(received_data.data[j])
                                   
          
    def getFutureStanceFeet(self, received_data):

        num_of_elements = np.size(received_data.data)         
        for j in range(0,num_of_elements):
            if str(received_data.name[j]) == str("future_stance_LF"):
                self.stanceFeet[0] = int(received_data.data[j])
            if str(received_data.name[j]) == str("future_stance_RF"):
                self.stanceFeet[1] = int(received_data.data[j])
            if str(received_data.name[j]) == str("future_stance_LH"):
                self.stanceFeet[2] = int(received_data.data[j])
            if str(received_data.name[j]) == str("future_stance_RH"):
                self.stanceFeet[3] = int(received_data.data[j])  
              
        self.numberOfContacts = np.sum(self.stanceFeet)
        
    def getCurrentStanceFeet(self, received_data):
        
        num_of_elements = np.size(received_data.data)
        for j in range(0,num_of_elements):
            if str(received_data.name[j]) == str("state_machineLF"):
                self.stanceFeet[0] = int(received_data.data[j])
            if str(received_data.name[j]) == str("state_machineRF"):
                self.stanceFeet[1] = int(received_data.data[j])
            if str(received_data.name[j]) == str("state_machineLH"):
                self.stanceFeet[2] = int(received_data.data[j])
            if str(received_data.name[j]) == str("state_machineRH"):
                self.stanceFeet[3] = int(received_data.data[j]) 
                
            if self.state_machineLF < 4.0:
                self.stanceFeet[0] = 1
            else:
                self.stanceFeet[0] = 0
                
            if self.state_machineRF < 4.0:
                self.stanceFeet[1] = 1
            else:
                self.stanceFeet[1] = 0
                
            if self.state_machineLH < 4.0:
                self.stanceFeet[2] = 1
            else:
                self.stanceFeet[2] = 0
                
            if self.state_machineRH < 4.0:
                self.stanceFeet[3] = 1                
            else:
                self.stanceFeet[3] = 0
            
 
        self.numberOfContacts = np.sum(self.stanceFeet)

