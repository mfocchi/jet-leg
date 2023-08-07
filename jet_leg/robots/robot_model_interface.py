# -*- coding: utf-8 -*-
"""
Created on Mon Jul  2 05:34:42 2018

@author: romeo orsolino
"""
import numpy as np

from jet_leg.robots.hyq.hyq_model import HyqModel
from jet_leg.robots.anymal.anymal_model import AnymalModel
from jet_leg.robots.hyqreal.hyqreal_model import HyqrealModel
from jet_leg.robots.crex.crex_model import CrexModel
from jet_leg.robots.aliengo.aliengo_model import AliengoModel
from jet_leg.robots.go1.go1_model import  Go1Model

class RobotModelInterface:
    def __init__(self, robot_name):
        self.robotName = robot_name
        if self.robotName == 'hyq':
            self.robotModel = HyqModel()
        elif self.robotName == 'anymal':
            self.robotModel = AnymalModel()
        elif self.robotName == 'hyqreal':
            self.robotModel = HyqrealModel()
        elif self.robotName == 'hyqreal':
            self.robotModel = HyqrealModel()
        elif self.robotName == 'crex':
            self.robotModel = CrexModel()
        elif self.robotName == 'aliengo':
            self.robotModel = AliengoModel()
        elif self.robotName == 'go1':
            self.robotModel = Go1Model()

        self.joint_torque_limits = self.robotModel.joint_torque_limits
        self.contact_torque_limits = self.robotModel.contact_torque_limits