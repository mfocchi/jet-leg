#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Oct  3 13:38:45 2018

@author: Romeo Orsolino
"""

from examples.ros.talker import talker
import rospy as ros

if __name__ == '__main__':
    
    try:
        talker()

    except ros.ROSInterruptException:
        pass
    

