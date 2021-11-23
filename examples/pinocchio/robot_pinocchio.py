from __future__ import print_function

from jet_leg.kinematics.kinematics_pinocchio import robotKinematics
from pinocchio.utils import *

import time

''' Must include robot urdf and data in resources/urdfs/'''

robotName = 'aliengo'
LF_foot = np.array([0.2331, 0.10884, -0.19765])
RF_foot = np.array([0.2331, -0.10884, -0.1976])
LH_foot = np.array([-0.2183, 0.10884, -0.1976])
RH_foot = np.array([-0.2183, -0.10884, -0.1976])
# LC_foot = np.array([0.0152, 0.3777, -0.1912])
# RC_foot = np.array([0.0152, -0.3777,-0.1912])

# feet_pos_des = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot, LC_foot, RC_foot))
feet_pos_des = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))
print ("feet_pos_des: ", feet_pos_des)

kin = robotKinematics(robotName)
start_time = time.time()
q = kin.fixedBaseInverseKinematics(feet_pos_des)
print('total time is ',time.time()-start_time)
print('q is:', q.T)
print('\nresult: %s' % q.flatten().tolist())

