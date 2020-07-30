# ############################################
# An example for doing computed torque control
#
# Author : Deepak Raina @ IIT Delhi
# Version : 0.1
# ############################################

import numpy as np
from pybullet_controller import RobotController

robot = RobotController(robot_type='ur5')
robot.createWorld(GUI=True)

# Computed torque controller
# Input: numpy array of joint angles
thi = np.array([0, 0, 0, 0, 0, 0]) # initial joint angles
thf = np.array([-1.5, -1.0, 1.0, -1.57, -1.57, -1.57]) # final joint nagles
robot.setJointPosition(thi)
robot.computedTorqueControl(thi, thf)


