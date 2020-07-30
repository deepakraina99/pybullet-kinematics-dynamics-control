# ############################################
# An example for doing impedence control
#
# Author : Deepak Raina @ IIT Delhi
# Version : 0.1
# ############################################

import numpy as np
from pybullet_controller import RobotController

robot = RobotController(robot_type='ur5')
robot.createWorld(GUI=True)

# Impedence controller
# Input: numpy array of joint angles
thi = np.array([0, -1.0, 1.0, -1.57, -1.57, -1.57]) # initial joint angles
thf = np.array([-1.5, -1.0, 1.0, -1.57, -1.57, -1.57]) # final joint nagles
robot.setJointPosition(thi)
desired_pose = np.array([-0.10857937593446423, 0.7166151451748437, 1.4467087828094798, -1.5700006464761673, 0.0007970376813502642, 1.5692036732274044]) #ur5-link
robot.impedenceController(thi, desired_pose, controller_gain=50)


