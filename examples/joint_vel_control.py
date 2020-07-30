# ############################################
# An example for doing joint velocity control
#
# Author : Deepak Raina @ IIT Delhi
# Version : 0.1
# ############################################

import numpy as np
from pybullet_controller import RobotController

robot = RobotController(robot_type='ur5')
robot.createWorld(GUI=True)

# Joint velocity control
# Input: numpy array of joint angles
joint_angles = np.array([0, -1.0, 1.0, -1.57, -1.57, -1.57])
robot.setJointPosition(joint_angles)
joint_velocities = np.array([0.5, 0, 0, 0, 0, 0])
robot.JointVelocityControl(joint_velocities, sim_time=5)
