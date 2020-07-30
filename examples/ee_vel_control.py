# ############################################
# An example for doing end-effector velocity control
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
end_eff_vel = np.array([0.0, 0, -0.05, 0, 0, 0])
robot.endEffectorVelocityControl(end_eff_vel, sim_time=2)


