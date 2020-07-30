# ############################################
# An example for doing free fall under gravity
#
# Author : Deepak Raina @ IIT Delhi
# Version : 0.1
# ############################################

import numpy as np
from pybullet_controller import RobotController

robot = RobotController(robot_type='3link')
robot.createWorld(GUI=True)

# Free fall under gravity dynamic simulation
robot.doFreeFall()


