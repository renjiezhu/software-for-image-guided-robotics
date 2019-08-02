import numpy as np
import os
import sys
from pyrep import PyRep
from arm import CtRobot
from my_rigid_kinematics import dh_robot_config

# disired trajectory of joint position
theta = np.linspace(0, 2*np.pi, 1000)

