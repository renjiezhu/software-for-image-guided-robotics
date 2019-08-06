#!/usr/bin/env python
# import rospy
# import numpy as np
# from geometry_msgs.msg import Twist

import numpy as np
import matplotlib.pyplot as plt
import time

import sys
# sys.path.append("..")
# sys.path.append("../../")
sys.path.append("/home/guosong/Documents/DH_Kinematics/robot_controls_library/")
from armControl import *
from forwardKinematics import robot_config
from utils.motor_setup import maxonGearSmall

import signal

socket_ip = '192.168.0.115'
socket_port = 1122

#trajPlanner = trajectoryGenerator()

motors = maxonGearSmall()
motors.tcp_init(socket_ip, socket_port)
# constantly get from vrep and set angles to fpga

# arm motors
print("Arming motors now...")
motors.arm_motors()
for i in range(8):
    motors.zero_motors_radians(i, motors.get_motors_position_radians()[i])

from importlib import reload
reload(armControl)

myArm = armControl.remoteRobotArm()

#for i in range(5):
while True:
    setpoint_arm = np.array([-0.000,0.000,0.6,-0.1,-0.2,0.,0.00])
    myArm.commandJoints(motors, setpoint_arm)
    #time.sleep(2)
        #setpoint_arm = np.array([0.0,0.00,-0.0,0.0,0.0,-0.0,0.00])
        #myArm.commandJoints(motors, setpoint_arm)
        #time.sleep(2)
        #myArm.commandJoints(motors, setpoint_arm)
    time.sleep(0.5)


motors.tcp_close()

