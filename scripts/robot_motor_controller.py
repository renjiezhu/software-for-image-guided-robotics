#!/usr/bin/env python
# # import rospy
# # import numpy as np
# # from geometry_msgs.msg import Twist

# import numpy as np
# import matplotlib.pyplot as plt
# import time

# import sys
# # sys.path.append("..")
# # sys.path.append("../../")
# # sys.path.append("/home/guosong/Documents/DH_Kinematics/robot_controls_library/")
# from armControl import *
# from forwardKinematics import robot_config
# from utils.motor_setup import maxonGearSmall

# import signal

# class RobotMotors:

#     def __init__(self):
#         self.socket_ip = '192.168.0.115'
#         self.socket_port = 1122


#         self.motors = maxonGearSmall()
#         self.motors.tcp_init(self.socket_ip, self.socket_port)

#         # arm motors
#         print("Arming motors now...")
#         self.motors.arm_motors()
#         for i in range(8):
#             motors.zero_motors_radians(i, motors.get_motors_position_radians()[i])

#         from importlib import reload
#         reload(armControl)

#         myArm = armControl.remoteRobotArm()

#         #for i in range(5):
#         while True:
#             setpoint_arm = np.array([-0.000,0.000,0.6,-0.1,-0.2,0.,0.00])
#             myArm.commandJoints(motors, setpoint_arm)
#             #time.sleep(2)
#                 #setpoint_arm = np.array([0.0,0.00,-0.0,0.0,0.0,-0.0,0.00])
#                 #myArm.commandJoints(motors, setpoint_arm)
#                 #time.sleep(2)
#                 #myArm.commandJoints(motors, setpoint_arm)
#             time.sleep(0.5)


    # motors.tcp_close()

import rospy
from software_interface.msg import JointAngles

def callback(data):
    rospy.loginfo("I heard %s", data.joint0.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("joint_angles", JointAngles, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    listener()

