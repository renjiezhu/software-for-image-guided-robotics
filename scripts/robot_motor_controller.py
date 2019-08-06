#!/usr/bin/env python3
import rospy
from software_interface.msg import JointAngles

import numpy as np

# import matplotlib.pyplot as plt
# import time

import sys

# sys.path.append("..")
# sys.path.append("../../")
# sys.path.append("/home/guosong/Documents/DH_Kinematics/robot_controls_library/")
import armControl
from forwardKinematics import robot_config
from utils.motor_setup import maxonGearSmall

import signal


class RobotMotors:
    def __init__(self):

        rospy.init_node("motor_controller", anonymous=True)
        rospy.loginfo("Motor controller started ...")

        self.socket_ip = "192.168.0.115"
        self.socket_port = 1122

        self.motors = maxonGearSmall()
        self.motors.tcp_init(self.socket_ip, self.socket_port)

        # arm motors
        rospy.loginfo("Arming motors now...")
        self.motors.arm_motors()
        for i in range(8):
            self.motors.zero_motors_radians(
                i, self.motors.get_motors_position_radians()[i]
            )

        self.myArm = armControl.remoteRobotArm()

    def signal_handler(self, sig, frame):

        rospy.loginfo("Calling tcp close ...")
        self.motors.tcp_close()
        rospy.signal_shutdown("from signal_handler")

    def joint_angles_callback(self, input_data):

        setpoint_arm = np.zeros(7)

        setpoint_arm[0] = input_data.joint0.data
        setpoint_arm[1] = input_data.joint1.data
        setpoint_arm[2] = input_data.joint2.data
        setpoint_arm[3] = input_data.joint3.data
        setpoint_arm[4] = input_data.joint4.data
        setpoint_arm[5] = input_data.joint5.data
        setpoint_arm[6] = input_data.joint6.data

        self.myArm.commandJoints(self.motors, setpoint_arm)

    def update_joint_angles(self):

        rospy.Subscriber("joint_angles", JointAngles, self.joint_angles_callback)

        signal.signal(signal.SIGINT, self.signal_handler)

        rospy.spin()


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node("listener", anonymous=True)

    rospy.Subscriber("joint_angles", JointAngles, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    listener()

