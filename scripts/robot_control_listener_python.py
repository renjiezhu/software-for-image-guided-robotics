#!/usr/bin/env python3
"""
 * Software Interface for Image Guided Robotics
 * 
 * Listerner 
 * 
 * Topics Published:
 * * robot_status_TRANSFORM (geometry_msgs/Transform)
 * * robot_status_TWIST (geometry_msgs/Twist)
 * * joint_angles
 *
 * Topics Subscribed:
 * * robot_movement (geometry_msgs/Twist)
 * * needle_insertion (std_msgs/Float64)
 * 
 * By Renjie Zhu (rezhu@eng.ucsd.edu)
 * 
 * July 9th, 2019
 * 
"""
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform
from std_msgs.msg import Float64
from software_interface.msg import JointAngles

import transforms3d.euler as euler

import signal
import sys

# sys.path.append(".")
# sys.path.append("./src/software_interface")
# import os
# print(f"current working directory: {os.getcwd()}")
sys.path.append("/home/renjie/Documents/igr/src/software_interface/")
# sys.path.append("/home/guosong/Documents/igr/src/software_interface/")
from pyrep import PyRep
from vrep_robot_control.ct_robot_control import IK_via_vrep
from vrep_robot_control.arm import CtRobot


class RobotState:
    """
    A class maintaining the robot state (pose), 
    getting updates from ros topics about movement
    send robot state to vrep for ik calculation
    send transform to IGTL link * (subject to change)
    """

    def __init__(self):

        rospy.init_node("robot_control_listener_python", anonymous=True)
        rospy.loginfo("V-REP update node is initialized...")

        # keep track of current robot position, orientation and needle position
        self.pos = [0.0011, -0.6585, 0.2218]
        self.ori = [0.0, 0.0, 0.0]
        self.needle_pos = 0.0

        # keep track of current pose in a 3x3 matrix (SO(3))
        self.cur_pose = np.eye(3)

        # publisher of the current robot position in a format of geometry_msgs::Twist
        self.robot_status_pub = rospy.Publisher(
            "robot_status_TRANSFORM", Transform, queue_size=1
            # "robot_status_TWIST", Twist, queue_size=1,
        )
        self.robot_status = Transform()
        # self.robot_status = Twist()

        # publisher of joint angles
        self.joint_angles_pub = rospy.Publisher('joint_angles', JointAngles, queue_size=1)
        self.joint_angles_msg = JointAngles()
        self.rate = rospy.Rate(100) # 100hz

        # dirty markers
        self.dirty = False
        self.needle_dirty = False
        self.rotating = False

        # pyrep instance
        self.pr = PyRep()
        self.pr.launch(
            "/home/renjie/Documents/igr/src/software_interface/vrep_robot_control/ct_robot_realigned.ttt",
            headless=False,
        )
        self.dt = 0.01
        self.pr.set_simulation_timestep(self.dt)
        self.pr.start()

        # pyrep robot model instance
        self.ct_robot = CtRobot()

    def shutdown_vrep(self):
        """
        shutdown vrep safely
        """
        self.pr.stop()
        rospy.loginfo("V-REP shutting down.")
        self.pr.shutdown()

    def signal_handler(self, sig, frame):
        """
        safely shutdown vrep when control C is pressed
        """
        rospy.loginfo("Calling exit for pyrep")
        self.shutdown_vrep()
        rospy.signal_shutdown("from signal_handler")

    def needle_retracted(self):
        """
        return True if the needle is retracted
        """
        return self.needle_pos < 1e-5 and self.needle_pos >= 0

    def robot_pos_callback(self, data):
        """
        update robot pos and ori with given keyboard instructions
        """
        if self.needle_retracted():

            # modify robot position given input
            self.pos[0] -= data.linear.y / 1000
            self.pos[1] += data.linear.x / 1000
            self.pos[2] += data.linear.z / 1000

            # transform world frame rotation to base frame rotation
            # step one: find corresponding rotation matrix
            # for the current implementation, rotation is only on one axis, accurate to
            # 1 degree.

            # due to difference of screen frame and world frame y is -x
            if data.angular.y != 0:
                theta = np.radians(-data.angular.y)
                c, s = np.cos(theta), np.sin(theta)
                rot_mat = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
                self.rotating = True
            elif data.angular.x != 0:
                theta = np.radians(data.angular.x)
                c, s = np.cos(theta), np.sin(theta)
                rot_mat = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
                self.rotating = True
            elif data.angular.z != 0:
                theta = np.radians(data.angular.z)
                c, s = np.cos(theta), np.sin(theta)
                rot_mat = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
                self.rotating = True
            else:
                self.rotating = False

            # step two: if rotated, find the world frame euler angles
            if self.rotating:
                self.cur_pose = rot_mat @ self.cur_pose
                # get euler angles for vrep wrt world frame (rxyz)
                self.ori = euler.mat2euler(self.cur_pose, "rxyz")

            self.dirty = True

            rospy.logdebug(self.pos)
            rospy.logdebug(self.ori)

            self.update_vrep()

            self.publish_robot_status()

        # if needle is not retracted, WARN (subject to change as safety precautions change)
        else:
            rospy.logwarn(
                "Needle (pos=%.1f) not retracted, cannot move robot." % self.needle_pos
            )

    def needle_pos_callback(self, data):
        """
        update needle position with given keyboard instructions
        """
        if self.needle_retracted() and data.data < 0:
            rospy.logwarn("Needle fully retracted.")
        else:
            self.needle_pos += data.data
            self.needle_dirty = True
            rospy.loginfo(self.needle_pos)

            self.update_vrep()

    def update_state(self):
        """
        subscribe to keyboard published to update the robot state
        
        Core method to be called in __main__; access point to both
        callbacks. Held by rospy.spin() until ctrl+C.
        """
        rospy.Subscriber("needle_insertion", Float64, self.needle_pos_callback)
        rospy.Subscriber("robot_movement", Twist, self.robot_pos_callback)

        signal.signal(signal.SIGINT, self.signal_handler)
        self.send_joint_angles()
        rospy.spin()

    def update_vrep(self):
        """
        update robot status for ik calculation via v-rep

        Call function developped by Guosong; interact with v-rep
        """
        if self.dirty:
            rospy.loginfo("updating robot position")
            IK_via_vrep(self.ct_robot, self.pos, self.ori, self.pr, self.dt)
            self.dirty = False
        elif self.needle_dirty:
            rospy.loginfo("updating needle insertion")
            # TODO: needle motor driving
            self.needle_dirty = False
        else:
            pass

    def publish_robot_status(self):
        """
        update robot status into a Transform form, and publish over a topic
        """

        # unit base in 3d slicer 'mm' , convert by multipling 1000
        self.robot_status.translation.x = self.pos[0] * 1000
        self.robot_status.translation.y = self.pos[1] * 1000
        self.robot_status.translation.z = self.pos[2] * 1000

        # find the quaternion for the current orientation 
        # parameter 'axes' corrects for frame differences
        quat = euler.euler2quat(self.ori[0], self.ori[1], self.ori[2], axes="sxyz")
        self.robot_status.rotation.w = quat[0]
        self.robot_status.rotation.x = quat[1]
        self.robot_status.rotation.y = quat[2]
        self.robot_status.rotation.z = quat[3]

        # for sending twist

        # self.robot_status.linear.x = self.pos[0] * 1000
        # self.robot_status.linear.y = self.pos[1] * 1000
        # self.robot_status.linear.z = self.pos[2] * 1000

        # self.robot_status.angular.x = self.ori[0]
        # self.robot_status.angular.y = self.ori[1]
        # self.robot_status.angular.z = self.ori[2]
        
        
        self.robot_status_pub.publish(self.robot_status)

    def send_joint_angles(self):

        while not rospy.is_shutdown():

            self.joint_angles_msg.joint0.data = joint_angles_vrep[0]
            self.joint_angles_msg.joint1.data = joint_angles_vrep[1]
            self.joint_angles_msg.joint2.data = joint_angles_vrep[2]
            self.joint_angles_msg.joint3.data = joint_angles_vrep[3]
            self.joint_angles_msg.joint4.data = joint_angles_vrep[4]
            self.joint_angles_msg.joint5.data = joint_angles_vrep[5]
            self.joint_angles_msg.joint6.data = joint_angles_vrep[6]
            

            self.joint_angles_pub.publish(self.joint_angles_msg)
            self.rate.sleep()

if __name__ == "__main__":

    robot = RobotState()
    robot.update_state()



