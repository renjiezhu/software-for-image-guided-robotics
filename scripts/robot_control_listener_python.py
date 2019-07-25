#!/usr/bin/env python3
"""
 * Software Interface for Image Guided Robotics
 * 
 * Listerner 
 * 
 * Topics Published:
 * * robot_status_TRANSFORM (geometry_msgs/Transform)
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

import transforms3d.euler as euler

import signal
import sys

sys.path.append("/home/renjie/Documents/igr/src/software_interface/")
# sys.path.append("/home/acrmri/homesoftware_interface/")
from pyrep import PyRep
from vrep_robot_control.ct_robot_control import IK_via_vrep
from vrep_robot_control.arm import CtRobot


class RobotState:
    """
    A class containing the current robot state
    """

    def __init__(self):

        rospy.init_node("robot_control_listener_python", anonymous=True)
        rospy.loginfo("V-REP update node is initialized...")

        # keep track of current robot position, orientation and needle position
        self.pos = [0.0011, -0.6585, 0.2218]
        self.ori = [0.0, 0.0, 0.0]
        self.needle_pos = 0.0

        # to rotate purely in world frame orientation and needle frame position,
        #
        self.cur_pose = np.eye(3)

        # publisher of the current robot position in a format of geometry_msgs::Twist
        self.robot_status_pub = rospy.Publisher(
            "robot_status_TRANSFORM", Transform, queue_size=1
        )
        self.robot_status = Transform()

        # dirty markers
        self.dirty = False
        self.needle_dirty = False

        # pyrep instance
        self.pr = PyRep()
        self.pr.launch(
            "/home/renjie/Documents/igr/src/software_interface/vrep_robot_control/ct_robot_realigned.ttt"
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
        rospy.loginfo("DONE")

    ## signal capture (sigint) ##
    def signal_handler(self, sig, frame):
        """
        safely shutdown vrep when control C is pressed
        """
        rospy.loginfo("Calling exit for pyrep")
        self.shutdown_vrep()
        rospy.signal_shutdown("from signal_handler")

    def needle_retracted(self):
        """
        return if the needle is retracted
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
            rotated = False
            # for the current implementation, rotation is only on one axis, accurate to
            # 1 degree.
            if data.angular.y != 0:  # due to difference of screen frame and world frame y is -x
                theta = np.radians(-data.angular.y)
                c, s = np.cos(theta), np.sin(theta)
                rot_mat = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
                rotated = True
            elif data.angular.x != 0:
                theta = np.radians(data.angular.x)
                c, s = np.cos(theta), np.sin(theta)
                rot_mat = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
                rotated = True
            elif data.angular.z != 0:
                theta = np.radians(data.angular.z)
                c, s = np.cos(theta), np.sin(theta)
                rot_mat = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
                rotated = True
            else:
                rotated = False

            # step two: if rotated, find the world frame euler angles
            if rotated:
                self.cur_pose = rot_mat @ self.cur_pose
                # get euler angles for vrep wrt world frame (rxyz)
                self.ori = euler.mat2euler(self.cur_pose, "rxyz")

            self.dirty = True

            rospy.logdebug(self.pos)
            rospy.logdebug(self.ori)

            self.update_vrep()

            self.publish_robot_status()

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
        """
        rospy.Subscriber("needle_insertion", Float64, self.needle_pos_callback)
        rospy.Subscriber("robot_movement", Twist, self.robot_pos_callback)

        signal.signal(signal.SIGINT, self.signal_handler)
        rospy.spin()

    def update_vrep(self):
        """
        update vrep for ik
        """
        if self.dirty:
            rospy.loginfo("updating robot position")
            IK_via_vrep(self.ct_robot, self.pos, self.ori, self.pr, self.dt)
            self.dirty = False
        elif self.needle_dirty:
            self.needle_dirty = False
            rospy.loginfo("updating needle insertion")
        else:
            pass

    def publish_robot_status(self):
        """
        update robot status into a Transform form, and publish over a topic
        """

        self.robot_status.translation.x = self.pos[0]
        self.robot_status.translation.y = self.pos[1]
        self.robot_status.translation.z = self.pos[2]

        quat = euler.euler2quat(self.ori[0], self.ori[1], self.ori[2])
        self.robot_status.rotation.w = quat[0]
        self.robot_status.rotation.x = quat[1]
        self.robot_status.rotation.y = quat[2]
        self.robot_status.rotation.z = quat[3]

        self.robot_status_pub.publish(self.robot_status)


if __name__ == "__main__":

    robot = RobotState()

    robot.update_state()

