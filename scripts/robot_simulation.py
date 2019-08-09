#!/usr/bin/env python3
"""
 * Software Interface for Image Guided Robotics
 * 
 * Listerner 
 * 
 * Topics Published:
 * * robot_status_TRANSFORM (geometry_msgs/Transform)
 * * robot_status_TWIST (geometry_msgs/Twist) (deprecated)
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
from std_msgs.msg import Float64, Bool
from software_interface.msg import JointAngles

import transforms3d.euler as euler

import signal, sys, os
sys.path.append(f"/home/{os.environ['USER']}/Documents/igr/src/software_interface/")

from pyrep import PyRep
from vrep_robot_control.ct_robot_control import IK_via_vrep
from vrep_robot_control.arm import CtRobot

from enum import Enum

class Mode(Enum):
    SETUP_IK = 1
    SETUP_PP = 2
    TELEOPERATION = 3
    DIRECT_TELEOP = 4


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

        # Two modes for controlling the robot
        self._mode = Mode.SETUP_IK

        # Saved Pose
        self.pos = [0.0011, -0.6585, 0.2218]
        self.ori = [0.0, 0.0, 0.0]
        # keep track of current robot position, orientation and needle position
        self.__pos = [e for e in self.pos]
        self.__ori = [e for e in self.ori]
        self.needle_pos = 0.0

        # keep track of current pose in a 3x3 matrix (SO(3))
        self.cur_pose = np.eye(3)

        # publisher of the current robot position in a format of geometry_msgs::TRANSFORM
        self.robot_status_pub = rospy.Publisher(
            "robot_status_TRANSFORM", Transform, queue_size=1
        )
        self.robot_status = Transform()

        # send pose
        self.confirmed_pose_pub = rospy.Publisher(
            "robot_confirmed_pose", Twist, queue_size=1
        )
        self.confirmed_pose = Twist()

        # publisher of joint angles
        self.joint_angles_pub = rospy.Publisher('joint_angles', JointAngles, queue_size=1)
        self.joint_angles_msg = JointAngles()
        self._rate = rospy.Rate(100) # 100hz

        # dirty markers
        self._dirty = False
        self._needle_dirty = False
        self._rotating = False

        # pyrep instance
        self._pr = PyRep()
        self._pr.launch(
            f"/home/{os.environ['USER']}/Documents/igr/src/software_interface/vrep_robot_control/ct_robot_realigned.ttt",
            headless=False,
        )
        self._dt = 0.01
        self._pr.set_simulation_timestep(self._dt)
        self._pr.start()

        # pyrep robot model instance
        self._ct_robot = CtRobot()

    def switch_mode(self, mode: Mode):
        self._mode = mode

    def shutdown_vrep(self):
        """
        shutdown vrep safely
        """
        self._pr.stop()
        rospy.loginfo("V-REP shutting down.")
        self._pr.shutdown()

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
                self._rotating = True
            elif data.angular.x != 0:
                theta = np.radians(data.angular.x)
                c, s = np.cos(theta), np.sin(theta)
                rot_mat = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
                self._rotating = True
            elif data.angular.z != 0:
                theta = np.radians(data.angular.z)
                c, s = np.cos(theta), np.sin(theta)
                rot_mat = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
                self._rotating = True
            else:
                self._rotating = False

            # step two: if rotated, find the world frame euler angles
            if self._rotating:
                self.cur_pose = rot_mat @ self.cur_pose
                # get euler angles for vrep wrt world frame (rxyz)
                self.ori = list(euler.mat2euler(self.cur_pose, "rxyz"))

            self._dirty = True

            rospy.loginfo(self.pos)
            rospy.loginfo(self.ori)

            self.update_vrep()

            self.send_robot_status()

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
            self._needle_dirty = True
            rospy.loginfo(self.needle_pos)

            self.update_vrep()

    def confirmation_callback(self, data):
        """
        send simulation confirmation
        """
        if self._mode is Mode.SETUP_IK:
            rospy.loginfo("confirmed; mode: setup_ik")
            # send current pose to ik node (or opti+pp)
            # save current pose as saved pose
            # set mode to teleoperation? 
            pass
        elif self._mode is Mode.SETUP_PP:
            rospy.loginfo("confirmed; mode: setup_pp")
            # send current joint configuration to pp node
            # set mode to teleoperation?
            pass
        elif self._mode is Mode.TELEOPERATION:
            rospy.loginfo("confirmed; mode: setup_to")
            # send current pose to ik node
            # save current pose as saved pose
            pass
        elif self._mode is Mode.DIRECT_TELEOP:
            rospy.loginfo("confirmed; mode: setup_dt")
            # fail this confirmation
            pass
        else:
            # do nothing raise error i guess
            pass

    def reset_callback(self, data):
        """
        reset pos and ori to original
        """
        if self._mode is Mode.DIRECT_TELEOP:
            rospy.loginfo("Cannot reset. You are in direct teleoperation mode. ")
        self.pos = [e for e in self.__pos]
        self.ori = [e for e in self.__ori]
        self._dirty = True
        self.update_vrep()
        self.send_robot_status()

    def update_vrep(self):
        """
        update robot status for ik calculation via v-rep

        Call function developped by Guosong; interact with v-rep
        """
        if self._dirty:
            rospy.loginfo("updating robot position")
            IK_via_vrep(self._ct_robot, self.pos, self.ori, self._pr, self._dt)
            self._dirty = False
        elif self._needle_dirty:
            rospy.loginfo("updating needle insertion")
            # TODO: needle motor driving
            self._needle_dirty = False
        else:
            pass

    def send_robot_status(self):
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

            joint_angles_vrep = self._ct_robot.get_joint_positions()

            self.joint_angles_msg.joint0.data = joint_angles_vrep[0]
            self.joint_angles_msg.joint1.data = joint_angles_vrep[1]
            self.joint_angles_msg.joint2.data = joint_angles_vrep[2]
            self.joint_angles_msg.joint3.data = joint_angles_vrep[3]
            self.joint_angles_msg.joint4.data = joint_angles_vrep[4]
            self.joint_angles_msg.joint5.data = joint_angles_vrep[5]
            self.joint_angles_msg.joint6.data = joint_angles_vrep[6]
            
            self.joint_angles_pub.publish(self.joint_angles_msg)
            self._rate.sleep()

    def update_state(self):
        """
        subscribe to keyboard published to update the robot state
        
        Core method to be called in __main__; access point to
        callbacks. Held by rospy.spin() until ctrl+C.
        """

        rospy.Subscriber("needle_insertion", Float64, self.needle_pos_callback)
        rospy.Subscriber("robot_movement", Twist, self.robot_pos_callback)
        rospy.Subscriber("confirmation", Bool, self.confirmation_callback)
        rospy.Subscriber("reset_confirmation", Bool, self.reset_callback)
        
        signal.signal(signal.SIGINT, self.signal_handler)

        # if self._mode is Mode.DIRECT_TELEOP:
        #     self.send_joint_angles()

        rospy.spin()


if __name__ == "__main__":

    robot = RobotState()
    robot.update_state()



