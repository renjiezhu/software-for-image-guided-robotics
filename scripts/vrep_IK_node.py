#!/usr/bin/env python3
"""
 * Software Interface for Image Guided Robotics
 * 
 * V-REP based Inverse Kinematics ROS Node
 * 
 * Topics Published:
 * * joint setpoint (Message: JointState)
 *
 * Topics Subscribed:
 * * end-effector pose (Message: Pose)
 * 
 * By Guosong Li (g4li@ucsd.edu)
 * 
 * Feb 11th, 2020
 * 
"""
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

import signal, sys, os
sys.path.append(f"/home/{os.environ['USER']}/Documents/igr/src/software_interface/")

from pyrep import PyRep
import numpy as np
from vrep_robot_control.arm import CtRobot
from pyrep.const import JointMode
import transforms3d.euler as euler
from vrep_robot_control.ct_robot_control import IK_via_vrep



class ControllerTester:
    """
    A class to test performance of torque controller based on VREP simulation
    """

    def __init__(self):
        rospy.init_node("vrep_IK", anonymous=True)
        rospy.loginfo("Vrep-based Inverse Kinematics ROS Node is intializing...")
        rospy.Rate(500)

        # Set up pulbisher and subscriber
        self.pub = rospy.Publisher("setpoint_joint_config", JointState, queue_size=1)
        self.sub = rospy.Subscriber("setpoint_EE_pose", Pose, callback=self.callback)
        self.ee_pos = [0,0,0]
        self.ee_quat = [0,0,0,0]
        self.joint_config = [0,0,0,0,0,0,0,0]
        
        # Launch VREP using pyrep
        self.pr = PyRep()
        self.pr.launch(
            f"/home/{os.environ['USER']}/Documents/igr/src/software_interface/vrep_robot_control/ctRobot_ROS_node.ttt",
            headless=True)
        self.robot = CtRobot()

        # Set up simulation parameter
        self.dt = 0.002
        self.pr.start()
        self.pr.set_simulation_timestep(self.dt)

        # Set up dynamics properties of arm
        for j in range(1, self.robot._num_joints + 1):
            self.robot.arms[j].set_dynamic(False)
        self.robot.arms[0].set_dynamic(False)

        # Set up joint properties
        for i in range(self.inbore._num_joints):
            self.robot.joints[i].set_joint_mode(JointMode.IK)
            self.robot.joints[i].set_joint_position(0)

        # Set up position
        self.pr.step()

    
    def signal_handler(self, sig, frame):
        """
        safely shutdown vrep when control C is pressed
        """
        rospy.loginfo("Calling exit for pyrep")
        self.shutdown_vrep()
        rospy.signal_shutdown("from signal_handler")

    def shutdown_vrep(self):
        """
        shutdown vrep safely
        """
        rospy.loginfo("Stopping pyrep.")
        self.pr.stop()
        rospy.loginfo("V-REP shutting down.")
        self.pr.shutdown()

    def publish(self):
        """
        publish joint setpoint all the time
        """
        while not rospy.is_shutdown():
            target = JointState()
            target.position = self.joint_config
            signal.signal(signal.SIGINT, self.signal_handler)
            self.pub.publish(target)

    def callback(self, data):
        """
        update joint configurations
        """
        self.ee_pos = data.position
        self.ee_quat = data.orientation
        # Update target pose
        self.robot._ik_target.set_position(self.ee_pos)
        self.robot._ik_target.set_quaternion(self.ee_quat)
        # Update joint angles according to IK solver of VREP
        self.joint_config = self.robot.get_joint_positions()
        self.pr.step()



if __name__ == "__main__":

    """
    NOTE:: Things to be resolved: have multiple pyrep activated to control different scene in VREP
    """






    ct = ControllerTester()
    try:
        ct.publish()
    except:
        pass