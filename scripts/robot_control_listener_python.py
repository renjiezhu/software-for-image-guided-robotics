#!/usr/bin/env python3
"""
 * Software Interface for Image Guided Robotics
 * 
 * Listerner 
 * 
 * Topics Published:
 * * robot_status_TWIST
 *
 * Topics Subscribed:
 * * robot_movement
 * * needle_insertion
 * 
 * By Renjie Zhu (rezhu@eng.ucsd.edu)
 * 
 * July 9th, 2019
 * 
 """
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

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

        # publisher of the current robot position in a format of geometry_msgs::Twist
        self.robot_status_pub = rospy.Publisher("robot_status_TWIST", Twist, queue_size=2)
        self.robot_status = Twist()

        # dirty markers
        self.dirty = False
        self.needle_dirty = False

        # pyrep instance
        self.pr = PyRep()
        self.pr.launch("/home/renjie/Documents/igr/src/software_interface/vrep_robot_control/ct_robot_realigned.ttt")
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

            self.pos[0] -= data.linear.y / 1000
            self.pos[1] += data.linear.x / 1000
            self.pos[2] += data.linear.z / 1000

            self.ori[0] -= data.angular.y * np.pi / 180
            self.ori[1] += data.angular.x * np.pi / 180
            self.ori[2] += data.angular.z * np.pi / 180

            self.dirty = True

            rospy.loginfo(self.pos)
            rospy.loginfo(self.ori)
            
            self.update_vrep()

            self.publish_robot_status()

        else:
            rospy.logwarn("Needle (pos=%.1f) not retracted, cannot move robot." % self.needle_pos)


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
            IK_via_vrep(self.ct_robot, self.pos, self.ori, self.pr, self.dt)
            self.dirty=False
            rospy.loginfo("updating robot position")
        elif self.needle_dirty:
            self.needle_dirty=False
            rospy.loginfo("updating needle insertion")
        else:
            pass
        

    def publish_robot_status(self):
        """
        update robot status into a Twist form, and publish over a topic
        """
        
        self.robot_status.linear.x = self.pos[0]
        self.robot_status.linear.y = self.pos[1]
        self.robot_status.linear.z = self.pos[2]

        self.robot_status.angular.x = self.ori[0]
        self.robot_status.angular.y = self.ori[1]
        self.robot_status.angular.z = self.ori[2]

        self.robot_status_pub.publish(self.robot_status)        



if __name__ == "__main__":

    robot = RobotState()

    robot.update_state()

