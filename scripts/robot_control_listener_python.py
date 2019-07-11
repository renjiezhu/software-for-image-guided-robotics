#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class RobotState:
    """
    A class containing the current robot state
    """


    def __init__(self):

        self.pos = np.zeros(3)
        self.ori = np.zeros(3)

        self.needle_pos = 0.0

        self.dirty = False
        self.needle_dirty = False

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

            self.pos[0] -= data.linear.y
            self.pos[1] += data.linear.x
            self.pos[2] += data.linear.z

            self.ori[0] -= data.angular.y
            self.ori[1] += data.angular.x
            self.ori[2] += data.angular.z

            self.dirty = True

            rospy.loginfo(self.pos)
            rospy.loginfo(self.ori)

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


    def update_state(self):
        """
        subscribe to keyboard published to update the robot state
        """
        rospy.Subscriber("needle_insertion", Float64, self.needle_pos_callback)
        rospy.Subscriber("robot_movement", Twist, self.robot_pos_callback)


    def update_vrep(self):
        """
        update vrep for ik
        """
        if dirty:
            pass
        elif needle_dirty:
            pass
        else:
            pass



if __name__ == "__main__":

    rospy.init_node("robot_control_listener_python", anonymous=True)

    robot = RobotState()
    robot.update_state()

    rospy.spin()