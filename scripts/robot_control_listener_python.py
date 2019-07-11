#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist


def test_callback(data):
    rospy.loginfo("test: x = %d" % data.linear.x)

def vrep_callback(data):
    pos_screen = np.array([data.linear.x, data.linear.y, data.linear.z])
    ori_screen = np.array([data.angular.x, data.angular.y, data.angular.z])
    rospy.logdebug(pos_screen)
    rospy.logdebug(ori_screen)

    pos_robot = np.array([pos_screen[1]*-1, pos_screen[0], pos_screen[2]])
    ori_robot = np.array([ori_screen[1]*-1, ori_screen[0], ori_screen[2]])

    # call vrep set



def listener():
    rospy.init_node('robot_control_listener_python', anonymous=True)
    rospy.Subscriber("robot_movement", Twist, vrep_callback)
    

    rospy.spin()


if __name__ == "__main__":
    listener()