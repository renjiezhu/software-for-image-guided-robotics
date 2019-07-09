#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist


def test_callback(data):
    rospy.loginfo("test: x = %d" % data.linear.x)

def vrep_callback(data):
    pos = np.array([data.linear.x, data.linear.y, data.linear.z])
    ori = np.array([data.angular.x, data.angular.y, data.angular.z])
    rospy.loginfo(pos)
    rospy.loginfo(ori)


def listener():
    rospy.init_node('robot_control_listener_python', anonymous=True)
    rospy.Subscriber("robot_movement", Twist, vrep_callback)
    

    rospy.spin()


if __name__ == "__main__":
    listener()