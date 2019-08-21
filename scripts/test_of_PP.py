#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import numpy as np
import time

# set up pyrep

def callback(input_msg):
    print(input_msg.position)
    rospy.loginfo("Received.")
    pass

if __name__ == "__main__":
    rospy.init_node("test_of_PP", anonymous=True)
    rospy.Subscriber("/vrep_ros_interface/target_joint_positions", JointState, callback)
    
    rospy.spin()