#!/usr/bin/env python3
"""
 * Software Interface for Image Guided Robotics
 * 
 * 
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
from std_msgs.msg import Float64

def publisher():
    pub = rospy.Publisher("sine", Float64, queue_size=1)
    rospy.init_node("sine_test", anonymous=True)
    rate = rospy.Rate(1)
    t = 0.0
    dt = 0.1
    while not rospy.is_shutdown():
        sine = Float64()
        sine.data = np.sin(t)
        t += dt
        pub.publish(sine)
        rate.sleep()


if __name__ == "__main__":
    try:
        publisher()
    except:
        pass
