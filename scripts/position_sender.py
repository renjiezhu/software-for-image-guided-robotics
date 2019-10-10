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
import rospy, sys
import numpy as np
# from std_msgs.msg import Float64
# from sensor_msgs.msg import JointState
from std_msgs.msg import Time

def publisher():
    pub = rospy.Publisher("latency_test_out", Time, queue_size=1)
    # pub = rospy.Publisher("sine", JointState, queue_size=1)
    # pub = rospy.Publisher("sine", Float64, queue_size=1)
    rospy.init_node("sine_test", anonymous=True)
    rate = rospy.Rate(1000)
    t = 0.0
    dt = 0.005

    while not rospy.is_shutdown():
        x = rospy.Time.now()
        # sine = JointState()
        # sine = Float64()
        sine = Time()
        # sine.position = [60*np.sin(t) for _ in range(8)]
        # print(sine.position[0])
        # sine.data = (np.sin(t) + 1.0) * 0.5
        # t += dt
        sine.data = x
        pub.publish(sine)
        rate.sleep()



if __name__ == "__main__":
    try:
        publisher()
    except NameError:
        print("something wrong ", sys.exc_info()[0])
