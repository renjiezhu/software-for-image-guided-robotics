#!/usr/bin/env python3
import sys,os
import numpy as np
import rospy
import time
from sensor_msgs.msg import JointState

 
"""
Walk through a set of points and repeat
"""

def publisher():
    pub = rospy.Publisher("joint_angles_streaming", JointState, queue_size=1)
    rospy.init_node("simply_repeability_test", anonymous=True)
    rate = rospy.Rate(20)

    num_points = 100
    joint_configurations = np.zeros((num_points*2,8))
    joint_velocities = np.zeros((num_points*2,8))

    #set joint limits for a single axis
    joint_upper_limits = np.array([0.1,0.13,0.34,np.pi,np.pi/3,np.pi/3,np.pi/3,np.pi/3])
    joint_lower_limits = np.array([0,0,0,-1*np.pi,-1*np.pi/3,-1*np.pi/3,-1*np.pi/3,-1*np.pi/3])
    for i in range(8):
        joint_configurations[:num_points,i] = np.linspace(joint_lower_limits[i], joint_upper_limits[i], num_points)
        joint_velocities[:num_points, i] = 0.01
        joint_configurations[num_points:,i] = np.flip(np.linspace(joint_lower_limits[i], joint_upper_limits[i], num_points))
        joint_velocities[num_points:, i] = -0.01


    state = JointState()
    state.position = [0]*8
    state.velocity = [0]*8
    state.effort = [0]*8

    # Test for one axis at a time
    axis = 0

    
    while not rospy.is_shutdown():
        for i in range(2*num_points):
            state.position[axis] = joint_configurations[i, axis]
            state.velocity[axis] = joint_velocities[i, axis]
            pub.publish(state)
            rate.sleep()
            # time.sleep(1.5)

        # EE_position.append(endEffectorPositionCurrent)



if __name__=="__main__":
    publisher()
