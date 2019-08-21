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
    

    pose = Twist()
    pub = rospy.Publisher("/vrep_ros_interface/final_pose", Twist, queue_size=1)
    time.sleep(3)
    pose.linear.x, pose.linear.y, pose.linear.z = [-0.4849, -1.5533, 1.3075]
    pose.angular.x, pose.angular.y, pose.angular.z = [90, 0, 0]
    pub.publish(pose)
    print('final pose sent---------------------------------')

    rospy.spin()