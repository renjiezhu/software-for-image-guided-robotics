#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import numpy as np
import time


if __name__ == "__main__":
    rospy.init_node("final_pose_sender", anonymous=True)
    pose = Twist()
    pub = rospy.Publisher("/vrep_ros_interface/final_pose", Twist, queue_size=1)
    time.sleep(8)
    pose.linear = [-0.4849, -1.2533, 1.3075]
    pose.angular = [90, 0, 0]
    pub.publish(pose)
    print('final pose sent---------------------------------')