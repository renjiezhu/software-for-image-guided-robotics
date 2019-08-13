#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point

def listener(data):
    rospy.loginfo(f"x: {data.x}, y: {data.y}, z: {data.z}")

if __name__ == "__main__":
    rospy.init_node("test_listener", anonymous=True)
    rospy.Subscriber("vrep_ros_interface/PP_VREP", Point, listener)
    rospy.spin()