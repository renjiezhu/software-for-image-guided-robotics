#!/usr/bin/env python3
import rospy
from software_interface.msg import Torque

def listener(data):
    rospy.loginfo(f"{data.joint0.data}")

if __name__ == "__main__":
    rospy.init_node("test_listener", anonymous=True)
    rospy.Subscriber("torque", Torque, listener)
    rospy.spin()