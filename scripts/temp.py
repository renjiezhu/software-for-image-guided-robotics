#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState


def callback(input_msg):
    
    for i in range(len(input_msg.position)):
        rospy.loginfo(input_msg.position[i])

    rospy.loginfo("Received.")

if __name__ == "__main__":
    rospy.init_node("test_listener", anonymous=True)
    rospy.Subscriber("torque_tau_controller", JointState, callback)
    rospy.spin()