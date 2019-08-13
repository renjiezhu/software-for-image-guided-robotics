#!/usr/bin/env python3
"""
 * Software Interface for Image Guided Robotics
 * 
 * torque_controller
 * 
 * Topics Published:
 * * joint torque (software_interface/Torque)
 *
 * Topics Subscribed:
 * * joint angles (software_interface/JointAngles)
 * 
 * By Renjie Zhu (rezhu@eng.ucsd.edu), Guosong Li ()
 * 
 * August 7th, 2019
 * 
"""

import rospy

from software_interface import Torque
from software_interface import JointAngles


class TorqueController:

    def __init__(self):
        
        rospy.init_node("torque_controller", anonymous=True)
        rospy.loginfo("Torque controller loaded ...")

        



if __name__ == "__main__":
    pass