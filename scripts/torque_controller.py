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

import sys, os
sys.path.append(f"/home/{os.environ['USER']}/Documents/igr/src/software_interface/")

from software_interface.msg import Torque
from software_interface.msg import JointAngles
from vrep_robot_control.JS_control import *
from vrep_robot_control.DH_dynamics import dh_robot_config
from vrep_robot_control.arm import CtRobot
import sympy as sp



class TorqueController:

    def __init__(self):
        
        rospy.init_node("torque_controller", anonymous=True)
        rospy.loginfo("Torque controller loaded ...")

        # Read parametes
        param = ['D', 'a', 'alpha', 'theta', 'num_joints', 'jointType', 'Tbase', 'L', 'M']
        config = dict()
        for i in range(len(param)):
            config[param[i]] = np.load(f"/home/{os.environ['USER']}/Documents/igr/src/software_interface/vrep_robot_control/robot_config/config_ct_7DOF/{param[i]}.npy")

        # Initialization 
        self.robot = dh_robot_config(int(config['num_joints']), config['alpha'], config['theta'], config['D'], config['a'], 
                                                config['jointType'], config['Tbase'], config['L'], config['M'])
        # self.robot.initKinematicTransforms()

        # publisher
        self.torque_pub = rospy.Publisher("torque_tau_controller", Torque, queue_size=1)
        self.torque_msg = Torque()

        # measured positions
        self.pos_target = None
        # target positions
        self.pos_measured = None

        # rate
        self._rate = rospy.Rate(500) # 500Hz

    
    def measured_callback(self, data):
        dpos = [data.joint0.data, data.joint1.data, data.joint2.data, data.joint3.data, data.joint4.data, data.joint5.data,
                                data.joint6.data, data.joint7.data]
        self.pos_target = dpos


    def target_callback(self, data):
        mpos = [data.joint0.data, data.joint1.data, data.joint2.data, data.joint3.data, data.joint4.data, data.joint5.data,
                                data.joint6.data, data.joint7.data]
        self.pos_measured = mpos

    def cacl_tau(self):
        return [0,0,0,0,0,0,0,0]

    
    def publish_func(self):
        while not rospy.is_shutdown():
            tau = self.cacl_tau()
            self.torque_msg.joint0.data = tau[0]
            self.torque_msg.joint1.data = tau[1]
            self.torque_msg.joint2.data = tau[2]
            self.torque_msg.joint3.data = tau[3]
            self.torque_msg.joint4.data = tau[4]
            self.torque_msg.joint5.data = tau[5]
            self.torque_msg.joint6.data = tau[6]
            self.torque_msg.joint7.data = tau[7]

            self.torque_pub.publish(self.torque_msg)
            self._rate.sleep()


    def Controller(self):
        rospy.Subscriber("measured_joint_angles", JointAngles, self.measured_callback)
        rospy.Subscriber("target_joint_angles", JointAngles, self.target_callback)
        
        self.publish_func()
        rospy.spin()
    



if __name__ == "__main__":
    torque_controller = TorqueController()
    torque_controller.Controller()
