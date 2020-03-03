import rospy
import numpy as np
import sys, os
sys.path.append(f"/home/{os.environ['USER']}/Documents/igr/src/software_interface/")
sys.path.append(f"/home/{os.environ['USER']}/Documents/igr/src/software_interface/vrep_robot_control")

from sensor_msgs.msg import JointState
from vrep_robot_control.JS_control import *
from vrep_robot_control.DH_dynamics import dh_robot_config_dynamics
from vrep_robot_control.arm import CtRobot
import sympy as sp


# just for testing of the model based controller
import time


num_joints = 4

mxing_matrix = np.array([[ 5.18518519,  0.        ,  0.        ,  0.        ],
                        [ -6.5993266/1.5 ,  5.18518519,  0.        ,  0.        ],
                        [13.72895623/7,  5.18518519,  4.07407407,  0.        ],
                        [-17.89036195/16,  6.48148148,  2.5462963 ,  1.85185185]])


class gravityCompController:

    def __init__(self):
        
        rospy.init_node("torque_controller", anonymous=True)
        rospy.loginfo("Torque controller loaded ...")

        # Read parametes
        param = ['D', 'a', 'alpha', 'theta', 'num_joints', 'jointType', 'Tbase', 'L', 'M']
        config = dict()
        for i in range(len(param)):
            config[param[i]] = np.load(f"/home/{os.environ['USER']}/Documents/igr/src/software_interface/vrep_robot_control/robot_config/inbore/config/{param[i]}.npy")

        # Initialization 
        self.robot = dh_robot_config_dynamics(int(config['num_joints']), config['alpha'], config['theta'], config['D'], config['a'], 
                                                config['jointType'], config['Tbase'], config['L'], config['M'], 'robot_config/inbore')
        self.robot.initKinematicTransforms()

        # publisher
        self.torque_pub = rospy.Publisher("vrep_ros_interface/torque_calculated", JointState, queue_size=1)
        self.torque_msg = JointState()

        # measurement
        self.pos_target = [0,0,0,0]
        self.vel_target = [0,0,0,0]

        # target
        self.pos_measured = [0,0,0,0]
        self.vel_measured = [0,0,0,0]

        # rate
        self._rate = rospy.Rate(500) # 500Hz

        # robot arm offset
        self.xyz = [0, 0, 0]

    
    def measured_callback(self, data):
        self.pos_measured = list(data.position)
        self.vel_measured = list(data.velocity)

    def target_callback(self, data):
        self.pos_target = list(data.position)
        self.vel_target = list(data.velocity)

    def cacl_tau(self):
        # kp = np.diag([0.4, 0.25, 0.00, 0.00000])
        # kv = np.diag([0.15, 0.088, 0.0, 0.000])
        kp = np.diag([0.4, 0.25, 0.00, 0.0000])
        kv = np.diag([0.15, 0.088, 0.0, 0.000])
        tau = cacl_tau_ModelFree(self.robot, kp, kv, self.pos_target, self.pos_measured,
             self.vel_target, self.vel_measured, self.xyz)
        return tau

    
    def publish_func(self):
        while not rospy.is_shutdown():
            tau = self.cacl_tau()
            print(tau)
            self.torque_msg.position = tau.tolist()
            self.torque_pub.publish(self.torque_msg)
            self._rate.sleep()


    def Controller(self):
        rospy.Subscriber("joint_e", JointState, self.measured_callback)
        rospy.Subscriber("target_joint_angles", JointState, self.target_callback)
        self.publish_func()
        rospy.spin()
    



if __name__ == "__main__":
    gravityComp = gravityCompController()
    gravityComp.Controller()