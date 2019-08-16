#!/usr/bin/env python3
"""
 * Software Interface for Image Guided Robotics
 * 
 * V-REP based controller
 * 
 * Topics Published:
 * * joint torque (software_interface/Torque)
 *
 * Topics Subscribed:
 * * joint angles (software_interface/JointAngles)
 * 
 * By Guosong Li (g4li@ucsd.edu), Renjie Zhu (rezhu@eng.ucsd.edu)
 * 
 * August 14th, 2019
 * 
"""
import rospy
from sensor_msgs.msg import JointState

import signal, sys, os
sys.path.append(f"/home/{os.environ['USER']}/Documents/igr/src/software_interface/")

from pyrep import PyRep
import numpy as np
from vrep_robot_control.arm import CtRobot
from pyrep.const import JointMode
import sympy as sp



class ControllerTester:
    """
    A class to test performance of torque controller based on VREP simulation
    """

    def __init__(self):
        rospy.init_node("controller_tester", anonymous=True)
        rospy.loginfo("controller tester node is initialized...")

        self.target_pub = rospy.Publisher("target_joint_angles", JointState, queue_size=1)
        
        # Launch VREP using pyrep
        self.pr = PyRep()
        self.pr.launch(
            f"/home/{os.environ['USER']}/Documents/igr/src/software_interface/vrep_robot_control/in-bore.ttt",
            headless=False)
        self.inbore = CtRobot(name='inbore_arm', num_joints=4, joint_type=['r','r','r','p'])

        # Set up simulation parameter
        self.dt = 0.002
        self.pr.start()
        self.pr.set_simulation_timestep(self.dt)

        # Set up dynamics properties of arm
        for j in range(1, self.inbore._num_joints + 1):
            self.inbore.arms[j].set_dynamic(False)
        self.inbore.arms[0].set_dynamic(False)

        # Set up joint properties
        for i in range(self.inbore._num_joints):
            self.inbore.joints[i].set_joint_mode(JointMode.PASSIVE)
            self.inbore.joints[i].set_control_loop_enabled(False)
            self.inbore.joints[i].set_motor_locked_at_zero_velocity(True)
            self.inbore.joints[i].set_joint_position(0)
            self.inbore.joints[i].set_joint_target_velocity(0)

        self.inbore.joints[0].set_joint_mode(JointMode.FORCE)
        self.inbore.joints[0].set_control_loop_enabled(False)
        self.inbore.joints[0].set_motor_locked_at_zero_velocity(True)
        self.inbore.arms[1].set_dynamic(True)

        self.pr.step()


    
        # Generate trajectory
        t = sp.Symbol('t')

        # Step response
        traj = [
            (-45/180*np.pi)*sp.ones(1),
            (0/180*np.pi)*sp.ones(1),
            (0/180*np.pi)*sp.ones(1),
            0.000*sp.ones(1)
        ]
        # traj = [
        #     (-30/180*np.pi)*sp.sin(t*4),
        #     (30/180*np.pi)*sp.cos(t*4),
        #     (30/180*np.pi)*sp.cos(t*4),
        #     0.006*sp.sin(t/2)+0.006
        # ]
        self.pos = [sp.lambdify(t, i) for i in traj]
        self.vel = [sp.lambdify(t, i.diff(t)) for i in traj]
        self.acc = [sp.lambdify(t, i.diff(t).diff(t)) for i in traj]

    
    def signal_handler(self, sig, frame):
        """
        safely shutdown vrep when control C is pressed
        """
        rospy.loginfo("Calling exit for pyrep")
        self.shutdown_vrep()
        rospy.signal_shutdown("from signal_handler")

    def shutdown_vrep(self):
        """
        shutdown vrep safely
        """
        rospy.loginfo("Stopping pyrep.")
        self.pr.stop()
        rospy.loginfo("V-REP shutting down.")
        self.pr.shutdown()

    def publish(self):
        t = 0
        while not rospy.is_shutdown():
            posd = [float(j(t)) for j in self.pos]
            veld = [float(j(t)) for j in self.vel]
            # accd = [float(j(t)) for j in self.acc]
            target = JointState()
            target.position = posd
            target.velocity = veld
            t = t + self.dt
            signal.signal(signal.SIGINT, self.signal_handler)
            self.target_pub.publish(target)
            self.pr.step()



if __name__ == "__main__":
    ct = ControllerTester()
    try:
        ct.publish()
    except:
        pass