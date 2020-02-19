#!/usr/bin/env python3

import signal
import scipy.signal as sg
import numpy as np
import rospy
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

# def signal_handler(sig, frame):
#     rate = rospy.Rate(sample_rate)
#     velocity = np.zeros(8)

#     for j in range(2000):
#         for i in range(8):
#             setpoint.position[i] = 0
#             setpoint.velocity[i] = -1*np.pi/8*np.sign(setpoint.position[i])
#             setpoint.header.stamp = rospy.Time.now()
#         pub.publish(setpoint)
#         rate.sleep()

#     rospy.signal_shutdown("from signal_handler")


class RobotHardware:
    """
    A class robot hardware class that handles safety precautions
    for joint limits, motor to joint values,
    joint mixing matrix, and etc.
    """
    def __init__(self, dt):
        self.dt = dt
        self.sample_rate = 1/self.dt


        rospy.init_node("robot_hardware", anonymous=True)
        # pub = rospy.Publisher("motorSetpoint", JointState, queue_size=1)
        # pubFrequency = rospy.Publisher("masterControlLoopFrequency", Int32, queue_size=1)
        # mclFrequency = Int32(sample_rate)
        self.rate = rospy.Rate(self.sample_rate)

        self.zero_motor_angles = np.zeros(8)

        self.setpoint = JointState()
        self.setpoint.position = [0]*8
        self.setpoint.velocity = [0]*8
        self.setpoint.effort = [0]*8

        self.joint_positions = np.zeros(8)
        self.joint_upper_limits = np.array([0.2,0.13,0.34,np.pi,np.pi/3,np.pi/3,np.pi/3,np.pi/3])
        self.joint_lower_limits = np.array([0,0,0,-1*np.pi,-1*np.pi/3,-1*np.pi/3,-1*np.pi/3,-1*np.pi/3])
        self.arm_mixing_matrix = np.zeros((4,4))
        self.j0_pitch = 0.005 #meters/rad
        self.j1_pitch = 0.002 #meters/rad
        self.j2_pitch = 0.01 #meters/rad
        self.j3_ratio = 1.0 #output/input (unitless --> both rad)

        self.motor_positions = np.zeros(8)
        self.motor_positions_old = np.zeros(8)
        self.motor_velocities = np.zeros(8)

        self.joint_positions = np.zeros(8)

        self.calculate_joint2motor_arm_mixing_matrix()

        self.clipped_message_pub = rospy.Publisher(
            "joint_setpoint_clipped", JointState, queue_size=1
        )

        # self.sub = rospy.Subscriber("/sim_ros_interface/set_point_IK", JointState, callback=self.clippingCallback)
        

    def signal_handler(self, sig, frame):

        velocity = np.zeros(8)

        for j in range(2000):
            for i in range(8):
                self.setpoint.position[i] = 0
                self.setpoint.velocity[i] = -1*np.pi/8*np.sign(self.setpoint.position[i])
                self.setpoint.header.stamp = rospy.Time.now()
            self.clipped_message_pub.publish(self.setpoint)
            self.rate.sleep()

        rospy.signal_shutdown("from [RobotHW] signal_handler")

    def zero_linear_axis(self):
        for _ in range(2):
            self.update_motor_positions()

    def calculate_joint2motor_arm_mixing_matrix(self):
        self.arm_mixing_matrix = np.array([[ 5.18518519,  0.        ,  0.        ,  0.        ],
                                           [ -6.5993266/1.5 ,  5.18518519,  0.        ,  0.        ],
                                           [13.72895623/7,  5.18518519,  4.07407407,  0.        ],
                                           [-17.89036195/16,  6.48148148,  2.5462963 ,  1.85185185]])
    
    def set_joint_positions(self, joint_setpoint):
        self.joint_positions = np.clip(joint_setpoint, self.joint_lower_limits, self.joint_upper_limits)
        self.update_motor_positions()
        self.reorder_setpoint_message()

    def update_motor_positions(self):
        self.motor_positions_old = self.motor_positions.copy()

        self.motor_positions[0] = self.joint_positions[0] / self.j0_pitch * -1 * (2*np.pi)
        self.motor_positions[1] = self.joint_positions[1] / self.j1_pitch * (2*np.pi)
        self.motor_positions[2] = self.joint_positions[2] / self.j2_pitch * -1 * (2*np.pi)
        self.motor_positions[3] = self.joint_positions[3] / self.j3_ratio 
        self.motor_positions[4:] = self.arm_mixing_matrix @ self.joint_positions[4:]
        self.motor_positions -= self.zero_motor_angles

        unclipped_velocity = (self.motor_positions - self.motor_positions_old) / self.dt
        self.motor_velocities = np.clip(unclipped_velocity, -100, 100)

    def reorder_setpoint_message(self):
        self.setpoint.position[0] = self.motor_positions[2].squeeze().astype(float)
        self.setpoint.velocity[0] = self.motor_velocities[2].squeeze().astype(float)

        self.setpoint.position[1] = self.motor_positions[0].squeeze().astype(float)
        self.setpoint.velocity[1] = self.motor_velocities[0].squeeze().astype(float)

        self.setpoint.position[2] = self.motor_positions[1].squeeze().astype(float)
        self.setpoint.velocity[2] = self.motor_velocities[1].squeeze().astype(float)
        self.setpoint.effort[2] = 0.13 #z axis gravity compensation

        self.setpoint.position[3] = self.motor_positions[3].squeeze().astype(float)
        self.setpoint.velocity[3] = self.motor_velocities[3].squeeze().astype(float)

        self.setpoint.position[5] = self.motor_positions[4].squeeze().astype(float)
        self.setpoint.velocity[5] = self.motor_velocities[4].squeeze().astype(float)

        self.setpoint.position[6] = self.motor_positions[5].squeeze().astype(float)
        self.setpoint.velocity[6] = self.motor_velocities[5].squeeze().astype(float)
        
        self.setpoint.position[4] = self.motor_positions[7].squeeze().astype(float)
        self.setpoint.velocity[4] = self.motor_velocities[7].squeeze().astype(float)

        self.setpoint.position[7] = self.motor_positions[6].squeeze().astype(float)
        self.setpoint.velocity[7] = self.motor_velocities[6].squeeze().astype(float)
        
        self.setpoint.header.stamp = rospy.Time.now()

    def get_motor_positions_velocities(self):
        return self.motor_positions, self.motor_velocities

    def get_joint_positions(self):
        return self.joint_positions

    def run(self):
        """
        keep the subscriber running
        """
        # while not rospy.is_shutdown():
        # rospy.Subscriber("/vrep_IK/set_point_IK", JointState, callback=self.clippingCallback)
        rospy.Subscriber("joint_angles_streaming", JointState, callback=self.clippingCallback)
        signal.signal(signal.SIGINT, self.signal_handler)

        rospy.spin()


    def clippingCallback(self, data):
        self.set_joint_positions(np.array(data.position))
        self.clipped_message_pub.publish(self.setpoint)





if __name__=="__main__":

    # sample_rate = 500
    # dt = 1/sample_rate

    # rospy.init_node("robot_hardware", anonymous=True)
    # pub = rospy.Publisher("motorSetpoint", JointState, queue_size=1)
    # pubFrequency = rospy.Publisher("masterControlLoopFrequency", Int32, queue_size=1)
    # mclFrequency = Int32(sample_rate)
    # rate = rospy.Rate(sample_rate)

    robot = RobotHardware(dt=0.002)
    robot.run()

    # frequency = 0.02*2*np.pi

    # time.sleep(2)

    # time = 0
    # time_old = 0
    # state = 0

    # while not rospy.is_shutdown():
        
    #     if state == 0:
    #         joint_setpoint = np.array([np.sin(time*frequency)*robot.joint_upper_limits[0]*0.4 + robot.joint_upper_limits[0]/2, np.sin(time*frequency/3)*robot.joint_upper_limits[1]*0.4 + robot.joint_upper_limits[1]/2, np.sin(time*frequency)*robot.joint_upper_limits[2]*0.2 + robot.joint_upper_limits[2]/4, 0, 0, 0, 0, 0])#[:,np.newaxis]
    #     elif state == 1:
    #         joint_setpoint = np.array([0, np.sin(time*frequency/3)*robot.joint_upper_limits[1]*0.4 + robot.joint_upper_limits[1]/2, 0.02, 0, 0, 0, 0, 0])
    #     elif state == 2:
    #         joint_setpoint = np.array([0, 0, np.sin(time*frequency)*robot.joint_upper_limits[2]*0.2 + robot.joint_upper_limits[2]/4, 0, 0, 0, 0, 0])
    #     #skipping rotary base axis due to poor current tuning
    #     elif state == 3:
    #         joint_setpoint = np.array([0, 0.01, 0, 0, np.sin(time*frequency)*robot.joint_upper_limits[5]*0.8, np.sin(time*frequency)*robot.joint_upper_limits[5]*0.8, np.sin(time*frequency)*robot.joint_upper_limits[5]*0.8, 0])
    #     elif state == 4:
    #         joint_setpoint = np.array([0, 0.01, 0.0, 0, 0, np.sin(time*frequency)*robot.joint_upper_limits[5]*0.8, 0, 0])
    #     elif state == 5:
    #         joint_setpoint = np.array([0, 0.01, 0.0, 0, 0, 0, np.sin(time*frequency)*robot.joint_upper_limits[5]*0.8, 0])
        
    #     if time-time_old > 30:
    #         time_old = time
    #         state += 1
    #         state = state % 1

    #     robot.set_joint_positions(joint_setpoint)
    #     motor_setpoint, motor_velocity = robot.get_motor_positions_velocities()
                
    #     pub.publish(robot.setpoint)
    #     pubFrequency.publish(mclFrequency)

    #     # rospy.loginfo(setpoint.position)
    #     time += dt
    #     signal.signal(signal.SIGINT, robot.signal_handler)
    #     rate.sleep()



