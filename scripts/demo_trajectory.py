#!/usr/bin/env python3

import signal
import scipy.signal as sg
import numpy as np
import rospy
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

L_INCRE = 0.04
R_INCRE = 0.04
   
sample_rate = 250


def signal_handler(sig, frame):
    rate = rospy.Rate(sample_rate)
    setpointOld = setpoint

    for j in range(250):
        for i in range(8):
            setpoint.position[i] = 0
            setpoint.velocity[i] = setpoint.position[i] - setpointOld.position[i]
            setpoint.header.stamp = rospy.Time.now()
        pub.publish(setpoint)
        rate.sleep()


    rospy.signal_shutdown("from signal_handler")



if __name__=="__main__":

    rospy.init_node("Demo_trajectory", anonymous=True)
    pub = rospy.Publisher("joint_setpoint_clipped", JointState, queue_size=1)
    dt = 1/sample_rate

    pubFrequency = rospy.Publisher("masterControlLoopFrequency", Int32, queue_size=1)
    mclFrequency = Int32(sample_rate)

    setpoint = JointState()
    setpoint.position = [0]*8
    setpoint.velocity = [0]*8
    time.sleep(5)


    rate = rospy.Rate(sample_rate)
    time = 0
    stop = False

    motor_setpoint_old = np.zeros(4)

    while not rospy.is_shutdown() and not stop:
        frequency = 0.1*np.pi
        
        # setpoint.position[0] = -25
        # setpoint.velocity[0] = 0

        setpoint.position[0] = np.sin(time*frequency)*50 - 51
        setpoint.velocity[0] = np.cos(time*frequency)*50*frequency
    
    #    setpoint.position[1] = np.sin(time*frequency)*25 - 30
    #    setpoint.velocity[1] = np.cos(time*frequency)*25*frequency

    #    setpoint.position[2] = np.sin(time*frequency)*25 + 30
    #    setpoint.velocity[2] = np.cos(time*frequency)*25*frequency

    #    setpoint.position[0] = 0
    #    setpoint.velocity[0] = 0
    
        setpoint.position[1] = 0
        setpoint.velocity[1] = 0

        setpoint.position[2] = 0
        setpoint.velocity[2] = 0

        '''
        setpoint.position[3] = np.sin(time*frequency)*80
        setpoint.velocity[3] = np.cos(time*frequency)*80        

        '''
        # setpoint.position[4] = np.sin(frequency*time)*3.0
        # setpoint.velocity[4] = np.cos(frequency*time)*3.0*frequency

        # setpoint.position[4] = 3.0
        # setpoint.velocity[4] = 0

        # setpoint.position[5] = np.sin(time*frequency)*2
        # setpoint.velocity[5] = np.cos(time*frequency)*2*frequency 

        
        T4_inv = np.array([[ 5.18518519,  0.        ,  0.        ,  0.        ],
       [ -6.5993266/1.5 ,  5.18518519,  0.        ,  0.        ],
       [13.72895623/7,  5.18518519,  4.07407407,  0.        ],
       [-17.89036195/16,  6.48148148,  2.5462963 ,  1.85185185]])

        joint_setpoint = np.array([0, np.sin(time*frequency)*np.pi/3, 0, 0])#[:,np.newaxis]
        
        
        motor_setpoint = T4_inv.dot(joint_setpoint)
        motor_velocity = (motor_setpoint - motor_setpoint_old) / dt
        motor_setpoint_old = motor_setpoint.copy()
        #5 / 1 is base joint
        #4 / 0 is linear
        #6 / 2 is 2nd revolute
        #7 / 3 is 3rd revolute

        motor_setpoint = motor_setpoint.astype(float)
        motor_velocity = motor_velocity.astype(float)

        # print(T4_inv)
        # print(motor_setpoint)
        # print(motor_velocity)

        setpoint.position[5] = motor_setpoint[0].squeeze().astype(float)
        setpoint.velocity[5] = motor_velocity[0].squeeze().astype(float)

        setpoint.position[6] = motor_setpoint[1].squeeze().astype(float)
        setpoint.velocity[6] = motor_velocity[1].squeeze().astype(float)
        
        setpoint.position[4] = motor_setpoint[3].squeeze().astype(float)
        setpoint.velocity[4] = motor_velocity[3].squeeze().astype(float)

        setpoint.position[7] = motor_setpoint[2].squeeze().astype(float)
        setpoint.velocity[7] = motor_velocity[2].squeeze().astype(float)

        # print(setpoint)
        
        # setpoint.position[5] = np.sin(frequency*time)*3.0
        # setpoint.velocity[5] = np.cos(frequency*time)*3.0*frequency

        # setpoint.position[6] = np.sin(frequency*time)*2.6
        # setpoint.velocity[6] = np.cos(frequency*time)*2.6*frequency
        
        # setpoint.position[7] = np.sin(frequency*time)*2.6
        # setpoint.velocity[7] = np.cos(frequency*time)*2.6*frequency
        

        if time > 10000:
            stop =  True
        setpoint.header.stamp = rospy.Time.now()
        pub.publish(setpoint)
        pubFrequency.publish(mclFrequency)

        # rospy.loginfo(setpoint.position)
        time += dt
        signal.signal(signal.SIGINT, signal_handler)
        rate.sleep()

    setpointOld = setpoint

    for j in range(250):
        for i in range(8):
            setpoint.position[i] = 0
            setpoint.velocity[i] = setpoint.position[i] - setpointOld.position[i]
            setpoint.header.stamp = rospy.Time.now()

        pub.publish(setpoint)
        rate.sleep()



