#!/usr/bin/env python3

import signal
import scipy.signal as sg
import numpy as np
import rospy
import time
from sensor_msgs.msg import JointState

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
    pub = rospy.Publisher("joint_setpoint", JointState, queue_size=1)
    dt = 1/sample_rate

    setpoint = JointState()
    setpoint.position = [0]*8
    setpoint.velocity = [0]*8
    time.sleep(12)


    rate = rospy.Rate(sample_rate)
    time = 0
    stop = False

    while not rospy.is_shutdown() and not stop:
        frequency = 0.5*np.pi
        
        setpoint.position[0] = np.sin(time*frequency)*25 - 30
        setpoint.velocity[0] = np.cos(time*frequency)*25 - 30

    
        setpoint.position[1] = np.sin(time*frequency)*25 - 30
        setpoint.velocity[1] = np.cos(time*frequency)*25 - 30

        setpoint.position[2] = np.sin(time*frequency)*25 + 30
        setpoint.velocity[2] = np.cos(time*frequency)*25 + 30

        '''
        setpoint.position[3] = np.sin(time*frequency)*80
        setpoint.velocity[3] = np.cos(time*frequency)*80        

        '''
        setpoint.position[4] = np.sin(frequency*time)*3.0
        setpoint.velocity[4] = np.cos(frequency*time)*3.0

        setpoint.position[5] = np.sin(frequency*time)*3.0
        setpoint.velocity[5] = np.cos(frequency*time)*3.0

        setpoint.position[6] = np.sin(frequency*time)*2.6
        setpoint.velocity[6] = np.cos(frequency*time)*2.6/2
        
        setpoint.position[7] = np.sin(frequency*time)*2.6
        setpoint.velocity[7] = np.cos(frequency*time)*2.6/2
        

        if time > 10000:
            stop =  True
        setpoint.header.stamp = rospy.Time.now()
        pub.publish(setpoint)
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



