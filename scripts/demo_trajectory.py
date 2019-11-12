#!/usr/bin/env python3
import signal
import scipy.signal as sg
import numpy as np
import rospy
import time
from sensor_msgs.msg import JointState

L_INCRE = 500
R_INCRE = 1000
   
sample_rate = 250


def signal_handler(sig, frame):
    rate = rospy.Rate(250)
    while setpoint.position[0] < 0:
        setpoint.position[0] += L_INCRE
        setpoint.position[1] += L_INCRE
        setpoint.position[2] -= L_INCRE
        setpoint.position[3] -= L_INCRE
        setpoint.header.stamp = rospy.Time.now()
        pub.publish(setpoint)
        rate.sleep()

    while setpoint.position[2] > 0:
        setpoint.position[2] -= L_INCRE
        setpoint.header.stamp = rospy.Time.now()
        pub.publish(setpoint)
        rate.sleep()

    
    for i in range(8):
        setpoint.position[i] = 0
        setpoint.header.stamp = rospy.Time.now()
        pub.publish(setpoint)

    rospy.signal_shutdown("from signal_handler")



if __name__=="__main__":

    rospy.init_node("Demo_trajectory", anonymous=True)
    pub = rospy.Publisher("testing", JointState, queue_size=1)
    dt = 1/sample_rate

    setpoint = JointState()
    setpoint.position = [0]*8

    input("Please press Enter key to continue...")

    rate = rospy.Rate(250)
    while setpoint.position[0] > -240000:
        setpoint.position[0] -= L_INCRE
        setpoint.position[1] -= L_INCRE
        setpoint.position[2] += L_INCRE
        setpoint.position[3] += L_INCRE
        setpoint.header.stamp = rospy.Time.now()
        pub.publish(setpoint)
        rate.sleep()

    while setpoint.position[2] < 420000:
        setpoint.position[2] += L_INCRE
        setpoint.header.stamp = rospy.Time.now()
        pub.publish(setpoint)
        rate.sleep()

    rospy.loginfo("Reaching midpoint.............")
    time.sleep(12)
    rospy.loginfo("Reached!!")

    rate = rospy.Rate(500)
    time = 0
    stop = False
    while not rospy.is_shutdown() and not stop:
        setpoint.position[0] = np.sin(time/3)*80000 - 240000
        setpoint.position[1] = np.sin(time/11)*80000 - 240000
        setpoint.position[2] = np.sin(time/9.5)*80000 + 420000
        setpoint.position[3] = np.sin(time/7.1)*120000 + 240000
        # setpoint.position[5] = sg.square(2 * np.pi * 0.1 * time)*80000
        setpoint.position[5] = np.sin(time/13)*60000
        setpoint.position[6] = np.sin(time/11)*60000 
        setpoint.position[7] = np.sin(time/10)*60000
        if time > 2*np.pi*30:
            stop =  True
        setpoint.header.stamp = rospy.Time.now()
        pub.publish(setpoint)
        # rospy.loginfo(setpoint.position)
        time += dt
        signal.signal(signal.SIGINT, signal_handler)
        rate.sleep()

    rate = rospy.Rate(250)
    while setpoint.position[0] < 0:
        setpoint.position[0] += L_INCRE
        setpoint.position[1] += L_INCRE
        setpoint.position[2] -= L_INCRE
        setpoint.position[3] -= L_INCRE
        setpoint.header.stamp = rospy.Time.now()
        pub.publish(setpoint)
        rate.sleep()

    while setpoint.position[2] > 0:
        setpoint.position[2] -= L_INCRE
        setpoint.header.stamp = rospy.Time.now()
        pub.publish(setpoint)
        rate.sleep()


    for i in range(8):
        setpoint.position[i] = 0
        setpoint.header.stamp = rospy.Time.now()
        pub.publish(setpoint)





