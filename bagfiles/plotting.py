#!/usr/bin/env python3
"""
 * Software Interface for Image Guided Robotics
 * 
 * Plotting
 * 
 * By Renjie Zhu (rezhu@eng.ucsd.edu)
 * 
 * August 14, 2019
 * 
"""


import rosbag
import numpy as np
import matplotlib.pyplot as plt

# filename = input("Which bagfile?")
# topicname = '/' + input("Which topic? ")
# type_name = input("Type name? ")

filename = "/home/guosong/.ros/p_t.bag"
tpc = "/vrep_ros_interface/measured_joint_angles"

def getDataTime(filename, topicname, type_name):
    
    time = []
    data = []

    with rosbag.Bag(filename) as bag:
        for _, msg, t in bag.read_messages(topics=[topicname]):
            data.append(eval("msg."+type_name))
            time.append(t.secs+t.nsecs*1e-9)
    
    return data, time


def myplot():
    pos0, time = getDataTime(filename, tpc, "position[0]")
    pos1, time = getDataTime(filename, tpc, "position[1]")
    pos2, time = getDataTime(filename, tpc, "position[2]")
    pos3, time = getDataTime(filename, tpc, "position[3]")

    plt.figure()
    plt.plot(time, np.array(pos0)/np.pi*180, label='joint1')
    # plt.hlines(-30)
    plt.plot(time, np.array(pos1)/np.pi*180, label='joint2')
    # plt.hlines(45)
    plt.plot(time, np.array(pos2)/np.pi*180, label='joint3')
    # plt.hlines(20)
    plt.legend()
    plt.grid()

    plt.figure()
    plt.plot(time, np.array(pos3)*1000, label='joint4')
    # plt.hlines(5)
    plt.legend()
    plt.grid()
