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

filename = input("Which bagfile?")
topicname = '/' + input("Which topic? ")
type_name = input("Type name? ")

with rosbag.Bag(filename) as bag:
    time = []
    data = []
    for _, msg, t in bag.read_messages(topics=[topicname]):
        data.append(eval("msg."+type_name))
        time.append(t.secs+t.nsecs*1e-9)
