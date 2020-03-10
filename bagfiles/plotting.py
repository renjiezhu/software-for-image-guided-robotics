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
import rospy
import numpy as np
from geometry_msgs.msg import Pose




def parseData(filename, topicname="magTrackerPose", type_name="position"):
    time = []
    data = []
    with rosbag.Bag(filename) as bag:
        for _, msg, t in bag.read_messages(topicname):
            data.append(eval("msg."+type_name))
            time.append(t.secs+t.nsecs*1e-9) 
    return data, time


def getMean():
    pass


def getStd():
    pass




if __name__=="__main__":
    filename = "/home/" + os.environ["USER"] + "/Documents/igr/.bag/mag_tracker_data_test1.bag"