#!/usr/bin/env python3
import signal
import sys, os
import transforms3d as tf3d
import time
import numpy as np
import rospy
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Quaternion

def signal_handler(sig, frame):
    """
    safely shutdown vrep when control C is pressed
    """
    rospy.loginfo("Calling for exit")
    streamingClient.c
    rospy.signal_shutdown("from signal_handler")


from utils.NatNetClient2 import NatNetClient
from utils.GetJointData import data, NatNetFuncs #receiveNewFrame, receiveRigidBodyFrameList
from utils.getRobotPose import getOptitrackPose

print_trak_data = False
print_cartesian = True

server_ip = "192.168.0.115"
multicastAddress = "239.255.42.99"

joint_names = ['base','needle']
ids = [0, 1]


#Tracking class
print("Starting streaming client now...")
streamingClient = NatNetClient(server_ip, multicastAddress, verbose = print_trak_data)
NatNet = NatNetFuncs()
streamingClient.newFrameListener = NatNet.receiveNewFrame
streamingClient.rigidBodyListListener = NatNet.receiveRigidBodyFrameList
prev_frame = 0
time.sleep(0.5)
streamingClient.run()
time.sleep(0.5)
track_data = data(joint_names, ids)
time.sleep(0.5)
track_data.parse_data(NatNet.joint_data, NatNet.frame) #updates the frame and data that is being used

# Record the initial relativa transformation between base and needle
# Subtract it to obtain the absolute relative transformation of needle
base = track_data.bodies[0].homogenous_mat
base_inv = track_data.bodies[0].homg_inv
rgdbdy = track_data.bodies[1].homogenous_mat
init_mat, init_pos, init_euler, init_quat = track_data.homg_mat_mult(base_inv, rgdbdy)


# Initial a ros node
rospy.init_node('optitrack', anonymous=True)
pub = rospy.Publisher('ee_measured_transform', Transform, queue_size=10)
rate = rospy.Rate(100)

while not rospy.is_shutdown():

    track_data.parse_data(NatNet.joint_data, NatNet.frame)

    trans = Transform()
    quat = Quaternion()

    # Transformation of base
    base = track_data.bodies[0].homogenous_mat
    base_inv = track_data.bodies[0].homg_inv

    # rospy.loginfo(f"Base:  {base}")
    # Transformation of needle
    rgdbdy = track_data.bodies[1].homogenous_mat
    # rospy.loginfo(f"Rigid body:  {rgdbdy}")

    # Relative transformation
    rel_mat, rel_pos, rel_euler, rel_quat = track_data.homg_mat_mult(base_inv, rgdbdy)
    
    # Convert into deg and mm
    rel_deg = np.array(rel_euler)*180/np.pi
    rel_pos_mm = np.array(rel_pos)*1000
    
    rospy.loginfo("Relative transformation between base and needle:\n\n\
    Homgenuous matrix of initial relative transformation: \n{}, \nposition: {}, \neuler angels: {}, \nquaternion:, {}" .\
    format(rel_mat,np.array(rel_pos)-np.array(init_pos), np.array(rel_euler)-np.array(init_euler), np.array(rel_quat)-np.array(init_quat)))
    # format(rel_mat,rel_pos, rel_euler, rel_quat))

    # publish to ROS
    trans.translation.x, trans.translation.y, trans.translation.z = rel_pos
    quat.x, quat.y, quat.z, quat.w = rel_quat
    trans.rotation = quat
    pub.publish(trans)

    # Interrupt
    signal.signal(signal.SIGINT, signal_handler)

    rate.sleep()
    
    