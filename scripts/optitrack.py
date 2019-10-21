#!/usr/bin/env python3
import signal
import sys, os
import transforms3d as tf3d
import time
import numpy as np
import rospy
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Quaternion

# import socket
# IP_address = socket.getfqdn(socket.gethostname())
# print(IP_address)
def signal_handler(sig, frame):
    """
    safely shutdown vrep when control C is pressed
    """
    rospy.loginfo("Calling for exit")
    
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
# joint_names = ['base']
# ids = [0]

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
#debug values


# Initial a ros node
rospy.init_node('optitrack', anonymous=True)
pub = rospy.Publisher('ee_measured_transform', Transform, queue_size=100)
rate = rospy.Rate(100)

while not rospy.is_shutdown():
    trans = Transform()
    quat = Quaternion()

    time.sleep(0.1)
    base = track_data.bodies[0].homogenous_mat
    base_inv = track_data.bodies[0].homg_inv
    rgdbdy1 = track_data.bodies[1].homogenous_mat
    
    rgdbdy1_base, rgdbdy1_pos, rgdbdy1_euler, _ = track_data.homg_mat_mult(base_inv,rgdbdy1)
    
    rgdbdy1_deg = np.array(rgdbdy1_euler)*180/np.pi
    rgdbdy1_pos_mm = np.array(rgdbdy1_pos)*1000
    
    rospy.loginfo("rgdbdy: {}, rgdbdy_base: {}, rgdbdy_euler: {}, rgdbdy_deg: {}, rgdbdy_pos:, {}" .format(rgdbdy1, rgdbdy1_base, rgdbdy1_euler, rgdbdy1_deg, rgdbdy1_pos_mm))

    trans.translation = rgdbdy1_pos
    quat.x, quat.y, quat.z, quat.w = tf3d.quaternions.mat2quat(rgdbdy1_base[:3, :3])
    trans.rotation = quat

    # publish
    pub.publish(trans)

    signal.signal(signal.SIGINT, signal_handler)


    rate.sleep()
    
    