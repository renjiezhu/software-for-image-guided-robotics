import numpy as np
from pyrep import PyRep
from arm import CtRobot
from my_rigid_kinematics import dh_robot_config
import sympy as sp
import os
import transforms3d as t3d
from pyrep.backend import vrep

pi = np.pi

pr = PyRep()
pr.launch(os.getcwd() + '/Temp.ttt', headless=True)

pr.start()
ct_robot = CtRobot()

# Grab modified DH parameters from V-REP using the dummy 'DH_frame'
# Need to set up their orientation following modified DH convention
D = []
theta = []
alpha = []
a = []
L = []
jointType = ['p', 'p', 'r', 'r', 'r', 'r', 'p']
for i in range(ct_robot._num_joints):
    j = i+1
    tmp_alpha = ct_robot.DH_frames[i+1].get_orientation(relative_to=ct_robot.DH_frames[i])
    tmp_a = ct_robot.DH_frames[i+1].get_position(relative_to=ct_robot.DH_frames[i])
    if tmp_alpha[0] < -1e-5 or tmp_alpha[0] > 1e-5:
        alpha.append(tmp_alpha[0])
    else:
        alpha.append(0)
    a.append(tmp_a[0])
    
    tmp_d = -ct_robot.DH_frames[j-1].get_position(relative_to=ct_robot.DH_frames[j])[2]
    tmp_theta = -ct_robot.DH_frames[j-1].get_orientation(relative_to=ct_robot.DH_frames[j])[2]
    if tmp_theta < -1e-5 or tmp_theta > 1e-5:
        theta.append(tmp_theta)
    else:
        theta.append(0)   
    D.append(tmp_d)
# Center of mass of link with respect to each joint    
for i in range(ct_robot._num_joints+1):
    L.append(ct_robot.COMs[i].get_position(relative_to=ct_robot.DH_frames[i]))
L = np.array(L)
# Base transformation of joint 0 with respect to world
Tbase = np.concatenate((np.array(ct_robot.DH_frames[0].get_matrix()).reshape(3,4), np.array([[0,0,0,1]])))
# Number of joints
num_joints = ct_robot._num_joints
# Mass information of each part of robot ( Any possibility to automatically grab from V-REP??)
# Commute with Lua script in V-REP
M = np.array([
    [5.539, 5.539, 5.539, 2.152e-2, 7.838e-3, 2.866e-2],
    [6.938e-1, 6.938e-1, 6.938e-1, 9.52e-3, 2.751e-2, 3.603e-2],
    [1.196, 1.196, 1.196, 2.411e-2, 9.92e-2, 1.164e-2],
    [4.147, 4.147, 4.147, 4.954e-2, 1.441e-1, 1.814e-2],
    [3.298e-2, 3.298e-2, 3.298e-2, 3.752e-1, 2.050e-4, 3.75e-1],
    [2.375e-2, 2.375e-2, 2.375e-2, 4.592e-2, 1.718e-4, 4.591e-1],
    [3.226e-2, 3.226e-2, 3.226e-2, 5.793e-1, 0, 5.771e-1],
    [1.491e-2, 1.491e-2, 1.491e-2, 6.070e-1, 1.831e-4, 6.058e-1]
])
# Check with FK library to confirm Transformation is correct
robot = dh_robot_config(num_joints, alpha, theta, D, a, jointType, Tbase, L, M)
robot.initKinematicTransforms()


# Print all DH parameters
for i in range(7):
    print('---------------------------')
    print('Joint {}'.format(i+1))
    print('D: {}, theta: {}, a[i-1]: {}, alpha[i-1]: {}'.format(D[i],theta[i],a[i],alpha[i]))


# Confirmation with V-REP
set_jot_pos = [0.01, 0.02, 0.4, 0.5, 0, 0.3, 0.01]
#set_jot_pos = [0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0]
ct_robot.set_joint_positions(set_jot_pos)

# V-REP
joint_pos = [ct_robot.joints[i].get_joint_position() for i in range(ct_robot._num_joints)]
print('Joint position is', joint_pos)
print('End-joint position is', ct_robot._ik_tip.get_position())
print('End-joint orientation is', ct_robot._ik_tip.get_orientation())


# Check transformation from joint to joint
err_j2j = 0
for i in range(7):
    #vrep
    T = np.eye(4)
    T[:3, :] = np.array(ct_robot.DH_frames[i+1].get_matrix(ct_robot.DH_frames[i])).reshape(3,4)
    
    # FK
    Tp = sp.lambdify(robot.q, robot._Tj2j[i])(*tuple(joint_pos))
    print('Joint %d: diff:'%(i+1), np.linalg.norm(Tp - T))
    if np.linalg.norm(Tp - T) > 1e-4:
        err_j2j += 1
        print('Wrong match of joint %d'%(i+1))
        print('getting from V-REP')
        print(T)
        print('----------------------------------')
        print('getting from FK')
        print(Tp)
        print('----------------------------------')

# Check transformation from joint to base
err_j2b = 0
for i in range(7):
    #vrep
    T = np.eye(4)
    T[:3, :] = np.array(ct_robot.DH_frames[i+1].get_matrix()).reshape(3,4)
    
    # FK
    Tp = sp.lambdify(robot.q, robot._Tjoint[i])(*tuple(joint_pos))
    print('Joint %d: diff:'%(i+1), np.linalg.norm(Tp - T))
    if np.linalg.norm(Tp - T) > 1e-4:
        err_j2b += 1
        print('Wrong match of joint %d'%(i+1))
        print('getting from V-REP')
        print(T)
        print('----------------------------------')
        print('getting from FK')
        print(Tp)
        print('----------------------------------')

# Save to file
param = ['D', 'a', 'alpha', 'theta', 'num_joints', 'jointType', 'Tbase', 'L', 'M']
if err_j2b == 0 and err_j2j == 0:
    print('Check FK library with V-REP correct')
    print('--------------------------------------------------')
    print('Writing DH parameters, Mass and COM information into file')
    for i in range(len(param)):
        np.save('./robot_config/config_ct_7DOF/%s'%param[i], eval(param[i]))
    print('--------------------------------------------------')
    print('Writing complete')
else:
    print('Please check DH convention with V-REP')

pr.stop()
pr.shutdown()

