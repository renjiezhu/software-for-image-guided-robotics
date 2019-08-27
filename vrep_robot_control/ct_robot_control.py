from pyrep import PyRep
import pyrep
import numpy as np
import transforms3d as t3d
from vrep_robot_control.arm import CtRobot
from pyrep.const import JointType, JointMode
from pyrep.backend import vrep
from pyrep.objects.dummy import Dummy
from pyrep.errors import ConfigurationPathError


def reach_target(robot: CtRobot):
    '''
    Check if the robot has reached its target position and orientation. 
    If not, then it's high likely that robot are in singularity or target pose are not reachable.
    '''
    error = np.zeros(8)
    pe = np.asarray(robot._ik_tip.get_position())-np.asarray(robot._ik_target.get_position())
    oe = np.asarray(robot._ik_tip.get_orientation())-np.asarray(robot._ik_target.get_orientation())
    error[0:3] = pe
    error[3:6] = oe
    error[6] = np.linalg.norm(pe)
    error[7] = np.linalg.norm(oe)
    return error

def IK_via_vrep(robot: CtRobot, pos: list, ori: list, pr: PyRep, dt: float = 0.01):
    '''
    Input :
    robot = Class of robot arm based on PyRep
    pos = target position [x, y, z]
    ori = target orientation [alpha, beta, gamma]
    pr = PyRep handle
    dt = desired simulation time (default = 0.01s)

    The function is to call Pseudoinverse solver of VREP IK configuration 
    to perform inverse kinematics. And use 'step' function of PyRep to update 
    position of VREP model.
    '''

    # Set joint to be in IK mode and disable all dynamics of part
    for i in range(robot._num_joints-1):
        robot.joints[i].set_joint_mode(JointMode.IK)
        robot.arms[i].set_dynamic(False)

    # Disable last joint to make sure needle is not inserted during robot setup
    robot.joints[-1].set_joint_mode(JointMode.PASSIVE)
    robot.arms[-1].set_dynamic(False)

    # Set IK target to target pose
    robot._ik_target.set_position(pos)
    robot._ik_target.set_orientation(ori)
    pr.step()
    
    # Retrive joint position anc check whether it reach target pose
    joint_pos = []
    t = 0
    er = reach_target(robot)
    tmp = np.zeros(robot._num_joints)
    
    while er[6]>1.7e-4 and t<4*dt and er[7]>3e-3: 
        # Precision is set to 0.1 mm and 0.1 deg in terms of pos and ori respectively 
        for i in range(robot._num_joints):
            tmp[i] = robot.joints[i].get_joint_position()
        joint_pos.append(tmp)
        t += dt
        er = reach_target(robot)
        
    if er[6]<1.7e-4 and er[7]<3e-3:
        print('Reached Target')
    else:
        if er[6]>1.7e-4:
            print('Unable to reach target with respect to position, Error is %.6f' % er[6])
            print('error on x-axis: %.6f' % er[0])
            print('error on y-axis: %.6f' % er[1])
            print('error on z-axis: %.6f' % er[2])
            print('Check your movement of robot')
        if er[7]>3e-3:
            print('Unable to reach target with respect to orientation, Error is %.6f' % er[7])
            print('error on x-axis: %.6f' % er[3])
            print('error on y-axis: %.6f' % er[4])
            print('error on z-axis: %.6f' % er[5])
            print('Check your movement of robot')

    # return np.asarray(joint_pos[-1])
    





