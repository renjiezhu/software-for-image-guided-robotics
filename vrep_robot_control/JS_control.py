import numpy as np

def cacl_torque(robot, posd, posm, veld, velm, xyz, accd=None):
    '''
    robot: Class that contains dynamic information of robot
    posd: numpy array of dimension 6 representing desired position and orientation of joint
    posm: numpy array of dimension 6 representing desired position and orientation of joint
    veld: numpy array of dimension 6 representing measured velocity of joint
    velm: numpy array of dimension 6 representing measured velocity of joint
    '''
    num_joints = robot.num_joints
    kv = np.diag([100, 200, 300, 200, 100, 15, 27.5])
    kp = np.diag([20, 100, 30, 40, 80, 5, 7])
    M = np.array(robot._Mq[0](*tuple(np.concatenate((posm, xyz)))))
    G = np.array(robot._Gq[0](*tuple(np.concatenate((posm, xyz))))).reshape(num_joints, 1)
    gain_v = kv.dot((veld-velm).reshape(num_joints, 1))
    gain_x = kp.dot((posd-posm).reshape(num_joints, 1))
    if accd is not None:
        tau = G+M.dot((gain_v + gain_x +accd.reshape(num_joints, 1)))
    else:
        tau = G+M.dot((gain_v + gain_x))
    return tau.reshape(num_joints)
