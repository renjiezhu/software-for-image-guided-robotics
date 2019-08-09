import numpy as np



def cacl_tau(robot, kv, kp, posd:list, posm, veld, velm, xyz, accd=None):
    '''
    robot: Class that contains dynamic information of robot
    posd: desired position of joint
    posm: measured position of joint
    veld: desired angular or linear velocity of joint
    velm: measured velocity of joint
    '''
    num_joints = robot.num_joints
    gain_v = kv.dot((veld-velm).reshape(num_joints, 1))
    gain_x = kp.dot((np.array(posd)-posm).reshape(num_joints, 1))
    M = np.array(robot.Mq(posd))        # (num_joints, num_joints)
    G = np.array(robot.Gq(posd))        # (num_joints, 1)
    if accd is not None:
        tau = G+M.dot((gain_v + gain_x +accd.reshape(num_joints, 1)))
    else:
        tau = G+M.dot((gain_v + gain_x))
    return tau.reshape(num_joints)


def cacl_tau_ModelFree(robot, kv, kp, posd:list, posm, veld, velm, xyz, accd=None):
    '''
    robot: Class that contains dynamic information of robot
    posd: desired position of joint
    posm: measured position of joint
    veld: desired angular or linear velocity of joint
    velm: measured velocity of joint
    '''
    num_joints = robot.num_joints
    gain_v = kv.dot((veld-velm).reshape(num_joints, 1))
    gain_x = kp.dot((np.array(posd)-posm).reshape(num_joints, 1))
    M = np.eye(num_joints)              # (num_joints, num_joints)
    G = np.zeros((num_joints, 1))       # (num_joints, 1)
    if accd is not None:
        tau = G+M.dot((gain_v + gain_x +accd.reshape(num_joints, 1)))
    else:
        tau = G+M.dot((gain_v + gain_x))
    return tau.reshape(num_joints)


def cacl_tau_GravityCompensation(robot, kv, kp, posd:list, posm, veld, velm, xyz, accd=None):
    '''
    robot: Class that contains dynamic information of robot
    posd: desired position of joint
    posm: measured position of joint
    veld: desired angular or linear velocity of joint
    velm: measured velocity of joint
    '''
    num_joints = robot.num_joints
    gain_v = kv.dot((veld-velm).reshape(num_joints, 1))
    gain_x = kp.dot((np.array(posd)-posm).reshape(num_joints, 1))
    M = np.eye(num_joints)              # (num_joints, num_joints)
    G = np.array(robot.Gq(posd))        # (num_joints, 1)
    if accd is not None:
        tau = G+M.dot((gain_v + gain_x +accd.reshape(num_joints, 1)))
    else:
        tau = G+M.dot((gain_v + gain_x))
    return tau.reshape(num_joints)

