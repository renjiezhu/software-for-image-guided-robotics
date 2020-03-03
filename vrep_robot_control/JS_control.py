import numpy as np

'''
Joint-Space controller
NOTE: Mixing matrix is not inculded in the controller. Need to be added!!!!
'''


def cacl_tau(robot, kp, kv, posd: list, posm: list, veld: list, velm: list, xyz: list, accd=None):
    '''
    Regular torque control: mass and inertia term with gravity compensation implemented by PID controller

    Input: 
        robot = Class that contains dynamic information of robot
        kp = parameter of proportional part : square matrix whose size = # DOF
        kv = parameter of derivative part : square matrix whose size = # DOF
        posd = desired position of joint
        posm = measured position of joint
        veld = desired angular or linear velocity of joint
        velm = measured angular or linear velocity of joint
        accd = desired accelration of joint (Optional)

    Output:
        torque need to be directly applied on the joint 
    '''
    veld = np.array(veld).squeeze()
    velm = np.array(velm).squeeze()
    posd = np.array(posd).squeeze()
    posm = np.array(posm).squeeze()
    
    num_joints = robot._num_joints
    gain_v = kv.dot((veld-velm).reshape(num_joints, 1))
    gain_x = kp.dot((posd-posm).reshape(num_joints, 1))
    M = np.array(robot.Mq(posd, xyz))        # (num_joints, num_joints)
    G = np.array(robot.Gq(posd, xyz))        # (num_joints, 1)
    if accd is not None:
        tau = G+M.dot((gain_v + gain_x +accd.reshape(num_joints, 1)))
    else:
        tau = G+M.dot((gain_v + gain_x))
    return tau.squeeze()


def cacl_tau_ModelFree(robot, kp, kv, posd: list, posm: list, veld: list, velm: list, xyz: list, accd=None):
    '''
    Model-Free torque control: identity mass and inertia term implemented by PID controller

    Input: 
        robot: Class that contains dynamic information of robot
        kp = parameter of proportional part : square matrix whose size = # DOF
        kv = parameter of derivative part : square matrix whose size = # DOF
        posd = desired position of joint
        posm = measured position of joint
        veld = desired angular or linear velocity of joint
        velm = measured angular or linear velocity of joint
        accd = desired accelration of joint (Optional)

    Output:
        torque need to be directly applied on the joint 
    '''
    veld = np.array(veld).squeeze()
    velm = np.array(velm).squeeze()
    posd = np.array(posd).squeeze()
    posm = np.array(posm).squeeze()

    num_joints = robot._num_joints
    gain_v = kv.dot((veld-velm).reshape(num_joints, 1))
    gain_x = kp.dot((posd-posm).reshape(num_joints, 1))
    M = np.eye(num_joints)       # (num_joints, num_joints)
    G = np.zeros((num_joints, 1))        # (num_joints, 1)
    if accd is not None:
        tau = G+M.dot((gain_v + gain_x +accd.reshape(num_joints, 1)))
    else:
        tau = G+M.dot((gain_v + gain_x))
    return tau.squeeze()


def cacl_tau_GravityCompensation(robot, kp, kv, posd: list, posm: list, veld: list, velm: list, xyz: list, accd=None):
    '''
    Gravity compensation torque control: identity mass and inertia term with gravity compensation implemented by PID controller

    Input: 
        robot: Class that contains dynamic information of robot
        kp = parameter of proportional part : square matrix whose size = # DOF
        kv = parameter of derivative part : square matrix whose size = # DOF
        posd = desired position of joint
        posm = measured position of joint
        veld = desired angular or linear velocity of joint
        velm = measured angular or linear velocity of joint
        accd = desired accelration of joint (Optional)

    Output:
        torque need to be directly applied on the joint 
    '''
    veld = np.array(veld).squeeze()
    velm = np.array(velm).squeeze()
    posd = np.array(posd).squeeze()
    posm = np.array(posm).squeeze()

    num_joints = robot._num_joints
    gain_v = kv.dot((veld-velm).reshape(num_joints, 1))
    gain_x = kp.dot((posd-posm).reshape(num_joints, 1))
    M = np.eye(num_joints)        # (num_joints, num_joints)
    G = np.array(robot.Gq(posd, xyz))        # (num_joints, 1)
    if accd is not None:
        tau = G+M.dot((gain_v + gain_x +accd.reshape(num_joints, 1)))
    else:
        tau = G+M.dot((gain_v + gain_x))
    return tau.squeeze()
