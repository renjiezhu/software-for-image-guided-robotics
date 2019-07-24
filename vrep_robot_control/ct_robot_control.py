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
    error = np.zeros(8)
    pe = np.asarray(robot._ik_tip.get_position())-np.asarray(robot._ik_target.get_position())
    oe = np.asarray(robot._ik_tip.get_orientation())-np.asarray(robot._ik_target.get_orientation())
    error[0:3] = pe
    error[3:6] = oe
    error[6] = np.linalg.norm(pe)
    error[7] = np.linalg.norm(oe)
    
    return error

def IK_via_vrep(robot: CtRobot, pos: list, ori: list, pr: PyRep, dt: float):
    '''
    pos = [x, y, z], ori = [alpha, beta, gamma] are the configuration parameter of needle frame with respect to 
    world frame.
    '''
    for i in range(robot._num_joints-1):
        robot.joints[i].set_joint_mode(JointMode.IK)
        robot.arms[i].set_dynamic(False)
    robot.joints[-1].set_joint_mode(JointMode.PASSIVE)
    robot.arms[-1].set_dynamic(False)

    robot._ik_target.set_position(pos)
    robot._ik_target.set_orientation(ori)
    pr.step()
    
    # joint_pos = []
    t = 0
    er = reach_target(robot)
    tmp = np.zeros(robot._num_joints)
    
    while er[6]>1e-4 and t<4*dt and er[7]>3e-3: 
        # Precision is set to 0.1 mm and 0.1 deg in terms of pos and ori respectively 
        pr.step()
        # for i in range(robot._num_joints):
        #     tmp[i] = robot.joints[i].get_joint_position()
        # joint_pos.append(tmp)
        t += dt
        er = reach_target(robot)
        
    if er[6]<1.7e-4 and er[7]<3e-3:
        print('Reached Target')
    else:
        if er[6]>1.7e-4:
            print('error on x-axis: %.6f' % er[0])
            print('error on y-axis: %.6f' % er[1])
            print('error on z-axis: %.6f' % er[2])
            print('Unable to reach target with respect to position, Error is %.6f' % er[6])
        if er[7]>3e-3:
            print('error on x-axis: %.6f' % er[3])
            print('error on y-axis: %.6f' % er[4])
            print('error on z-axis: %.6f' % er[5])
            print('Unable to reach target with respect to orientation, Error is %.6f' % er[7])
    # return np.asarray(joint_pos[-1])
    
            

if __name__ == "__main__":
    
    pr = PyRep()
    pr.launch('/media/acrmri/A5EB-573A/arm.ttt')

    dt = 0.01
    pr.set_simulation_timestep(dt)
    pr.start()
    ct_robot = CtRobot()

    ct_robot._ik_tip = Dummy('tip_frame')
    origin_joint_pos = ct_robot.get_joint_positions()
    init_pos = ct_robot._ik_target.get_position()
    init_ori = ct_robot._ik_target.get_orientation()
    
    # Lower and Upper limit of change of configuration
    pos_low = np.array([-0.006, -0.005, -0.003])
    pos_upper = np.array([0.006, 0.005, 0.003])
    ori_low = np.array([-0.2, -0.1, -0.2])
    ori_upper = np.array([0.2, 0.1, 0.2])

    ct_robot.set_joint_positions(origin_joint_pos)
    pr.step()
    loop = 10
    for i in range(loop):
        new_pos = np.random.uniform(pos_low, pos_upper) + np.asarray(init_pos)
        new_ori = np.random.uniform(ori_low, ori_upper) + np.asarray(init_ori)
        # print(new_pos)
        # print(new_ori)
        ct_robot._ik_target.set_position(new_pos.tolist())
        ct_robot._ik_target.set_orientation(new_ori.tolist())
        ct_robot.set_joint_positions(origin_joint_pos)
        pr.step()
        joint_pos = perform_IK_via_vrep(ct_robot, new_pos.tolist(), new_ori.tolist())
    
    pr.stop()



