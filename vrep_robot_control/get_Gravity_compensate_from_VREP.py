import numpy as np
import os
from my_rigid_kinematics import dh_robot_config
# from JS_control import cacl_torque
import matplotlib.pyplot as plt
from pyrep import PyRep
from arm import CtRobot
from pyrep.backend import vrep
import sympy as sp
from pyrep.const import JointMode
import time

pr = PyRep()


pr.launch(os.getcwd() + '/Modified_DH_ct_7DOF.ttt', headless=True)
ct_robot = CtRobot()

# Set up joints to be in force control mode
for i in range(ct_robot._num_joints):
    ct_robot.joints[i].set_joint_mode(JointMode.FORCE)
    ct_robot.joints[i].set_control_loop_enabled(False)
    ct_robot.joints[i].set_motor_locked_at_zero_velocity(False)
for j in range(1, ct_robot._num_joints + 1):
    ct_robot.arms[i].set_dynamic(True)
ct_robot.arms[0].set_dynamic(False)

pr.start()

for i in range(100):
    set_point = [0,0,0,0,0,0,0]
    max_force = [100, 100, 100, 100, 100, 100, 100]
    ct_robot.set_joint_target_velocities(set_point)
    ct_robot.set_joint_forces(max_force)
    pr.step()

    tau = ct_robot.get_joint_forces()
    print(tau)

pr.stop()
pr.shutdown()


