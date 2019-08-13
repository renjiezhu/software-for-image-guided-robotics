import numpy as np
import os
from DH_dynamics import dh_robot_config
from JS_control import *
import matplotlib.pyplot as plt
from pyrep import PyRep
from arm import CtRobot
from pyrep.backend import vrep
import sympy as sp
from pyrep.const import JointMode
import time

pr = PyRep()

# Robot dynamics import
param = ['D', 'a', 'alpha', 'theta', 'num_joints', 'jointType', 'Tbase', 'L', 'M']
config = dict()
for i in range(len(param)):
    config[param[i]] = np.load('./robot_config/config_ct_7DOF/%s.npy'%param[i])

robot = dh_robot_config(int(config['num_joints']), config['alpha'], config['theta'], config['D'], config['a'], 
config['jointType'], config['Tbase'], config['L'], config['M'])
robot.initKinematicTransforms()

# Open V-REP with PyRep
pr.launch(os.getcwd() + '/Modified_DH_ct_7DOF.ttt', headless=True)
ct_robot = CtRobot()
xyz = np.array([0, 0, 0])
dt = 0.01
pr.set_simulation_timestep(dt)

end_time = 4
t_dic = np.linspace(0, end_time, num=int(end_time/dt), endpoint=True)
t = sp.Symbol('t')

# Desired trajectory of system
# Step response
traj = [
    0.01 * sp.ones(1),
    0.01 * sp.ones(1),
    (15/180*np.pi)*sp.ones(1),
    (-30/180*np.pi)*sp.ones(1),
    (45/180*np.pi)*sp.ones(1),
    (20/180*np.pi)*sp.ones(1),
    0.005*sp.ones(1)
]

pos = [sp.lambdify(t, i) for i in traj]
vel = [sp.lambdify(t, i.diff(t)) for i in traj]
acc = [sp.lambdify(t, i.diff(t).diff(t)) for i in traj]

kp = np.diag([10, 10, 200, 2.5, 1, 0.4, 0.4])
kv = np.diag([0.8, 0.8, 160, 10, 7, 2, 3])

# Model Based controller
pr.start()
err_pos_MB = []
err_vel_MB = []
mespos_MB = []
depos_MB = []
fre_MB = np.zeros(int(end_time/dt))
# Set joint mode and dynamics simulation
for i in range(ct_robot._num_joints):
    ct_robot.joints[i].set_joint_mode(JointMode.FORCE)
    ct_robot.joints[i].set_control_loop_enabled(False)
    ct_robot.joints[i].set_motor_locked_at_zero_velocity(True)
for j in range(1, ct_robot._num_joints + 1):
    ct_robot.arms[i].set_dynamic(True)
ct_robot.arms[0].set_dynamic(False)
pr.step()
# Simulation
for i in range(int(end_time/dt)):
    posd = [float(j(t_dic[i])) for j in pos]
    veld = np.array([j(t_dic[i]) for j in vel]).squeeze()
    accd = np.array([j(t_dic[i]) for j in acc]).squeeze()
    posm = np.array(ct_robot.get_joint_positions())
    velm = np.array(ct_robot.get_joint_velocities())
    ts = time.time()
    tau = cacl_tau(robot, kp, kv, posd, posm, veld, velm, xyz, accd)
    te = time.time()
    fre_MB[i] = te-ts
    ct_robot.set_joint_target_velocities((np.sign(tau)*1000000).tolist())
    ct_robot.set_joint_forces(np.abs(tau).tolist())
    pr.step()
    measure_pos = np.array(ct_robot.get_joint_positions())
    measure_vel = np.array(ct_robot.get_joint_velocities())
    # Record error of each joint
    err_pos_MB.append(posd - measure_pos)
    err_vel_MB.append(veld - measure_vel)
    depos_MB.append(posd)
    mespos_MB.append(measure_pos)
pr.stop()




# Model Free controller
pr.start()
err_pos_MF = []
err_vel_MF = []
mespos_MF = []
depos_MF = []
fre_MF = np.zeros(int(end_time/dt))
# Set joint mode and dynamics simulation
for i in range(ct_robot._num_joints):
    ct_robot.joints[i].set_joint_mode(JointMode.FORCE)
    ct_robot.joints[i].set_control_loop_enabled(False)
    ct_robot.joints[i].set_motor_locked_at_zero_velocity(True)
for j in range(1, ct_robot._num_joints + 1):
    ct_robot.arms[i].set_dynamic(True)
ct_robot.arms[0].set_dynamic(False)
pr.step()
# Simulation
for i in range(int(end_time/dt)):
    posd = [float(j(t_dic[i])) for j in pos]
    veld = np.array([j(t_dic[i]) for j in vel]).squeeze()
    accd = np.array([j(t_dic[i]) for j in acc]).squeeze()
    posm = np.array(ct_robot.get_joint_positions())
    velm = np.array(ct_robot.get_joint_velocities())
    ts = time.time()
    tau = cacl_tau_ModelFree(robot, kp, kv, posd, posm, veld, velm, xyz, accd)
    te = time.time()
    fre_MF[i] = te-ts
    ct_robot.set_joint_target_velocities((np.sign(tau)*1000000).tolist())
    ct_robot.set_joint_forces(np.abs(tau).tolist())
    pr.step()
    measure_pos = np.array(ct_robot.get_joint_positions())
    measure_vel = np.array(ct_robot.get_joint_velocities())
    # Record error of each joint
    err_pos_MF.append(posd - measure_pos)
    err_vel_MF.append(veld - measure_vel)
    depos_MF.append(posd)
    mespos_MF.append(measure_pos)
pr.stop()




# Gravity compensation controller
pr.start()
err_pos_GC = []
err_vel_GC = []
mespos_GC = []
depos_GC = []
fre_GC = np.zeros(int(end_time/dt))
# Set joint mode and dynamics simulation
for i in range(ct_robot._num_joints):
    ct_robot.joints[i].set_joint_mode(JointMode.FORCE)
    ct_robot.joints[i].set_control_loop_enabled(False)
    ct_robot.joints[i].set_motor_locked_at_zero_velocity(True)
for j in range(1, ct_robot._num_joints + 1):
    ct_robot.arms[i].set_dynamic(True)
ct_robot.arms[0].set_dynamic(False)
pr.step()
# Simulation
for i in range(int(end_time/dt)):
    posd = [float(j(t_dic[i])) for j in pos]
    veld = np.array([j(t_dic[i]) for j in vel]).squeeze()
    accd = np.array([j(t_dic[i]) for j in acc]).squeeze()
    posm = np.array(ct_robot.get_joint_positions())
    velm = np.array(ct_robot.get_joint_velocities())
    ts = time.time()
    tau = cacl_tau(robot, kp, kv, posd, posm, veld, velm, xyz, accd)
    te = time.time()
    fre_GC[i] = te-ts
    ct_robot.set_joint_target_velocities((np.sign(tau)*1000000).tolist())
    ct_robot.set_joint_forces(np.abs(tau).tolist())
    pr.step()
    measure_pos = np.array(ct_robot.get_joint_positions())
    measure_vel = np.array(ct_robot.get_joint_velocities())
    # Record error of each joint
    err_pos_GC.append(posd - measure_pos)
    err_vel_GC.append(veld - measure_vel)
    depos_GC.append(posd)
    mespos_GC.append(measure_pos)
pr.stop()


# Change Unit
err_pos_MB = np.array(err_pos_MB)
mespos_MB = np.array(mespos_MB)
depos_MB = np.array(depos_MB)
err_pos_MF = np.array(err_pos_MF)
mespos_MF = np.array(mespos_MF)
depos_MF = np.array(depos_MF)
err_pos_GC = np.array(err_pos_GC)
mespos_GC = np.array(mespos_GC)
depos_GC = np.array(depos_GC)

for name in ['err_pos_MB', 'mespos_MB', 'depos_MB']:
    x = eval(name)
    x[:, 0:1] = x[:, 0:1]*1000
    x[:, 1:2] = x[:, 1:2]*1000
    x[:, 6:7] = x[:, 6:7]*1000
    x[:, 2:3] = x[:, 2:3]*180/np.pi
    x[:, 3:4] = x[:, 3:4]*180/np.pi
    x[:, 4:5] = x[:, 4:5]*180/np.pi
    x[:, 5:6] = x[:, 5:6]*180/np.pi

# Plot error of position from joint to joint
plt.figure(figsize=(18,10))
plt.suptitle('Absoluter value')
for i in range(6):
    plt.subplot(2,4,i+1)
    plt.title('Joint%d'%(i+1))
    plt.plot(mespos_MB[:, i], label='mesaured')
    plt.plot(depos_MB[:, i], label='desired')
    plt.legend()
    if i in [0,1,6]:
        plt.ylabel('mm')
    else:
        plt.ylabel('deg')

