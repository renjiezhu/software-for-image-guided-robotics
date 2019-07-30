import numpy as np
from pyrep import PyRep
from arm import CtRobot
import sympy as sp
import os
import transforms3d as t3d

def getMassfromSW(file_name):
    '''Get Mass, Inertiam Center of Mass from CAD in Solidworks'''
    inertia = np.zeros((3, 3))
    centr_of_mass = np.zeros(3)
    f = open('./massAndinertia/'+file_name + '.txt')
    # Read from file
    lines = f.readlines()
    for i, line in enumerate(lines):
        if 'Mass =' in line:
            mass = float(line.split('=')[1].strip().split(' ')[0])/1000
        if 'Center of mass:' in line:
            curr = i
        if 'Moments of inertia: ' in line:
            curr_inertia = i
    for i in range(3):
        centr_of_mass[i] = lines[curr+i+1].split('=')[1].strip()
    for i in range(3):
        line = lines[curr_inertia+i+2]
        txt = line.split('=')
        for j in range(3):
            inertia[i, j] = float(txt[j+1].strip().split('\t')[0])
    # Align the base from between V-REP and Soliworks
    tmp = centr_of_mass.copy()
    centr_of_mass[2] = tmp[1]
    centr_of_mass[1] = -tmp[2]
    centr_of_mass = centr_of_mass/1000
    # Align unit and form
    inertia = inertia*1e-9/mass
    tmp = inertia.copy()
    inertia[0,0] = tmp[0,0]
    inertia[0,1] = -tmp[0,2]
    inertia[0,2] = tmp[0,1]
    inertia[1,0] = -tmp[2,0]
    inertia[1,1] = tmp[2,2]
    inertia[1,2] = -tmp[2,1]
    inertia[2,0] = tmp[1,0]
    inertia[2,1] = -tmp[1,2]
    inertia[2,2] = tmp[1,1]

    return mass, inertia, centr_of_mass


if __name__ == "__main__":
    pr = PyRep()
    pr.launch(os.getcwd() + '/DH_ct.ttt', headless=True)
    pr.start()
    ct_robot = CtRobot()

    

    names = ['arm%d' % i for i in range(ct_robot._num_joints+1)]
    for i in range(ct_robot._num_joints+1):
        handle = ct_robot.arms[i].get_handle()
        mass, inertia, centr_of_mass = getMassfromSW(names[i])
        inertia = tuple(inertia.reshape(9))
        centr_of_mass = tuple(centr_of_mass)
        ct_robot.setMassAndInertia(handle, mass, inertia, centr_of_mass)
    
    # ct_robot.saveScene('Temp.ttt')
    # print('save_complete')
    pr.shutdown()




