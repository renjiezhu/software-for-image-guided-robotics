from pyrep.robots.robot_component import RobotComponent
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.backend import sim
from pyrep.backend import utils
from pyrep.backend import simConst
import numpy as np
        

class CtRobot(RobotComponent):
    """
    Robot class representing a robot arm with based on Vrep model and PyRep.

    Containing handle of joints, arms, COMs, DH_frames, ik_target, ik_tip, needle_tip.
    """

    def __init__(self, count: int = 0, name: str = 'ct_robot', num_joints: int = 8,
                 base_name: str = None, joint_type : list = ['p', 'p', 'p', 'r', 'r', 'r', 'r', 'p'], 
                 link : list = []):
        """Count is used for when we have multiple copies of arms"""
        suffix = '' if count == 0 else '#%d' % (count - 1)

        self.name = name
        self.joint_type = joint_type
        joint_names = []
        for i in range(num_joints):
            if joint_type[i] == 'p':
                joint_names.append('j%d_%s' % (i+1, 'prismatic'))
            elif joint_type[i] == 'r':
                joint_names.append('j%d_%s' % (i+1, 'revolute'))
        super().__init__(count, name, joint_names, base_name)

        part_names = ['arm%d_dynamic' % i for i in range(num_joints+1)]
        self.arms = [Shape(pname + suffix) for pname in part_names]
        # com_names = ['COM_arm%d' % i for i in range(num_joints+1)]
        # self.COMs = [Dummy(com + suffix) for com in com_names]
        # DH_frame_names = ['DH_frame_j%d' % i for i in range(num_joints+1)]
        # self.DH_frames = [Dummy(dhfname + suffix) for dhfname in DH_frame_names]

        # Motion planning handles
        self._ik_target = Dummy('target'+suffix)
        self._ik_tip = Dummy('tip'+suffix)
        self._ik_needle = Dummy('tip'+suffix)
        # self._ik_group = vrep.simGetIkGroupHandle(name+'_ik'+suffix)
        # self._collision_collection = vrep.simGetCollectionHandle(name+'_arm'+suffix)

        
    '''
    The following function are meant to implement some Regular API of VREP through PyRep.
    They are implemented by using 'script_call' function of PyRep and writing Lua code in 
    corresponding Vrep model. 

    Some of the functions are stilling under testing and debugging.
    USE WITH CAUTION!!!!
    '''   

    def getJacobian(self):
        '''Retrive Jacobian matrix from V-REP'''
        Jsize, J, _, _ = utils.script_call('getJacobian@ct_robot', simConst.sim_scripttype_childscript, ints=[], floats=[], strings=[])
        J.reverse()
        return np.array(J).reshape(tuple(Jsize))

    def setMassAndInertia(self, handle, mass, inertia, centerofmass):
        '''Set mass, inertia and center of mass in V-REP model'''
        i1,i2,i3,i4,i5,i6,i7,i8,i9 = inertia
        c1,c2,c3 = centerofmass
        o, f, s, _ = utils.script_call('setMassAndInertia@ct_robot', simConst.sim_scripttype_childscript, ints=[handle], 
        floats=[mass,i1,i2,i3,i4,i5,i6,i7,i8,i9,c1,c2,c3], strings=[])
        
    def getMassAndInertia(self, handle):
        '''Get mass, inertia and center of mass in V-REP model with respect to World Frame'''
        _, floatOutput, _ = utils.script_call('getMassAndInertia@ct_robot', simConst.sim_scripttype_childscript, ints=[handle], floats=[], strings=[])
        mass = floatOutput[0]
        inertia = np.array(floatOutput[1]).reshape(3,3)
        centerofmass = floatOutput[2]
        return mass, inertia, centerofmass

    def saveScene(self, filename):
        '''Save scene'''
        _, _, _, _ = utils.script_call('saveScene@ct_robot', simConst.sim_scripttype_childscript, ints=[], floats=[], strings=[filename])
