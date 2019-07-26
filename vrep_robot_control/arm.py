from pyrep.robots.robot_component import RobotComponent
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.backend import vrep
from pyrep.backend import utils
from pyrep.backend.vrepConst import *
        

class CtRobot(RobotComponent):
    """Robot class representing a robot arm with based on Vrep model.
    """

    def __init__(self, count: int = 0, name: str = 'ct_robot', num_joints: int = 7,
                 base_name: str = None, joint_type : list = ['p', 'p', 'r', 'r', 'r', 'r', 'p'], 
                 max_velocity=1.0, max_acceleration=4.0, max_jerk=1000):
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

        part_names = ['arm%d_dynamic' % (i+1) for i in range(num_joints)]
        part_names.insert(3, 'arm3_dynamic_out')
        self.arms = [Shape(pname + suffix) for pname in part_names]
        frame_names = ['reference_frame_j%d' % (i+1) for i in range(num_joints)]
        frame_names.insert(0, 'reference_frame_base')
        self.frames = [Dummy(fname + suffix) for fname in frame_names]
        DH_frame_names = ['DH_frame_j%d' % (i+1) for i in range(num_joints)]
        DH_frame_names.insert(0, 'DH_frame_base')
        self.DH_frames = [Dummy(dhfname + suffix) for dhfname in DH_frame_names]
        # self.DH_frames.append(Dummy('tip'))
        
        # Used for motion planning
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.max_jerk = max_jerk

        # Motion planning handles
        self._ik_target = Dummy('target')
        self._ik_tip = Dummy('tip_frame')
        self._ik_group = vrep.simGetIkGroupHandle(name+'_ik'+suffix)
        self._collision_collection = vrep.simGetCollectionHandle(
            name+'_arm'+suffix)

        
    def getJacobian(self, handle):
        _, _, Jacobian, _ = utils.script_call('getJacobian', sim_scripttype_childscript, ints=[handle], )
        return Jacobian
        
        
        