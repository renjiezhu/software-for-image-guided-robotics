from pyrep.robots.arms.arm import Arm
from pyrep.const import JointType, JointMode
from pyrep.robots.robot_component import RobotComponent
from pyrep.objects.dummy import Dummy


class CtRobot(Arm):

    def __init__(self, count: int = 0, name: str = 'ct_robot', num_joints: int = 8,
                 base_name: str = None,
                 max_velocity=1.0, max_acceleration=4.0, max_jerk=1000):
        
        super().__init__(count, name, num_joints, base_name, max_velocity, max_acceleration, max_jerk)
        suffix = '' if count == 0 else '#%d' % (count - 1)
        
        # Arm part that is dynamical
        # part_names = []
        # for i in range(num_joints):
            # if self.joints[i].get_joint_type() == JointType.PRISMATIC:
                # part_names.append('j%d_%slink_dynamic' % (i+1, 'prismatic'))
            # if self.joints[i].get_joint_type() == JointType.REVOLUTE:
                # part_names.append('j%d_%slink_dynamic' % (i+1, 'rotary'))
                
        # self.parts = [RobotComponent(count, name, pname + suffix) for pname in part_names]
        
        # Reference frame for each dynamic part
        frame_names = ['reference_frame_j%d' % (i+1) for i in range(num_joints)]
        self.frames = [Dummy(fname + suffix) for fname in frame_names]

        