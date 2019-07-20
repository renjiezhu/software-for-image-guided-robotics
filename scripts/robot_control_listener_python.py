#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import sys
sys.path.append("/home/renjie/Documents/igr/src/software_interface/")
# sys.path.append("/home/acrmri/homesoftware_interface/")
# from pyrep import PyRep
# from vrep_robot_control.ct_robot_control import IK_via_vrep
# from vrep_robot_control.arm import CtRobot


class RobotState:
    """
    A class containing the current robot state
    """


    def __init__(self):

        self.pos = np.array([0.0011, -0.6585, 0.2218])
        self.ori = np.zeros(3)

        self.needle_pos = 0.0

        self.dirty = False
        self.needle_dirty = False

        # self.pr = PyRep()

        # self.pr.launch("/home/renjie/Documents/igr/src/software_interface/vrep_robot_control/ct_robot_realigned.ttt")
        # self.dt = 0.01
        # self.pr.set_simulation_timestep(self.dt)
        # self.pr.start()

        # self.ct_robot = CtRobot()

    # def shutdown_vrep(self):
    #     self.pr.stop()
    #     print("V-REP shutting down.")
    #     self.pr.shutdown()
    #     print("DONE")


    def needle_retracted(self):
        """
        return if the needle is retracted
        """
        return self.needle_pos < 1e-5 and self.needle_pos >= 0

    def robot_pos_callback(self, data):
        """
        update robot pos and ori with given keyboard instructions
        """
        
        if self.needle_retracted():

            self.pos[0] -= data.linear.y / 1000
            self.pos[1] += data.linear.x / 1000
            self.pos[2] += data.linear.z / 1000

            self.ori[0] -= data.angular.y * np.pi / 180
            self.ori[1] += data.angular.x * np.pi / 180
            self.ori[2] += data.angular.z * np.pi / 180

            self.dirty = True

            rospy.loginfo(self.pos)
            rospy.loginfo(self.ori)
            
            # self.update_vrep()

        else:
            rospy.logwarn("Needle (pos=%.1f) not retracted, cannot move robot." % self.needle_pos)


    def needle_pos_callback(self, data):
        """
        update needle position with given keyboard instructions
        """

        if self.needle_retracted() and data.data < 0:
            rospy.logwarn("Needle fully retracted.")
        else: 
            self.needle_pos += data.data
            self.needle_dirty = True
            rospy.loginfo(self.needle_pos)

            # self.update_vrep()


    def update_state(self):
        """
        subscribe to keyboard published to update the robot state
        """
        rospy.Subscriber("needle_insertion", Float64, self.needle_pos_callback)
        rospy.Subscriber("robot_movement", Twist, self.robot_pos_callback)

        # try:
        #     rospy.on_shutdown(self.shutdown_vrep)
        # finally:
        #     self.shutdown_vrep()

        rospy.spin()


    def update_vrep(self):
        """
        update vrep for ik
        """
        if self.dirty:
            # IK_via_vrep(self.ct_robot, self.pos.tolist(), self.ori.tolist(), self.pr, self.dt)
            self.dirty=False
            rospy.loginfo("updating robot position")
        elif self.needle_dirty:
            self.needle_dirty=False
            rospy.loginfo("updating needle insertion")
        else:
            pass
        




if __name__ == "__main__":

    rospy.init_node("robot_control_listener_python", anonymous=True)

    robot = RobotState()

    robot.update_state()

