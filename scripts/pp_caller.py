#!/usr/bin/env python3
from pyrep import PyRep
import os, sys, signal
import rospy

class PathPlanning:

    def __init__(self):
        rospy.init_node("robot_path_planning", anonymous=True)
        self.pr = PyRep()
        self.pr.launch(
            f"/home/{os.environ['USER']}/Documents/igr/src/software_interface/vrep_robot_control/CtRobot_pathplanning.ttt",
            headless=True,
        )

        self.pr.set_simulation_timestep(0.005)


    def signal_handler(self, sig, frame):
        """
        safely shutdown vrep when control C is pressed
        """
        rospy.loginfo("Calling exit for pyrep")
        self.shutdown_vrep()
        rospy.signal_shutdown("from signal_handler")

    def shutdown_vrep(self):
        """
        shutdown vrep safely
        """
        self.pr.stop()
        rospy.loginfo("V-REP shutting down.")
        self.pr.shutdown()

    def run(self):
        self.pr.start()
        signal.signal(signal.SIGINT, self.signal_handler)
        rospy.spin()

if __name__ == "__main__":

    pp = PathPlanning()
    pp.run()


