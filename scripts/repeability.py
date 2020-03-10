import sys,os
import rospy
from geometry_msgs.msg import Twist

 
"""Send a fixed EE pose"""
def publisher():
    pub = rospy.Publisher("/vrep_IK/robot_confirmed_pose", Twist, queue_size=1)
    rospy.init_node("fixed_pose_sender", anonymous=True)
    rate = rospy.Rate(100)
    

    while not rospy.is_shutdown():
        pass


    pass



if __name__=="__main__":
    pass
