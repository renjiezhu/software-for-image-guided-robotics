import sys,os
import rospy
from geometry_msgs.msg import Twist

 
"""
Walk through a set of points and repeat
"""
def publisher():
    pub = rospy.Publisher("/vrep_IK/robot_confirmed_pose", Twist, queue_size=1)
    rospy.init_node("fixed_pose_sender", anonymous=True)
    rate = rospy.Rate(100)

    # find a path to take: a collection of points
    path = []


    it = 0
    loopcount = 0

    while not rospy.is_shutdown():
        
        

        it += 1
        if it > len(path):
            loopcount += 1
            it = 0


    pass



if __name__=="__main__":
    pass
