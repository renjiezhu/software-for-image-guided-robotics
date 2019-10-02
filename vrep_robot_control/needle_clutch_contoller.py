import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray


class ClutchController:

    def __init__(self):
        self.thre = 34          # The max distance that slider(the last prismatic joint) 
                                # can move each time (mm).

        self.desire = Float32MultiArray() # [inner_clutch, outer_clutch, distance to be moved]
                                          # 0 represents lossen, and 1 represents tighten
        self.forward = True
        rospy.init_node("clutch_controller", anonymous=True)
        rospy.loginfo("Clutch controller loaded ...")
        self.pub = rospy.Publisher("clutch_desire", Float32MultiArray, queue_size=1)


    def move(self, dist, dir, curr_pos=0):
        self.distance2move = dist
        if dir=='forward':
            self.forward = True
        else:
            self.forward = False

        if self.forward:        # needle is moving forward
            if self.distance2move>self.thre:
                while self.distance2move > self.thre:
                    self.desire.data.extend([0, 1, self.thre/1000])
                    self.disire.data.extend([1, 0, -self.thre/1000])
                    self.distance2move -= self.thre
                self.disire.data.extend([0, 1, self.distance2move/1000])
            else:
                self.desire.data = [0, 1, self.distance2move/1000]   
                self.pub.publish(self.desire)
        else:                   # needle is moving backward
            if self.distance2move<-self.thre:
                while self.distance2move > self.thre:
                    self.desire.data.extend([1, 0, -self.thre/1000])
                    self.disire.data.extend([0, 1, self.thre/1000])
                    self.distance2move += self.thre
                self.disire.data.extend([1, 0, self.distance2move/1000])
            else:
                self.desire.data = [1, 0, self.distance2move/1000]   
                self.pub.publish(self.desire)  

    

if __name__ == "__main__":
    cc = ClutchController()
    cc.move(38, 'forward')