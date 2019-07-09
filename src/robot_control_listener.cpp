/*
 * Software Interface for Image Guided Robotics
 * 
 * Needle position control listening
 * 
 * By Renjie Zhu (rezhu@eng.ucsd.edu)
 * 
 * July 3rd, 2019
 * 
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

void keyboardCallback(const geometry_msgs::TwistConstPtr &input)
{
    ROS_INFO("Robot position:\tx=%1d, y=%1d, z=%1d, \n\t\t\t\t\t\tr=%1d, p=%1d, w=%1d", 
    (int)input->linear.x, (int)input->linear.y, (int)input->linear.z,
    (int)input->angular.x, (int)input->angular.y, (int)input->angular.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "needle_position_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("robot_movement", 1, keyboardCallback);

    ros::spin();

    return 0;
}