/*
 * Software Interface for Image Guided Robotics
 * 
 * Needle insertion control listenning
 * 
 * By Renjie Zhu (rezhu@eng.ucsd.edu)
 * 
 * July 8rd, 2019
 * 
 */

#include "ros/ros.h"
#include "std_msgs/Float64.h"

void keyboardCallback(const std_msgs::Float64ConstPtr &input)
{
    ROS_INFO("Needle insert position: %1d", (int)input->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "needle_insertion_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("needle_insertion", 1, keyboardCallback);

    ros::spin();

    return 0;
}