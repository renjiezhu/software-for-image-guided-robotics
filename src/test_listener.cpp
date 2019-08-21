/*
 * Software Interface for Image Guided Robotics
 * 
 * Needle insertion control listening
 * 
 * By Renjie Zhu (rezhu@eng.ucsd.edu)
 * 
 * July 8th, 2019
 * 
 */

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "software_interface/Torque.h"

void keyboardCallback(const software_interface::TorqueConstPtr &input)
{
    ROS_INFO(tostring(input->joint0.data));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("torque", 1, keyboardCallback);

    ros::spin();

    return 0;
}