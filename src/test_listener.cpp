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

void keyboardCallback(const std_msgs::BoolConstPtr &input)
{
    if (input->data) {
        ROS_INFO("Simulation Confirmed");
    } 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("simulation_confirmation", 1, keyboardCallback);

    ros::spin();

    return 0;
}