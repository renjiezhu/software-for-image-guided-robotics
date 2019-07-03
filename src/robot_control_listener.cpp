/*
 * Software Interface for Image Guided Robotics
 * 
 * Keyboard control (test listener)
 * 
 * By Renjie Zhu (rezhu@eng.ucsd.edu)
 * 
 * July 3rd, 2019
 * 
 */

#include "ros/ros.h"
#include "geometry_msgs/Point.h"

void keyboardCallback(const geometry_msgs::PointConstPtr& point) 
{
    ROS_INFO("Robot position: x=%1d, y=%1d, z=%1d", (int)point->x, (int)point->y, (int)point->z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("software_interface/keyboard_test", 1, keyboardCallback);

    ros::spin();

    return 0;
}