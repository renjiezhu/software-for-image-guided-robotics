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
#include "software_interface/keyboard_input.h"

void keyboardCallback(const software_interface::keyboard_inputConstPtr &input)
{
    ROS_INFO("Robot position:\tx=%1d, y=%1d, z=%1d, \n\t\t\t\t\t\tr=%1d, p=%1d, w=%1d, \n\t\t\t\t\t\tz_needle=%1d", 
    (int)input->robot.linear.x, (int)input->robot.linear.y, (int)input->robot.linear.z,
    (int)input->robot.angular.x, (int)input->robot.angular.y, (int)input->robot.angular.z,
    (int)input->z_needle);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("keyboard_input", 1, keyboardCallback);

    ros::spin();

    return 0;
}