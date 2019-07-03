/*
 * Software Interface for Image Guided Robotics
 * 
 * World to Base Transformation, client 
 * 
 * By Renjie Zhu (rezhu@eng.ucsd.edu)
 * 
 * July 2nd, 2019
 * 
 */

#include "ros/ros.h"

#include "software_interface/world_to_base.h"

// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// #include "tf2/LinearMath/Transform.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "world_to_base");
    // handle any argument errors

    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<software_interface::world_to_base>("world_to_base");
    software_interface::world_to_base srv;
    srv.request.world_frame.loc.x = 1;
    srv.request.world_frame.ori.x = 1;
    
    ROS_INFO("World Frame: loc.x : %1d, ori.x : %1d", (int)srv.request.world_frame.loc.x, (int)srv.request.world_frame.ori.x);
    return 0;
}
