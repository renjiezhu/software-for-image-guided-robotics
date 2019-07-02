#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"

// #include "software_interface/worldToBase.h"

bool worldToBase(geometry_msgs::QuaternionPtr quaternion) {
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "worldToBase");

    // handle any argument errors

    ros::NodeHandle n;

    // ros::ServiceClient client = n.serviceClient<software_interface::worldToBase>("worldToBase");

    return 0;
}
