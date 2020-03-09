#include "interpolation.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <vector>

interpolation helper;

void callBack(const sensor_msgs::JointStateConstPtr &data){
    helper.updateSetPoint(data->position, data->velocity);
}


int main(int args, char** argc)
{   
    ros::init(args, argc, "reflexxex_node");
    ros::NodeHandle nh;
    
    // Interpolated Setpoint
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("joint_setpoint_interpolated", 1);
    sensor_msgs::JointState setpoint_int;
    setpoint_int.position.resize(8);

    // Subscribe from upper level setpoint
    ros::Subscriber sub = nh.subscribe("joint_setpoint_clipped", 1, callBack);
    
    // ROS fequency
    ros::Rate sensorPublisherRate(1000);

    while (ros::ok())
    {
        // update interpolated setpoint and publish
        setpoint_int.position = helper.setpoint;
        setpoint_int.header.stamp = ros::Time::now();
        pub.publish(setpoint_int);

        ros::spinOnce();
        sensorPublisherRate.sleep();

    }
    return 0;
}
