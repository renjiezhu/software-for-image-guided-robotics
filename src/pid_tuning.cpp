#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "signal.h"
#include "termios.h"
#include "stdio.h"
#include <vector>
#include "math.h"
#include <iostream>

#include "unistd.h"

#define L_INCRE 500
#define R_INCRE 1000
    
bool midpointReached = false;

class Demo 
{

public:
    Demo();

    void start_up();

    void publish();

    // void oscillate();

    ros::NodeHandle nh;

    void encoderReachedCallback(const sensor_msgs::JointStateConstPtr& encoder);

    ros::Rate* sensorPublisherRate;

// private:

    sensor_msgs::JointState input;

    ros::Publisher joint_pub;

};


Demo::Demo()
{
    joint_pub = nh.advertise<sensor_msgs::JointState>("testing", 1);
    sensorPublisherRate = new ros::Rate(200);
    input.position.resize(8);
}

void Demo::publish() 
{
    input.header.stamp = ros::Time::now();
    joint_pub.publish(input);
}

void Demo::encoderReachedCallback(const sensor_msgs::JointStateConstPtr& encoder)
{
    ROS_INFO("trying");
    if (abs(encoder->position[0]+360000) < 1000 && 
        abs(encoder->position[1]+360000) < 1000 && 
        abs(encoder->position[2]-620000) < 1000) 
    {
        midpointReached = true;
        ROS_INFO("REACHED");
    }
}

void Demo::start_up() 
{
    while (input.position[0] > -360000) 
    {
        input.position[0] -= L_INCRE;
        input.position[1] -= L_INCRE;
        input.position[2] += L_INCRE;
        publish();
        sensorPublisherRate->sleep();
    }

    while (input.position[2] < 620000)
    {
        input.position[2] += L_INCRE;
        publish();
        sensorPublisherRate->sleep();
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "tele_test");

    Demo demo;
    // ros::Subscriber encoder_sub = demo.nh.subscribe("encoder_main", 1, &Demo::encoderReachedCallback, &demo);
    // signal(SIGINT, quit);
    do 
    {
        std::cout << '\n' << "Press a key to continue...";
    } while (std::cin.get() != '\n');
    


    // sleep(20);


    while (ros::ok())
    {   
        demo.start_up();
        double sine_t = sin(0.5*M_PI*ros::Time::now().toSec());
        // demo.input.position[0] = sin(0.5*M_PI*ros::Time::now().toSec())*120000 - 360000;
        demo.input.position[1] = sine_t*120000 - 360000;
        demo.input.position[2] = sine_t*120000 + 620000;
        // demo.input.position[4] = sin(0.5*M_PI*ros::Time::now().toSec())*60000;
        demo.input.position[5] = sine_t*60000;
        demo.input.position[6] = sine_t*60000;
        demo.input.position[7] = sine_t*60000;
        demo.publish();

        ros::spinOnce();
        demo.sensorPublisherRate->sleep();
    }

    return 0;
}