/*
 * Software Interface for Image Guided Robotics
 * 
 * Keyboard control 
 * 
 * By Renjie Zhu (rezhu@eng.ucsd.edu)
 * 
 * July 3rd, 2019
 * 
 */

#include "ros/ros.h"
// #include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/Point.h"
#include "tf2/utils.h"
#include "signal.h"
#include "termios.h"
#include "stdio.h"

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42

// Keyboard control Class

class RobotControlKey
{

public:
    RobotControlKey();
    void keyLoop();

private:
    ros::NodeHandle nh;
    // test values
    double x, y, z;

    ros::Publisher robot_pos_pub;
};

// ctor
RobotControlKey::RobotControlKey()
    : x(0), y(0), z(0)
{

    nh.param("test_x", x, x);
    nh.param("test_y", y, y);
    nh.param("test_z", z, z);

    robot_pos_pub = nh.advertise<geometry_msgs::Point>("software_interface/keyboard_test", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
    (void)sig;
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "robot_control_key");

    RobotControlKey robotCtrl;

    signal(SIGINT, quit); // don't know what this does, should be 'quitting when ^C'

    robotCtrl.keyLoop();

    return 0;
}

void RobotControlKey::keyLoop()
{
    char c;
    bool dirty = false;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the robot.");

    while (ros::ok())
    {
        // get the next event from the keyboard
        if (read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        switch (c)
        {
        case KEYCODE_L:
            ROS_DEBUG("LEFT");
            ROS_INFO("LEFT,   x=%1d", (int)--x);
            dirty = true;
            break;
        case KEYCODE_R:
            ROS_DEBUG("RIGHT");
            ROS_INFO("RIGHT,  x=%1d", (int)++x);
            dirty = true;
            break;
        case KEYCODE_U:
            ROS_DEBUG("UP");
            ROS_INFO("UP,     y=%1d", (int)++y);
            dirty = true;
            break;
        case KEYCODE_D:
            ROS_DEBUG("DOWN");
            ROS_INFO("DOWN,   y=%1d", (int)--y);
            dirty = true;
            break;
        case 'i': case 'I':
            ROS_DEBUG("INSERT");
            ROS_INFO("INS,    z=%1d", (int)++z);
            dirty = true;
            break;
        case 'o': case 'O':
            ROS_DEBUG("EXTRACT");
            ROS_INFO("EXT,    z=%1d", (int)--z);
            dirty = true;
            break;
        }

        geometry_msgs::Point position;
        position.x = x;
        position.y = y;
        position.z = z;

        if (dirty == true)
        {
            robot_pos_pub.publish(position);
            dirty = false;
        }
    }

    return;
}