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
#include "software_interface/keyboard_input.h"
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
    double x, y, z, roll, pitch, yaw, z_needle;

    ros::Publisher robot_pos_pub;
};

// ctor
RobotControlKey::RobotControlKey()
    : x(0), y(0), z(0), roll(0), pitch(0), yaw(0), z_needle(0)
{

    nh.param("x", x, x);
    nh.param("y", y, y);
    nh.param("z", z, z);
    nh.param("roll", roll, roll);
    nh.param("pitch", pitch, pitch);
    nh.param("yaw", yaw, yaw);
    nh.param("z_needle", z_needle, z_needle);

    robot_pos_pub = nh.advertise<software_interface::keyboard_input>("keyboard_input", 1);
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
            --x;
            ROS_DEBUG("X - ,   x=%1d", (int) x);
            dirty = true;
            break;
        case KEYCODE_R:
            ++x;
            ROS_DEBUG("X + ,   x=%1d", (int) x);
            dirty = true;
            break;
        case KEYCODE_U:
            ++y;
            ROS_DEBUG("Y + ,   y=%1d", (int) y);
            dirty = true;
            break;
        case KEYCODE_D:
            --y;
            ROS_DEBUG("Y - ,   y=%1d", (int) y);
            dirty = true;
            break;
        case 'i': case 'I':
            ++z_needle;
            ROS_DEBUG("INS,    n=%1d", (int) z_needle);
            dirty = true;
            break;
        case 'o': case 'O':
            --z_needle;
            ROS_DEBUG("EXT,    n=%1d", (int) z_needle);
            dirty = true;
            break;
        case 'z': case 'Z':
            ++z;
            ROS_DEBUG("Z + ,   z=%1d", (int) z);
            dirty = true;
            break;
        case 'x': case 'X':
            --z;
            ROS_DEBUG("Z - ,   z=%1d", (int) z);
            dirty = true;
            break;
        case 'r': case 'R':
            ++roll;
            ROS_DEBUG("R + ,   r=%1d", (int) roll);
            dirty = true;
            break;
        case 'f': case 'F':
            --roll;
            ROS_DEBUG("R - ,   r=%1d", (int) roll);
            dirty = true;
            break;
        case 't': case 'T':
            ++pitch;
            ROS_DEBUG("P + ,   p=%1d", (int) pitch);
            dirty = true;
            break;
        case 'g': case 'G':
            --pitch;
            ROS_DEBUG("P - ,   p=%1d", (int) pitch);
            dirty = true;
            break;
        case 'y': case 'Y':
            ++yaw;
            ROS_DEBUG("W + ,   w=%1d", (int) yaw);
            dirty = true;
            break;
        case 'h': case 'H':
            --yaw;
            ROS_DEBUG("W - ,   w=%1d", (int) yaw);
            dirty = true;
            break;
        }

        software_interface::keyboard_input input;
        input.robot.linear.x = x;
        input.robot.linear.y = y;
        input.robot.linear.z = z;
        input.robot.angular.x = roll;
        input.robot.angular.y = pitch;
        input.robot.angular.z = yaw;
        input.z_needle = z_needle;

        if (dirty == true)
        {
            robot_pos_pub.publish(input);
            dirty = false;
        }
    }

    return;
}