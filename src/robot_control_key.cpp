/*
 * Software Interface for Image Guided Robotics
 * 
 * Keyboard control 
 * 
 * Topics Published:
 * * robot_movement
 * * needle_insertion
 * * simulation_confirmation
 * 
 * Topics Subscribed:
 * 
 * 
 * By Renjie Zhu (rezhu@eng.ucsd.edu)
 * 
 * July 3rd, 2019
 * 
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
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
    void returnZero();

private:
    ros::NodeHandle nh;
    // test values
    double x, y, z, roll, pitch, yaw, z_needle;

    short confirmed;

    ros::Publisher robot_pos_pub;
    ros::Publisher needle_pub;
    ros::Publisher confirmation_pub;
};

// ctor
RobotControlKey::RobotControlKey()
    : x(0), y(0), z(0), roll(0), pitch(0), yaw(0), z_needle(0), confirmed(0)
{

    nh.param("x", x, x);
    nh.param("y", y, y);
    nh.param("z", z, z);
    nh.param("roll", roll, roll);
    nh.param("pitch", pitch, pitch);
    nh.param("yaw", yaw, yaw);
    nh.param("z_needle", z_needle, z_needle);

    robot_pos_pub = nh.advertise<geometry_msgs::Twist>("robot_movement", 1);
    needle_pub = nh.advertise<std_msgs::Float64>("needle_insertion", 1);
    confirmation_pub = nh.advertise<std_msgs::Bool>("simulation_confirmation", 1);
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
    bool needle_dirty = false;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("\nUse keyboard to move the robot.\n");
    puts("Key bindings:\n|-----------------------|");
    puts("|\t|   +\t|   -\t|");
    puts("|-------|-------|-------|");
    puts("|  \u03b4x\t|   \u2191\t|   \u2193\t|");
    puts("|  \u03b4y\t|   \u2192\t|   \u2190\t|");
    puts("|  \u03b4z\t|   z\t|   x\t|");
    puts("| \u03b4roll\t|   r\t|   f\t|");
    puts("|\u03b4pitch\t|   t\t|   g\t|");
    puts("| \u03b4yaw\t|   y\t|   h\t|");
    puts("|needle\t|   i\t|   o\t|");
    puts("|-----------------------|\n");

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
            // --x;
            x = -1;
            ROS_DEBUG("X - ,   x=%1d", (int)x);
            dirty = true;
            break;
        case KEYCODE_R:
            // ++x;
            x = 1;
            ROS_DEBUG("X + ,   x=%1d", (int)x);
            dirty = true;
            break;
        case KEYCODE_U:
            // ++y;
            y = 1;
            ROS_DEBUG("Y + ,   y=%1d", (int)y);
            dirty = true;
            break;
        case KEYCODE_D:
            // --y;
            y = -1;
            ROS_DEBUG("Y - ,   y=%1d", (int)y);
            dirty = true;
            break;
        case 'i':
        case 'I':
            // ++z_needle;
            z_needle = 1;
            ROS_DEBUG("INS,    n=%1d", (int)z_needle);
            needle_dirty = true;
            break;
        case 'o':
        case 'O':
            // --z_needle;
            z_needle = -1;
            ROS_DEBUG("EXT,    n=%1d", (int)z_needle);
            needle_dirty = true;
            break;
        case 'z':
        case 'Z':
            // ++z;
            z = 1;
            ROS_DEBUG("Z + ,   z=%1d", (int)z);
            dirty = true;
            break;
        case 'x':
        case 'X':
            // --z;
            z = -1;
            ROS_DEBUG("Z - ,   z=%1d", (int)z);
            dirty = true;
            break;
        case 'r':
        case 'R':
            // ++roll;
            roll = 1;
            ROS_DEBUG("R + ,   r=%1d", (int)roll);
            dirty = true;
            break;
        case 'f':
        case 'F':
            // --roll;
            roll = -1;
            ROS_DEBUG("R - ,   r=%1d", (int)roll);
            dirty = true;
            break;
        case 't':
        case 'T':
            // ++pitch;
            pitch = 1;
            ROS_DEBUG("P + ,   p=%1d", (int)pitch);
            dirty = true;
            break;
        case 'g':
        case 'G':
            // --pitch;
            pitch = -1;
            ROS_DEBUG("P - ,   p=%1d", (int)pitch);
            dirty = true;
            break;
        case 'y':
        case 'Y':
            // ++yaw;
            yaw = 1;
            ROS_DEBUG("W + ,   w=%1d", (int)yaw);
            dirty = true;
            break;
        case 'h':
        case 'H':
            // --yaw;
            yaw = -1;
            ROS_DEBUG("W - ,   w=%1d", (int)yaw);
            dirty = true;
            break;
        case ' ':
            // confirmation;
            confirmed++;
            if (confirmed == 1)
            {
                ROS_INFO("Press [space] again to send current pose... ");
            }
            break;
        }

        geometry_msgs::Twist input;
        input.linear.x = x;
        input.linear.y = y;
        input.linear.z = z;
        input.angular.x = roll;
        input.angular.y = pitch;
        input.angular.z = yaw;

        std_msgs::Float64 input_n;
        input_n.data = z_needle;

        std_msgs::Bool confirmation;
        confirmation.data = 1;

        if (dirty)
        {
            robot_pos_pub.publish(input);
            dirty = false;
            returnZero();
        }

        if (needle_dirty)
        {
            needle_pub.publish(input_n);
            needle_dirty = false;
            returnZero();
        }

        if (confirmed >= 2)
        {
            confirmation_pub.publish(confirmation);
            ROS_INFO("Simulation result sent. ");
            returnZero();
        }
    }

    return;
}

void RobotControlKey::returnZero()
{
    x = 0;
    y = 0;
    z = 0;
    roll = 0;
    pitch = 0;
    yaw = 0;
    z_needle = 0;
    confirmed = 0;
}