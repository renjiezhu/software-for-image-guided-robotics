/*
 * Software Interface for Image Guided Robotics
 * 
 * Keyboard control 
 * 
 * Topics Published:
 * 
 * 
 * Topics Subscribed:
 * 
 * 
 * By Renjie Zhu (rezhu@eng.ucsd.edu)
 * 
 * Oct 10th, 2019
 * 
 */


#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "signal.h"
#include "termios.h"
#include "stdio.h"
#include <vector>

#define LINEAR_INCRE 0.005
#define ROTATE_INCRE 0.0872665

// robot status

class RobotTele
{

public:
    RobotTele();
    void keyLoop();
    
private:
    ros::NodeHandle nh;

    std::vector<double> joints;
    // double j1, j2, j3, j4, j5, j6, j7, j8;

    ros::Publisher joint_pub;

};

// ctor
RobotTele::RobotTele() : joints(8, 0.0)
{
    joint_pub = nh.advertise<sensor_msgs::JointState>("testing", 1);
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

void RobotTele::keyLoop()
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
            case '1':
                joints[0] += LINEAR_INCRE;
                dirty = true;
                break;
            case 'q':
            case 'Q':
                joints[0] -= LINEAR_INCRE;
                dirty = true;
                break;
            case '2':
                joints[1] += LINEAR_INCRE;
                dirty = true;
                break;
            case 'w':
            case 'W':
                joints[1] -= LINEAR_INCRE;
                dirty = true;
                break;
            case '3':
                joints[2] += LINEAR_INCRE;
                dirty = true;
                break;
            case 'e':
            case 'E':
                joints[2] -= LINEAR_INCRE;
                dirty = true;
                break;
            case '4':
                joints[3] += ROTATE_INCRE;
                dirty = true;
                break;
            case 'r':
            case 'R':
                joints[3] -= ROTATE_INCRE;
                dirty = true;
                break;
            case '5':
                joints[4] += ROTATE_INCRE;
                dirty = true;
                break;
            case 't':
            case 'T':
                joints[4] -= ROTATE_INCRE;
                dirty = true;
                break;
            case '6':
                joints[5] += ROTATE_INCRE;
                dirty = true;
                break;
            case 'y':
            case 'Y':
                joints[5] -= ROTATE_INCRE;
                dirty = true;
                break;
            case '7':
                joints[6] += ROTATE_INCRE;
                dirty = true;
                break;
            case 'u':
            case 'U':
                joints[6] -= ROTATE_INCRE;
                dirty = true;
                break;
            case '8':
                joints[7] += ROTATE_INCRE;
                dirty = true;
                break;
            case 'i':
            case 'I':
                joints[7] -= ROTATE_INCRE;
                dirty = true;
                break;
            default:
                break;
        }

        sensor_msgs::JointState input;
        input.position.resize(8);
        for (size_t i=0 ; i<8 ; ++i)
        {
            input.position[i] = joints[i];
        }

        if (dirty)
        {
            joint_pub.publish(input);
            dirty = false;
            
            // for (auto ele : joints)
            // {
            //     std::cout << ele << std::endl;
            // }
        }


    }

    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tele_test");

    RobotTele robotTele;

    signal(SIGINT, quit);

    robotTele.keyLoop();

    return 0;
}