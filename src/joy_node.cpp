#include "joy_node.h"

#define MAX_LIN_VEL 0.25
#define MAX_ANG_VEL 1.0

joy_handler::joy_handler()
{
    joy_sub  = nodeh.subscribe<sensor_msgs::Joy>("/joy", 1, &joy_handler::joy_receive, this);
    twist_pub = nodeh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    twist.linear.x=0;
    twist.linear.y=0;
    twist.linear.z=0;
    twist.angular.x=0;
    twist.angular.y=0;
    twist.angular.z=0;
}

void joy_handler::joy_receive(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
    if(dual_mode)
    {
	twist.linear.x = MAX_LIN_VEL*joy_msg->axes.at(1);
	twist.angular.z = MAX_ANG_VEL*joy_msg->axes.at(2);
    }
    else
    {
	twist.linear.x = MAX_LIN_VEL*joy_msg->axes.at(1);
	twist.angular.z = MAX_ANG_VEL*joy_msg->axes.at(0);
    }
    
    if(joy_msg->buttons.at(0)) dual_mode=!(dual_mode);

    twist_pub.publish(twist);
}

joy_handler::~joy_handler()
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_node");

    joy_handler joy_h;

    ROS_INFO("Joy Handler Started");

    ros::spin();
}