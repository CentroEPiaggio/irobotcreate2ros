#include "joy_node.h"

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
    twist.linear.x = 0.5*joy_msg->axes.at(1);
    twist.angular.z = 0.5*joy_msg->axes.at(0);

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