#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class joy_handler
{
public:
  joy_handler();
  ~joy_handler();
private:
  ros::NodeHandle nodeh;
  ros::Subscriber joy_sub;
  ros::Publisher twist_pub;
  geometry_msgs::Twist twist;
  bool dual_mode=true;

  void joy_receive(const sensor_msgs::Joy::ConstPtr& joy_msg);
};