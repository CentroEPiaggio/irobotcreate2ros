/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Gonçalo Cabrita on 07/10/2010
*********************************************************************/
#define NODE_VERSION 2.01

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>			// odom
#include <geometry_msgs/Twist.h>		// cmd_vel

#include "roomba_500_series/OpenInterface.h"

#include <string>

std::string port;
irobot::OpenInterface * roomba;


std::string prefixTopic(std::string prefix, char * name)
{
	std::string topic_name = prefix;
	topic_name.append(name);
	
	return topic_name;
}

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	roomba->drive(cmd_vel->linear.x, cmd_vel->angular.z);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "roomba560_light_node");

	ROS_INFO("Roomba for ROS %.2f", NODE_VERSION);
	
	double last_x, last_y, last_yaw;
	double vel_x, vel_y, vel_yaw;
	double dt;

	ros::NodeHandle n;
	
	n.param<std::string>("roomba/port", port, "/dev/ttyUSB0");
	
	roomba = new irobot::OpenInterface(port.c_str());
	
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
	tf::TransformBroadcaster odom_broadcaster;
	ros::Subscriber cmd_vel_sub  = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelReceived);

	if( roomba->openSerialPort(true) == 0) ROS_INFO("Connected to Roomba.");
	else
	{
		ROS_FATAL("Could not connect to Roomba.");
		ROS_BREAK();
	}

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	//int heartvalue = 0;
	//bool inc = true;
	
	ros::Rate r(10.0);
	while(n.ok())
	{
		/*if(inc==true) heartvalue += 20;
		else heartvalue -= 20;
		if(heartvalue>=255)
		{
			heartvalue = 255;
			inc=false;
		}
		if(heartvalue<=0)
		{
			heartvalue = 0;
			inc=true;
		}
		roomba->setLeds(0, 0, 0, 0, (unsigned char)heartvalue, 255);*/

		current_time = ros::Time::now();
		
		last_x = roomba->odometry_x_;
		last_y = roomba->odometry_y_;
		last_yaw = roomba->odometry_yaw_;
		
		if( roomba->getSensorPackets(100) == -1) ROS_ERROR("Could not retrieve sensor packets.");
		else roomba->calculateOdometry();
		
		dt = (current_time - last_time).toSec();
		vel_x = (roomba->odometry_x_ - last_x)/dt;
		vel_y = (roomba->odometry_y_ - last_y)/dt;
		vel_yaw = (roomba->odometry_yaw_ - last_yaw)/dt;
		
		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(roomba->odometry_yaw_);
		
		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";
		
		odom_trans.transform.translation.x = roomba->odometry_x_;
		odom_trans.transform.translation.y = roomba->odometry_y_;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;
		
		//send the transform
		odom_broadcaster.sendTransform(odom_trans);
		
		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		
		//set the position
		odom.pose.pose.position.x = roomba->odometry_x_;
		odom.pose.pose.position.y = roomba->odometry_y_;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		
		//set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = vel_x;
		odom.twist.twist.linear.y = vel_y;
		odom.twist.twist.angular.z = vel_yaw;
		
		//publish the message
		odom_pub.publish(odom);

		ros::spinOnce();
		r.sleep();
	}
	
	roomba->powerDown();
	roomba->closeSerialPort();
}

// EOF
