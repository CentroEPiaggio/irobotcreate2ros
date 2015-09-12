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
* Author: Gonçalo Cabrita on 05/10/2010
*********************************************************************/
#define NODE_VERSION 2.01

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>				// odom
#include <geometry_msgs/Twist.h>			// cmd_vel
#include <roomba_500_series/Battery.h>		// battery
#include <roomba_500_series/Bumper.h>		// bumper
#include <roomba_500_series/Buttons.h>		// buttons
#include <roomba_500_series/RoombaIR.h>		// ir_bumper cliff
#include <roomba_500_series/IRCharacter.h>	// ir_character
#include <roomba_500_series/WheelDrop.h>	// wheel_drop
#include <roomba_500_series/Leds.h>			// leds
#include <roomba_500_series/DigitLeds.h>	// digit_leds
#include <roomba_500_series/Song.h>			// song
#include <roomba_500_series/PlaySong.h>		// play_song

#include "roomba_500_series/OpenInterface.h"

#include "roomba_500_series/GoDock.h"	// GoDock action

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

void ledsReceived(const roomba_500_series::Leds::ConstPtr& leds)
{
	roomba->setLeds(leds->warning, leds->dock, leds->spot, leds->dirt_detect, leds->clean_color, leds->clean_intensity);
}

void digitLedsReceived(const roomba_500_series::DigitLeds::ConstPtr& leds)
{
	if(leds->digits.size()!=4) return;

	roomba->setDigitLeds(leds->digits[3], leds->digits[2], leds->digits[1], leds->digits[0]);
}

void songReceived(const roomba_500_series::Song::ConstPtr& song)
{
	unsigned char notes[song->notes.size()];
	unsigned char lengths[song->notes.size()];

	for(int i=0 ; i<song->notes.size() ; i++)
	{
		notes[i] = song->notes[i].note;
		lengths[i] = song->notes[i].length;
	}

	roomba->setSong(song->song_number, song->notes.size(), notes, lengths);
}

void playSongReceived(const roomba_500_series::PlaySong::ConstPtr& song)
{
	roomba->playSong(song->song_number);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "roomba560_node");

	ROS_INFO("Roomba for ROS %.2f", NODE_VERSION);
	
	double last_x, last_y, last_yaw;
	double vel_x, vel_y, vel_yaw;
	double dt;
	float last_charge = 0.0;
	int time_remaining = -1;
	
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	pn.param<std::string>("port", port, "/dev/ttyUSB0");
	
	std::string base_frame_id;
	std::string odom_frame_id;
	pn.param<std::string>("base_frame_id", base_frame_id, "base_link");
	pn.param<std::string>("odom_frame_id", odom_frame_id, "odom");
	
	roomba = new irobot::OpenInterface(port.c_str());

	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
	ros::Publisher battery_pub = n.advertise<roomba_500_series::Battery>("/battery", 50);
	ros::Publisher bumper_pub = n.advertise<roomba_500_series::Bumper>("/bumper", 50);
	ros::Publisher buttons_pub = n.advertise<roomba_500_series::Buttons>("/buttons", 50);
	ros::Publisher cliff_pub = n.advertise<roomba_500_series::RoombaIR>("/cliff", 50);
	ros::Publisher irbumper_pub = n.advertise<roomba_500_series::RoombaIR>("/ir_bumper", 50);
	ros::Publisher irchar_pub = n.advertise<roomba_500_series::IRCharacter>("/ir_character", 50);
	ros::Publisher wheeldrop_pub = n.advertise<roomba_500_series::WheelDrop>("/wheel_drop", 50);

	tf::TransformBroadcaster tf_broadcaster;
	
	ros::Subscriber cmd_vel_sub  = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelReceived);
	ros::Subscriber leds_sub  = n.subscribe<roomba_500_series::Leds>("/leds", 1, ledsReceived);
	ros::Subscriber digitleds_sub  = n.subscribe<roomba_500_series::DigitLeds>("/digit_leds", 1, digitLedsReceived);
	ros::Subscriber song_sub  = n.subscribe<roomba_500_series::Song>("/song", 1, songReceived);
	ros::Subscriber playsong_sub  = n.subscribe<roomba_500_series::PlaySong>("/play_song", 1, playSongReceived);
	
	irobot::OI_Packet_ID sensor_packets[1] = {irobot::OI_PACKET_GROUP_100};
	roomba->setSensorPackets(sensor_packets, 1, OI_PACKET_GROUP_100_SIZE);

	if( roomba->openSerialPort(true) == 0) ROS_INFO("Connected to Roomba.");
	else
	{
		ROS_FATAL("Could not connect to Roomba.");
		ROS_BREAK();
	}
	
	GoDockAction go_dock("/godock");

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate r(10.0);
	while(n.ok())
	{
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
		
		// ******************************************************************************************
		//first, we'll publish the transforms over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = odom_frame_id;
		odom_trans.child_frame_id = base_frame_id;
		odom_trans.transform.translation.x = roomba->odometry_x_;
		odom_trans.transform.translation.y = roomba->odometry_y_;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(roomba->odometry_yaw_);
		tf_broadcaster.sendTransform(odom_trans);
		
		//TODO: Finish brodcasting the tf for all the ir sensors on the Roomba
		/*geometry_msgs::TransformStamped cliff_left_trans;
		cliff_left_trans.header.stamp = current_time;
		cliff_left_trans.header.frame_id = "base_link";
		cliff_left_trans.child_frame_id = "base_cliff_left";
		cliff_left_trans.transform.translation.x = 0.0;
		cliff_left_trans.transform.translation.y = 0.0;
		cliff_left_trans.transform.translation.z = 0.0;
		cliff_left_trans.transform.rotation = ;
		tf_broadcaster.sendTransform(cliff_left_trans);	*/

		// ******************************************************************************************
		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = odom_frame_id;
		
		//set the position
		odom.pose.pose.position.x = roomba->odometry_x_;
		odom.pose.pose.position.y = roomba->odometry_y_;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(roomba->odometry_yaw_);
		
		//set the velocity
		odom.child_frame_id = base_frame_id;
		odom.twist.twist.linear.x = vel_x;
		odom.twist.twist.linear.y = vel_y;
		odom.twist.twist.angular.z = vel_yaw;
		
		//publish the message
		odom_pub.publish(odom);

		// ******************************************************************************************
		//publish battery
		roomba_500_series::Battery battery;
		battery.header.stamp = current_time;
		battery.power_cord = roomba->power_cord_;
		battery.dock = roomba->dock_;
		battery.level = 100.0*(roomba->charge_/roomba->capacity_);
		if(last_charge > roomba->charge_) time_remaining = (int)(battery.level/((last_charge-roomba->charge_)/roomba->capacity_)/dt)/60;
		last_charge = roomba->charge_;
		battery.time_remaining = time_remaining;
		battery_pub.publish(battery);
	
		// ******************************************************************************************	
		//publish bumpers
		roomba_500_series::Bumper bumper;
		bumper.left.header.stamp = current_time;
		bumper.left.state = roomba->bumper_[LEFT];
		bumper.right.header.stamp = current_time;
		bumper.right.state = roomba->bumper_[RIGHT];
		bumper_pub.publish(bumper);
	
		// ******************************************************************************************	
		//publish buttons
		roomba_500_series::Buttons buttons;
		buttons.header.stamp = current_time;
		buttons.clean = roomba->buttons_[BUTTON_CLEAN];
		buttons.spot = roomba->buttons_[BUTTON_SPOT];
		buttons.dock = roomba->buttons_[BUTTON_DOCK];
		buttons.day = roomba->buttons_[BUTTON_DAY];
		buttons.hour = roomba->buttons_[BUTTON_HOUR];
		buttons.minute = roomba->buttons_[BUTTON_MINUTE];
		buttons.schedule = roomba->buttons_[BUTTON_SCHEDULE];
		buttons.clock = roomba->buttons_[BUTTON_CLOCK];
		buttons_pub.publish(buttons);

		// ******************************************************************************************
		//publish cliff
		roomba_500_series::RoombaIR cliff;
		cliff.header.stamp = current_time;

		cliff.header.frame_id = "base_cliff_left";
		cliff.state = roomba->cliff_[LEFT];
		cliff.signal = roomba->cliff_signal_[LEFT];
		cliff_pub.publish(cliff);

		cliff.header.frame_id = "base_cliff_front_left";
		cliff.state = roomba->cliff_[FRONT_LEFT];
		cliff.signal = roomba->cliff_signal_[FRONT_LEFT];
		cliff_pub.publish(cliff);

		cliff.header.frame_id = "base_cliff_front_right";
		cliff.state = roomba->cliff_[FRONT_RIGHT];
		cliff.signal = roomba->cliff_signal_[FRONT_RIGHT];
		cliff_pub.publish(cliff);

		cliff.header.frame_id = "base_cliff_right";
		cliff.state = roomba->cliff_[RIGHT];
		cliff.signal = roomba->cliff_signal_[RIGHT];
		cliff_pub.publish(cliff);

		// ******************************************************************************************
		//publish irbumper
		roomba_500_series::RoombaIR irbumper;
		irbumper.header.stamp = current_time;

		irbumper.header.frame_id = "base_irbumper_left";
		irbumper.state = roomba->ir_bumper_[LEFT];
		irbumper.signal = roomba->ir_bumper_signal_[LEFT];
		irbumper_pub.publish(irbumper);

		irbumper.header.frame_id = "base_irbumper_front_left";
		irbumper.state = roomba->ir_bumper_[FRONT_LEFT];
		irbumper.signal = roomba->ir_bumper_signal_[FRONT_LEFT];
		irbumper_pub.publish(irbumper);

		irbumper.header.frame_id = "base_irbumper_center_left";
		irbumper.state = roomba->ir_bumper_[CENTER_LEFT];
		irbumper.signal = roomba->ir_bumper_signal_[CENTER_LEFT];
		irbumper_pub.publish(irbumper);

		irbumper.header.frame_id = "base_irbumper_center_right";
		irbumper.state = roomba->ir_bumper_[CENTER_RIGHT];
		irbumper.signal = roomba->ir_bumper_signal_[CENTER_RIGHT];
		irbumper_pub.publish(irbumper);

		irbumper.header.frame_id = "base_irbumper_front_right";
		irbumper.state = roomba->ir_bumper_[FRONT_RIGHT];
		irbumper.signal = roomba->ir_bumper_signal_[FRONT_RIGHT];
		irbumper_pub.publish(irbumper);

		irbumper.header.frame_id = "base_irbumper_right";
		irbumper.state = roomba->ir_bumper_[RIGHT];
		irbumper.signal = roomba->ir_bumper_signal_[RIGHT];
		irbumper_pub.publish(irbumper);

		// ******************************************************************************************
		//publish irchar
		roomba_500_series::IRCharacter irchar;
		irchar.header.stamp = current_time;
		irchar.omni = roomba->ir_char_[OMNI];
		irchar.left = roomba->ir_char_[LEFT];
		irchar.right = roomba->ir_char_[RIGHT];
		irchar_pub.publish(irchar);

		// ******************************************************************************************
		//publish wheeldrop
		roomba_500_series::WheelDrop wheeldrop;
		wheeldrop.left.header.stamp = current_time;
		wheeldrop.left.state = roomba->wheel_drop_[LEFT];
		wheeldrop.right.header.stamp = current_time;
		wheeldrop.right.state = roomba->wheel_drop_[RIGHT];
		wheeldrop_pub.publish(wheeldrop);
		
		ros::spinOnce();
		r.sleep();
	}
	
	roomba->powerDown();
	roomba->closeSerialPort();
}

// EOF
