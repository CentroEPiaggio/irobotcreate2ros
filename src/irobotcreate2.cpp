/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, University of Pisa
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
* Author: Alessandro Settimi 2015
* Author: Mirko Ferrati 2015
*********************************************************************/

#define NODE_VERSION 2.01

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>				// odom
#include <geometry_msgs/Twist.h>			// cmd_vel
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <irobotcreate2/Battery.h>		// battery
#include <irobotcreate2/Bumper.h>		// bumper
#include <irobotcreate2/Buttons.h>		// buttons
#include <irobotcreate2/RoombaIR.h>		// ir_bumper cliff
#include <irobotcreate2/IRCharacter.h>	// ir_character
#include <irobotcreate2/WheelDrop.h>	// wheel_drop
#include <irobotcreate2/Leds.h>			// leds
#include <irobotcreate2/DigitLeds.h>	// digit_leds
#include <irobotcreate2/Song.h>			// song
#include <irobotcreate2/PlaySong.h>		// play_song

#include "irobotcreate2/OpenInterface.h"
#include "irobotcreate2/odometry.h"

#include <string>
#include <atomic>
#include <eigen3/Eigen/Eigen>
#include <XmlRpc.h>

std::string port;
irobot::OpenInterface * roomba;
std::atomic_bool ir_warning;
std::atomic_bool bumper_warning;
ros::Time last_cmd_vel(0);
ros::Duration cmd_vel_duration(0,200000000);

std::string prefixTopic(std::string prefix, char * name)
{
	std::string topic_name = prefix;
	topic_name.append(name);
	
	return topic_name;
}

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	if(bumper_warning.load()) roomba->drive(-0.2, 0);
	else if(ir_warning.load()) roomba->drive(0, cmd_vel->angular.z);
	else roomba->drive(cmd_vel->linear.x,cmd_vel->angular.z);

	last_cmd_vel = ros::Time::now();
}

void cmdSpecialReceived(const std_msgs::String::ConstPtr& cmd_)
{
    if(cmd_->data=="reset_odom") roomba->resetOdometry();
}

void cmdModeReceived(const std_msgs::String::ConstPtr& cmd_)
{
	std::string cmd = cmd_->data.c_str();

	if(cmd=="exit") return;
	else if(cmd=="start")
	{
	    roomba->Start();
	}
	else if(cmd=="stop")
	{
	    roomba->Stop();
	}
	else if(cmd=="reset")
	{
	    roomba->Reset();
	}
	else if(cmd=="powerdown")
	{
	    roomba->powerDown();
	}
	else if(cmd=="safe")
	{
	    roomba->Safe();
	}
	else if(cmd=="full")
	{
	    roomba->Full();
	}
}

void ledsReceived(const irobotcreate2::Leds::ConstPtr& leds)
{
	roomba->setLeds(leds->warning, leds->dock, leds->spot, leds->dirt_detect, leds->clean_color, leds->clean_intensity);
}

void digitLedsReceived(const irobotcreate2::DigitLeds::ConstPtr& leds)
{
	if(leds->digits.size()!=4) return;

	roomba->setDigitLeds(leds->digits[3], leds->digits[2], leds->digits[1], leds->digits[0]);
}

void songReceived(const irobotcreate2::Song::ConstPtr& song)
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

void playSongReceived(const irobotcreate2::PlaySong::ConstPtr& song)
{
	roomba->playSong(song->song_number);
}


int main(int argc, char** argv)
{
	bool desired_mode_full=false; //if false go in safe_mode
	bool ir_warning_;
	bool bumper_warning_;

	ros::init(argc, argv, "roomba560_node");

	ROS_INFO("Roomba for ROS %.2f", NODE_VERSION);
	
	double last_x, last_y, last_yaw;
	double vel_x, vel_y, vel_yaw;
	double dt;
	float last_charge = 0.0;
	int time_remaining = -1;
	
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	std::string base_name;
	int id;
	pn.param<std::string>("base_name_", base_name, "iRobot_");
	pn.param<int>("id_", id, 0);
	std_msgs::String my_namespace;
	my_namespace.data = base_name + std::to_string(id);

	bool publish_name;
	ros::Publisher listpub;
	pn.param<bool>("publish_name_", publish_name, false);
	if(publish_name)
	{
	    listpub = n.advertise<std_msgs::String>("/agent_list", 50);
	}
	int name_count=0;

	pn.param<std::string>("port_", port, "/dev/ttyUSB0");

	std::string base_frame_id;
	std::string odom_frame_id;
	pn.param<std::string>("base_frame_id", base_frame_id, my_namespace.data + "/base_link");
	pn.param<std::string>("odom_frame_id", odom_frame_id, my_namespace.data + "/odom");
    
    bool publishTf;
    pn.param<bool>("publishTf", publishTf, true);

    Eigen::MatrixXd poseCovariance(6,6);
    Eigen::MatrixXd twistCovariance(6,6);
    bool pose_cov_mat = false;
    bool twist_cov_mat = false;
    std::vector<double> pose_covariance_matrix; 
    std::vector<double> twist_covariance_matrix;
    XmlRpc::XmlRpcValue poseCovarConfig;
    XmlRpc::XmlRpcValue twistCovarConfig;
    
    if (pn.hasParam("poseCovariance"))
    {
        try
        {
            pn.getParam("poseCovariance", poseCovarConfig);

            ROS_ASSERT(poseCovarConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

            int matSize = poseCovariance.rows();

            if (poseCovarConfig.size() != matSize * matSize)
            {
                ROS_WARN_STREAM("Pose_covariance matrix should have " << matSize * matSize << " values.");
            }
            else
            {
                for (int i = 0; i < matSize; i++)
                {
                    for (int j = 0; j < matSize; j++)
                    {
                        std::ostringstream ostr;
                        ostr << poseCovarConfig[matSize * i + j];
                        std::istringstream istr(ostr.str());
                        istr >> poseCovariance(i, j);
                        pose_covariance_matrix.push_back(poseCovariance(i,j));
                    }
                }
                pose_cov_mat = true;
            }
        }
        catch (XmlRpc::XmlRpcException &e)
        {
            ROS_ERROR_STREAM("ERROR reading sensor config: " << e.getMessage() << " for pose_covariance (type: " << poseCovarConfig.getType() << ")");
        }
        
    }
    
    if (pn.hasParam("twistCovariance"))
    {
        try
        {
            pn.getParam("twistCovariance", twistCovarConfig);

            ROS_ASSERT(twistCovarConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

            int matSize = twistCovariance.rows();

            if (twistCovarConfig.size() != matSize * matSize)
            {
                ROS_WARN_STREAM("Twist_covariance matrix should have " << matSize * matSize << " values.");
            }
            else
            {
                for (int i = 0; i < matSize; i++)
                {
                    for (int j = 0; j < matSize; j++)
                    {
                        std::ostringstream ostr;
                        ostr << twistCovarConfig[matSize * i + j];
                        std::istringstream istr(ostr.str());
                        istr >> twistCovariance(i, j);
                        twist_covariance_matrix.push_back(twistCovariance(i,j));
                    }
                }
                twist_cov_mat = true;
            }
        }
        catch (XmlRpc::XmlRpcException &e)
        {
            ROS_ERROR_STREAM("ERROR reading sensor config: " << e.getMessage() << " for twist_covariance (type: " << poseCovarConfig.getType() << ")");
        }
    }
    
	roomba = new irobot::OpenInterface(port.c_str());

	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Publisher battery_pub = n.advertise<irobotcreate2::Battery>("battery", 50);
    ros::Publisher bumper_pub = n.advertise<irobotcreate2::Bumper>("bumper", 50);
    ros::Publisher buttons_pub = n.advertise<irobotcreate2::Buttons>("buttons", 50);
    ros::Publisher cliff_pub = n.advertise<irobotcreate2::RoombaIR>("cliff", 50);
    ros::Publisher irbumper_pub = n.advertise<irobotcreate2::RoombaIR>("ir_bumper", 50);
    ros::Publisher irchar_pub = n.advertise<irobotcreate2::IRCharacter>("ir_character", 50);
    ros::Publisher wheeldrop_pub = n.advertise<irobotcreate2::WheelDrop>("wheel_drop", 50);

	tf::TransformBroadcaster tf_broadcaster;
	
	ros::Subscriber cmd_vel_sub  = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, cmdVelReceived);
    ros::Subscriber leds_sub  = n.subscribe<irobotcreate2::Leds>("leds", 1, ledsReceived);
    ros::Subscriber digitleds_sub  = n.subscribe<irobotcreate2::DigitLeds>("digit_leds", 1, digitLedsReceived);
    ros::Subscriber song_sub  = n.subscribe<irobotcreate2::Song>("song", 1, songReceived);
    ros::Subscriber playsong_sub  = n.subscribe<irobotcreate2::PlaySong>("play_song", 1, playSongReceived);
    ros::Subscriber mode_sub  = n.subscribe<std_msgs::String>("mode", 1, cmdModeReceived);
    ros::Subscriber special_sub  = n.subscribe<std_msgs::String>("special", 1, cmdSpecialReceived);
	
	irobot::OI_Packet_ID sensor_packets[1] = {irobot::OI_PACKET_GROUP_100};
	roomba->setSensorPackets(sensor_packets, 1, OI_PACKET_GROUP_100_SIZE);

	if( roomba->openSerialPort(true) == 0) ROS_INFO("Connected to Roomba.");
	else
	{
		ROS_FATAL("Could not connect to Roomba.");
		ROS_BREAK();
	}
	
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	bool first_loop=true;

    while(roomba->getSensorPackets(100) == -1 && ros::ok())
    {
        usleep(100);
        ROS_INFO_STREAM("Waiting for roomba sensors");
    }
    roomba->calculateOdometry();
    roomba->resetOdometry();

    
    
	ros::Rate r(50.0);
	while(n.ok())
	{
		if(ros::Time::now() - last_cmd_vel > cmd_vel_duration)
		{
		    roomba->drive(0,0);
		}

	        if(publish_name)
		{
		    name_count++;
		    if(name_count >= 10)
		    {
			listpub.publish(my_namespace);
			name_count=0;
		    }
		}

		ir_warning_ = false;
		bumper_warning_ = false;
		current_time = ros::Time::now();
		
		last_x = roomba->odometry_x_;
		last_y = roomba->odometry_y_;
		last_yaw = roomba->odometry_yaw_;
		
		if( roomba->getSensorPackets(100) == -1) ROS_ERROR("Could not retrieve sensor packets.");
		else roomba->calculateOdometry();
		
		/* OLD CODE
         * dt = (current_time - last_time).toSec();
		vel_x = (roomba->odometry_x_ - last_x)/dt;
		vel_y = (roomba->odometry_y_ - last_y)/dt;
		vel_yaw = (roomba->odometry_yaw_ - last_yaw)/dt;*/
        
        vel_x = roomba->new_odometry_.getLinear();
        vel_y = 0.0;
        vel_yaw = roomba->new_odometry_.getAngular();
		
		// ******************************************************************************************
		//first, we'll publish the transforms over tf
        if(publishTf)
        {
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = odom_frame_id;
            odom_trans.child_frame_id = base_frame_id;
            odom_trans.transform.translation.x = roomba->odometry_x_;
            odom_trans.transform.translation.y = roomba->odometry_y_;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(roomba->odometry_yaw_);
            tf_broadcaster.sendTransform(odom_trans);
        }
		
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
        if(pose_cov_mat)
            for(int i = 0; i < pose_covariance_matrix.size(); i++)
                odom.pose.covariance[i] = pose_covariance_matrix[i];
		
		//set the velocity
		odom.child_frame_id = base_frame_id;
		odom.twist.twist.linear.x = vel_x;
		odom.twist.twist.linear.y = vel_y;
		odom.twist.twist.angular.z = vel_yaw;
        if(twist_cov_mat)
            for(int i = 0; i < pose_covariance_matrix.size(); i++)
                odom.twist.covariance[i] = twist_covariance_matrix[i];
        
		//publish the message
		odom_pub.publish(odom);

		// ******************************************************************************************
		//publish battery
        irobotcreate2::Battery battery;
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
        irobotcreate2::Bumper bumper;
		bumper.left.header.stamp = current_time;
		bumper.left.state = roomba->bumper_[LEFT];
		bumper.right.header.stamp = current_time;
		bumper.right.state = roomba->bumper_[RIGHT];
		bumper_pub.publish(bumper);
		bumper_warning_ = bumper.left.state || bumper.right.state;
		bumper_warning.store(bumper_warning_);
	
		// ******************************************************************************************	
		//publish buttons
        irobotcreate2::Buttons buttons;
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
        irobotcreate2::RoombaIR cliff;
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
        irobotcreate2::RoombaIR irbumper;
		irbumper.header.stamp = current_time;

		irbumper.header.frame_id = "base_irbumper_left";
		irbumper.state = roomba->ir_bumper_[LEFT];
		irbumper.signal = roomba->ir_bumper_signal_[LEFT];
		irbumper_pub.publish(irbumper);
		ir_warning_ = ir_warning_ || irbumper.state;

		irbumper.header.frame_id = "base_irbumper_front_left";
		irbumper.state = roomba->ir_bumper_[FRONT_LEFT];
		irbumper.signal = roomba->ir_bumper_signal_[FRONT_LEFT];
		irbumper_pub.publish(irbumper);
		ir_warning_ = ir_warning_ || irbumper.state;

		irbumper.header.frame_id = "base_irbumper_center_left";
		irbumper.state = roomba->ir_bumper_[CENTER_LEFT];
		irbumper.signal = roomba->ir_bumper_signal_[CENTER_LEFT];
		irbumper_pub.publish(irbumper);
		ir_warning_ = ir_warning_ || irbumper.state;

		irbumper.header.frame_id = "base_irbumper_center_right";
		irbumper.state = roomba->ir_bumper_[CENTER_RIGHT];
		irbumper.signal = roomba->ir_bumper_signal_[CENTER_RIGHT];
		irbumper_pub.publish(irbumper);
		ir_warning_ = ir_warning_ || irbumper.state;

		irbumper.header.frame_id = "base_irbumper_front_right";
		irbumper.state = roomba->ir_bumper_[FRONT_RIGHT];
		irbumper.signal = roomba->ir_bumper_signal_[FRONT_RIGHT];
		irbumper_pub.publish(irbumper);
		ir_warning_ = ir_warning_ || irbumper.state;

		irbumper.header.frame_id = "base_irbumper_right";
		irbumper.state = roomba->ir_bumper_[RIGHT];
		irbumper.signal = roomba->ir_bumper_signal_[RIGHT];
		irbumper_pub.publish(irbumper);
		ir_warning_ = ir_warning_ || irbumper.state;

		ir_warning.store(ir_warning_);

		// ******************************************************************************************
		//publish irchar
        irobotcreate2::IRCharacter irchar;
		irchar.header.stamp = current_time;
		irchar.omni = roomba->ir_char_[OMNI];
		irchar.left = roomba->ir_char_[LEFT];
		irchar.right = roomba->ir_char_[RIGHT];
		irchar_pub.publish(irchar);

		// ******************************************************************************************
		//publish wheeldrop
        irobotcreate2::WheelDrop wheeldrop;
		wheeldrop.left.header.stamp = current_time;
		wheeldrop.left.state = roomba->wheel_drop_[LEFT];
		wheeldrop.right.header.stamp = current_time;
		wheeldrop.right.state = roomba->wheel_drop_[RIGHT];
		wheeldrop_pub.publish(wheeldrop);
		
		ros::spinOnce();
		r.sleep();
		
		if(first_loop)
		{
		    roomba->startOI(true);
		    if(!desired_mode_full) roomba->Safe();
		    first_loop=false;
		}
	}
	
	roomba->powerDown();
	roomba->closeSerialPort();
}

// EOF
