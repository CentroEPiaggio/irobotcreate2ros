/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, University of Pisa
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
* Author: Alessandro Settimi 2015
* Author: Mirko Ferrati 2015
*********************************************************************/

#include "joy_node.h"

#define MAX_LIN_VEL 0.2
#define MAX_ANG_VEL 1.0

joy_handler::joy_handler()
{
    joy_sub  = nodeh.subscribe<sensor_msgs::Joy>("/joy", 1, &joy_handler::joy_receive, this);
    twist_pub = nodeh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    song_pub = nodeh.advertise<irobotcreate2::Song>("/song", 1);
    playsong_pub = nodeh.advertise<irobotcreate2::PlaySong>("/play_song", 1);
    saver = nodeh.advertise<std_msgs::String>("/syscommand", 1);
    mode_pub = nodeh.advertise<std_msgs::String>("/mode", 1);

    twist.linear.x=0;
    twist.linear.y=0;
    twist.linear.z=0;
    twist.angular.x=0;
    twist.angular.y=0;
    twist.angular.z=0;

    selection_sub = nodeh.subscribe("/agent_selection",10,&joy_handler::agent_selection_callback,this);
    dual_mode=true;
}

void joy_handler::change_topics(std::string ns)
{
    if(ns!="") ns="/"+ns;

    twist_pub = nodeh.advertise<geometry_msgs::Twist>(ns+"/cmd_vel", 1);
    song_pub = nodeh.advertise<irobotcreate2::Song>(ns+"/song", 1);
    playsong_pub = nodeh.advertise<irobotcreate2::PlaySong>(ns+"/play_song", 1);
    saver = nodeh.advertise<std_msgs::String>("/syscommand", 1);
    mode_pub = nodeh.advertise<std_msgs::String>(ns+"/mode", 1);
}

void joy_handler::agent_selection_callback(const std_msgs::String& msg)
{
    if(msg.data == "All") change_topics("");
    else change_topics(msg.data);
}

void joy_handler::joy_receive(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
    if(dual_mode)
    {
        twist.linear.x = MAX_LIN_VEL*joy_msg->axes.at(LEFT_X_AXIS);
        twist.angular.z = MAX_ANG_VEL*joy_msg->axes.at(RIGHT_Y_AXIS);
    }
    else
    {
        twist.linear.x = MAX_LIN_VEL*joy_msg->axes.at(LEFT_X_AXIS);
        twist.angular.z = MAX_ANG_VEL*joy_msg->axes.at(LEFT_Y_AXIS);
    }
    
    if(joy_msg->buttons.at(SELECT_BUTTON)) dual_mode=!(dual_mode);

    if(joy_msg->buttons.at(START_BUTTON))
    {
	mode.data="safe";
	mode_pub.publish(mode);
    }

    if(joy_msg->buttons.at(X_BUTTON))
    {
        song.notes.clear();
        song.song_number = 0;
        irobotcreate2::Note note;
        note.note = 64;
        note.length = 40;
        song.notes.push_back(note);
        note.note = 16;
        note.length = 3;
        song.notes.push_back(note);
        song_pub.publish(song);

        usleep(10000);
        play.song_number=0;
        playsong_pub.publish(play);
    }
    
    if(joy_msg->buttons.at(CIRCLE_BUTTON)) 
    {
        std_msgs::String mess;
        mess.data = "save_image";
        saver.publish(mess);
        ros::Duration(0.3).sleep();
    }
    
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