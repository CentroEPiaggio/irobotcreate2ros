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

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include "irobotcreate2/Note.h"
#include "irobotcreate2/Song.h"
#include "irobotcreate2/PlaySong.h"
#include <std_msgs/String.h>

#define SELECT_BUTTON 0
#define L3_BUTTON 1
#define R3_BUTTON 2
#define START_BUTTON 3
#define UP_BUTTON 4
#define RIGHT_BUTTON 5
#define DOWN_BUTTON 6
#define LEFT_BUTTON 7
#define L2_BUTTON 8
#define R2_BUTTON 9
#define L1_BUTTON 10
#define R1_BUTTON 11
#define TRIANGLE_BUTTON 12
#define CIRCLE_BUTTON 13
#define X_BUTTON 14
#define SQUARE_BUTTON 15
#define PLAY_BUTTON 16

#define LEFT_Y_AXIS 0
#define LEFT_X_AXIS 1
#define RIGHT_Y_AXIS 2
#define RIGHT_X_AXIS 3

class joy_handler
{
public:
  joy_handler();
  ~joy_handler();
private:
  ros::NodeHandle nodeh;
  ros::Subscriber joy_sub;
  ros::Publisher twist_pub, song_pub, playsong_pub, mode_pub;
  ros::Publisher saver;
  geometry_msgs::Twist twist;
  irobotcreate2::Song song;
  irobotcreate2::PlaySong play;
  std_msgs::String mode;
  bool dual_mode;

  void joy_receive(const sensor_msgs::Joy::ConstPtr& joy_msg);
  void agent_selection_callback(const std_msgs::String& msg);
  ros::Subscriber selection_sub;
  void change_topics(std::string ns);
};