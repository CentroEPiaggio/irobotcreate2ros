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
* 
*********************************************************************/

#include "keyboard_node.h"

#define MAX_LIN_VEL 0.1
#define MAX_ANG_VEL 1.0

int kfd = 0;
struct termios cooked, raw;

keyboard_handler::keyboard_handler()
{
    twist_pub = nodeh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    selection_sub = nodeh.subscribe("/agent_selection",10,&keyboard_handler::agent_selection_callback,this);

    twist.linear.x=0;
    twist.linear.y=0;
    twist.linear.z=0;
    twist.angular.x=0;
    twist.angular.y=0;
    twist.angular.z=0;
}

void keyboard_handler::agent_selection_callback(const std_msgs::String& msg)
{
    if(msg.data == "All") twist_pub = nodeh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    else twist_pub = nodeh.advertise<geometry_msgs::Twist>("/"+msg.data+"/cmd_vel", 1);
}

void keyboard_handler::keyboard_reading()
{
    char c;

    // get the console in raw mode                                                              
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file                         
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    ROS_INFO("Use arrow keys to move the robot.");

    while(1)
    {
	if(read(kfd, &c, 1) < 0)
	{
	    ROS_ERROR("Something wrong!");
	    exit(-1);
	}

	twist.linear.x = 0;
	twist.angular.z = 0;

	switch(c)
	{
	    case LEFT:
		twist.angular.z = MAX_ANG_VEL;
		break;
	    case RIGHT:
		twist.angular.z = -MAX_ANG_VEL;
		break;
	    case FORWARD:
		twist.linear.x = MAX_LIN_VEL;
		break;
	    case BACKWARD:
		twist.linear.x = -MAX_LIN_VEL;
		break;
	    case STOP:
		twist.linear.x = 0;
		twist.angular.z = 0;
	    break;
	}

	twist_pub.publish(twist);    
 
	ros::spinOnce();

	usleep(100);
    }    
}

keyboard_handler::~keyboard_handler()
{

}

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "keyboard_node");

    keyboard_handler keyboard_h;

    ROS_INFO("Keyboard Handler Started");

    signal(SIGINT,quit);

    keyboard_h.keyboard_reading();

    return 0;
}