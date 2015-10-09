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
#include "irobotcreate2/OpenInterface.h"
#include <string>

std::string port;
irobot::OpenInterface * iRC2;
std::vector<std::string> commands({"start","stop","reset","powerdown","safe","full"});


void change_mode(std::string cmd)
{
        ROS_INFO_STREAM("Received command: "<<cmd);
	
	uint8_t digit0=45,digit1=45,digit2=45,digit3=45;
	bool warning=0,dock=0,spot=0,dirt=0,clean_col=255,clean_int=255;

	if(cmd=="exit") return;
	else if(cmd=="start")
	{
	    ROS_INFO_STREAM("Result: " << iRC2->Start());
	    digit0=79;
	    digit1=78;
	    digit2=32;
	    digit3=32;
	    warning=true;
	}
	else if(cmd=="stop")
	{
	    ROS_INFO_STREAM("Result: " << iRC2->Stop());
	    digit0=79;
	    digit1=70;
	    digit2=70;
	    digit3=32;
	}
	else if(cmd=="reset")
	{
	    ROS_INFO_STREAM("Result: " << iRC2->Reset());
	    digit0=82;
	    digit1=69;
	    digit2=83;
	    digit3=32;
	}
	else if(cmd=="powerdown")
	{
	    ROS_INFO_STREAM("Result: " << iRC2->powerDown());
	    digit0=68;
	    digit1=79;
	    digit2=87;
	    digit3=78;
	}
	else if(cmd=="safe")
	{
	    ROS_INFO_STREAM("Result: " << iRC2->Safe());
	    digit0=83;
	    digit1=65;
	    digit2=70;
	    digit3=69;
	}
	else if(cmd=="full")
	{
	    ROS_INFO_STREAM("Result: " << iRC2->Full());
	    digit0=70;
	    digit1=85;
	    digit2=76;
	    digit3=76;
	}

	if(iRC2->setDigitLeds(digit3,digit2,digit1,digit0)==-1) ROS_WARN("Error using display");
	if(iRC2->setLeds(warning, dock, spot, dirt, clean_col, clean_int)==-1) ROS_WARN("Error using leds");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "basic_commands");

	ROS_INFO("iRobotCreate2 Basic Utility");

	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	std::string port_name;
	if(argc==2) port_name=argv[1];
	else port_name = "/dev/ttyUSB0";
	
	ROS_INFO_STREAM("Connecting to "<< port_name);
	
	pn.param<std::string>("port", port, port_name);
	
	iRC2 = new irobot::OpenInterface(port.c_str());
	
	irobot::OI_Packet_ID sensor_packets[1] = {irobot::OI_PACKET_GROUP_100};
	iRC2->setSensorPackets(sensor_packets, 1, OI_PACKET_GROUP_100_SIZE);

	if( iRC2->openSerialPort(true) == 0) ROS_INFO("Connected to robot.");
	else
	{
		ROS_FATAL("Could not connect to robot.");
		ROS_BREAK();
	}
	
	std::cout<<"| Possible commands:"<<std::endl;
	for(int i=0; i< commands.size();i++)
	    std::cout<<"|  - "<<commands.at(i)<<std::endl;
	std::cout<<"| Enter 'exit' to quit"<<std::endl;
	
	std::string cmd;
	while(cmd!="exit")
	{
	    std::cin>>cmd;
	    change_mode(cmd);
	}

	iRC2->powerDown();
	iRC2->closeSerialPort();
}

// EOF
