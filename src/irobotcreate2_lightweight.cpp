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
* Author: Giuseppe Lafortezza 2016
* Author: Alessandro Settimi 2015
* Author: Mirko Ferrati 2015
* Author: Gonçalo Cabrita on 07/10/2010
*********************************************************************/

#define NODE_VERSION 2.01

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>          // odom
#include <geometry_msgs/Twist.h>        // cmd_vel

#include "irobotcreate2/OpenInterface.h"

#include <string>
#include <std_msgs/String.h>
#include <atomic>
#include <eigen3/Eigen/Eigen>
#include <XmlRpc.h>

std::string port;
bool publishTf;
std::string robot_id;
irobot::OpenInterface* roomba;


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

void cmdSpecialReceived(const std_msgs::String::ConstPtr& cmd_)
{
    if(cmd_->data=="reset_odom") roomba->resetOdometry();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "roomba560_light_node");

    ROS_INFO("Roomba for ROS %.2f", NODE_VERSION);
    
    double last_x, last_y, last_yaw;
    double vel_x, vel_y, vel_yaw;
    double dt;

    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    
    pn.param<std::string>("port_", port, "/dev/ttyUSB0");
    pn.param<bool>("publishTf", publishTf, false);
    pn.param<std::string>("robot_id", robot_id, "lcr_1");
    
    //Load covariances
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
    irobot::OI_Packet_ID sensor_packets[1] = {irobot::OI_PACKET_GROUP_100};
    roomba->setSensorPackets(sensor_packets, 1, OI_PACKET_GROUP_100_SIZE);

    tf::TransformBroadcaster odom_broadcaster;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Subscriber cmd_vel_sub  = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, cmdVelReceived);
    ros::Subscriber special_sub  = n.subscribe<std_msgs::String>("special", 1, cmdSpecialReceived);


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

        /*
        dt = (current_time - last_time).toSec();
        vel_x = (roomba->odometry_x_ - last_x)/dt;
        vel_y = (roomba->odometry_y_ - last_y)/dt;
        vel_yaw = (roomba->odometry_yaw_ - last_yaw)/dt;*/
        vel_x = roomba->new_odometry_.getLinear();
        vel_y = 0.0;
        vel_yaw = roomba->new_odometry_.getAngular();
        
        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(roomba->odometry_yaw_);
        
        //first, we'll publish the transform over tf
        if(publishTf)
        {
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = robot_id + "/odom";
            odom_trans.child_frame_id = robot_id + "/base_link";
            odom_trans.transform.translation.x = roomba->odometry_x_;
            odom_trans.transform.translation.y = roomba->odometry_y_;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;
            //send the transform
            odom_broadcaster.sendTransform(odom_trans);
        }

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = robot_id + "/odom";
        
        //set the position
        odom.pose.pose.position.x = roomba->odometry_x_;
        odom.pose.pose.position.y = roomba->odometry_y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        if(pose_cov_mat)
            for(int i = 0; i < pose_covariance_matrix.size(); i++)
                odom.pose.covariance[i] = pose_covariance_matrix[i];
        
        //set the velocity
        odom.child_frame_id = robot_id + "/base_link";
        odom.twist.twist.linear.x = vel_x;
        odom.twist.twist.linear.y = vel_y;
        odom.twist.twist.angular.z = vel_yaw;
        if(twist_cov_mat)
            for(int i = 0; i < pose_covariance_matrix.size(); i++)
                odom.twist.covariance[i] = twist_covariance_matrix[i];
        
        //publish the message
        odom_pub.publish(odom);

        ros::spinOnce();
        r.sleep();
    }
    roomba->powerDown();
    roomba->closeSerialPort();
}