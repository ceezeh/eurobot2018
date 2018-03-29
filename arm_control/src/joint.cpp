/*
 * joint.cpp
 *
 *  Created on: Feb 6, 2018
 *      Author: Chinemelu Ezeh
 */

#include "arm_control/joint.h"
using namespace std;

void Joint::initialise(ros::NodeHandle nh, std::string joint_name, double dt) {
	// initialize parameters
	this->joint_name = joint_name;
	pos_cmd = 0.0;
	pos_cur = 0.0;
	pos_err = 0.0;
	vel_cur = 0.0;
	trq_cmd = 0.0;
	ros::Duration duration(dt);
	kp = 500.0;
	kv = 150.0;

	// initialize gazebo clients
	get_jnt_state_client = nh.serviceClient < gazebo_msgs::GetJointProperties
			> ("/gazebo/get_joint_properties");
	set_trq_client = nh.serviceClient < gazebo_msgs::ApplyJointEffort
			> ("/gazebo/apply_joint_effort");
	// initialize publisher objects
	pos_publisher = nh.advertise < std_msgs::Float64 > (joint_name + "_pos", 1);
	vel_publisher = nh.advertise < std_msgs::Float64 > (joint_name + "_vel", 1);
	trq_publisher = nh.advertise < std_msgs::Float64 > (joint_name + "_trq", 1);
	joint_state_publisher = nh.advertise < sensor_msgs::JointState
			> (joint_name + "_states", 1);
	// set up get_joint_state_srv_msg
	get_joint_state_srv_msg.request.joint_name = joint_name;
	// set up effort_cmd_srv_msg
	effort_cmd_srv_msg.request.joint_name = joint_name;
	effort_cmd_srv_msg.request.effort = 0.0;
	effort_cmd_srv_msg.request.duration = duration;
	// set up joint_state_msg
	joint_state_msg.header.stamp = ros::Time::now();
	joint_state_msg.name.push_back(joint_name);
	joint_state_msg.position.push_back(0.0);
	joint_state_msg.velocity.push_back(0.0);
}

void Joint::setPosCmd(double cmd) {
	// too much information
	// ROS_INFO("received value of %s_pos_cmd is: %f", joint_name.c_str(), pos_cmd_msg.data);
	pos_cmd = cmd;
}

void Joint::getJointState() {
	// get joint state
	get_jnt_state_client.call(get_joint_state_srv_msg);
	// publish joint position
	pos_cur = get_joint_state_srv_msg.response.position[0];
	pos_msg.data = pos_cur;
	pos_publisher.publish(pos_msg);
	// publish joint velocity
	vel_cur = get_joint_state_srv_msg.response.rate[0];
	vel_msg.data = vel_cur;
	vel_publisher.publish(vel_msg);
	// publish joint_state_msg
	joint_state_msg.header.stamp = ros::Time::now();
	joint_state_msg.position[0] = pos_cur;
	joint_state_msg.velocity[0] = vel_cur;
	joint_state_publisher.publish(joint_state_msg);
}

// calculate joint torque, publish them, send to gazebo
void Joint::jointTrqControl() {
	pos_err = pos_cmd - pos_cur;
	// watch for periodicity
	if (pos_err > M_PI)
		pos_err = pos_err - 2 * M_PI;
	if (pos_err > M_PI)
		pos_err = pos_err + 2 * M_PI;
	// control algorithm in one line
	trq_cmd = kp * pos_err - kv * vel_cur;
	// publish the torque message
	trq_msg.data = trq_cmd;
	trq_publisher.publish(trq_msg);
	// send torque command to gazebo
	effort_cmd_srv_msg.request.effort = trq_cmd;
	set_trq_client.call(effort_cmd_srv_msg);
	// make sure service call was successful
	bool result = effort_cmd_srv_msg.response.success;
	if (!result)
		ROS_WARN("service call to apply_joint_effort failed!");
}

void Joint::kpkvSetting(double kp, double kv) {
	this->kp = kp;
	this->kv = kv;
}

