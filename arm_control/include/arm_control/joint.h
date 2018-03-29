/*
 * joint.h
 *
 *  Created on: Feb 6, 2018
 *      Author: Chinemelu Ezeh
 */

#ifndef SOURCE_DIRECTORY__ARM_CONTROL_INCLUDE_ARM_CONTROL_JOINT_H_
#define SOURCE_DIRECTORY__ARM_CONTROL_INCLUDE_ARM_CONTROL_JOINT_H_

/*
 * arm_control.cpp
 *
 *  Created on: Feb 6, 2018
 *      Author: Chinemelu Ezeh
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <string>
// important messages
#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <sensor_msgs/JointState.h>

// define class to instantiate two joints
class Joint {
public:
	Joint() {
		pos_cmd = 0.0;
		pos_cur = 0.0;
		pos_err = 0.0;
		vel_cur = 0.0;
		trq_cmd = 0.0;
		kp = 500.0;
		kv = 150.0;
	} // constructor
	void initialise(ros::NodeHandle nh, std::string joint_name, double dt);
	~Joint() {
	}
	; // destructor
	void getJointState();
	void jointTrqControl();
	void kpkvSetting(double kp, double kv);
	void setPosCmd(double);
private:
	// callback for the pos_cmd subscriber

	// callback for kpkv service server
	// service clients
	ros::ServiceClient get_jnt_state_client;
	ros::ServiceClient set_trq_client;
	// publisher objects
	ros::Publisher trq_publisher;
	ros::Publisher vel_publisher;
	ros::Publisher pos_publisher;
	ros::Publisher joint_state_publisher;
	// gazebo/sensor messages
	gazebo_msgs::GetJointProperties get_joint_state_srv_msg;
	gazebo_msgs::ApplyJointEffort effort_cmd_srv_msg;
	sensor_msgs::JointState joint_state_msg;
	// position/velocity/torque messages to be published
	std_msgs::Float64 pos_msg; // position
	std_msgs::Float64 vel_msg; // velocity
	std_msgs::Float64 trq_msg; // torque

	// control parameters
	double pos_cur; // current joint position
	double vel_cur; // current joint velocity
	double pos_cmd; // joint position from commander
	double pos_err; // error between pos_cmd and pos_cur
	double trq_cmd; // torque to be published
	double kp;
	double kv;
	// other parameters
	std::string joint_name;
};

#endif /* SOURCE_DIRECTORY__ARM_CONTROL_INCLUDE_ARM_CONTROL_JOINT_H_ */
