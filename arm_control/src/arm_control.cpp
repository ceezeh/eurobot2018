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
#include "arm_control/joint.h"
#include "arm_control/helper.h"
#include "arm_control/arm_control.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <task_planner/helper.h> // Contains definitions of task ids
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <queue>
#include <thread>

using namespace std;
bool newJob = false;
Joint rightFinger;
Joint leftFinger;
Joint armJoint0;
Joint armJoint1;
Joint armJoint2;
Joint palm;

std::queue<geometry_msgs::PoseStamped> command_queue;

double gripper_center = 0.15; // distance from gripper center to link4
double gripper_radius = 0.04; // the radius of the gripper paddle
double gripper_range = 0.16; // distance from the gripper paddle axis to gripper center

// TODO: create job fifo.
void cmdCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_t) {
	// Only allow arm commands to pass.
	if ((goal_t->header.frame_id == PICK_ID)
			|| (goal_t->header.frame_id == PLACE_ID)) {
		command_queue.push(*goal_t);
	}
}

void moveArm(const geometry_msgs::PoseStamped goal_t) {
	geometry_msgs::Point goal = geometry_msgs::Point(goal_t.pose.position);

	if ((goal_t.header.frame_id == PICK_ID)
			|| (goal_t.header.frame_id == PLACE_ID)) { // Only allow arm commands to pass.
		std::vector<double> joint_angles = calcInvK(goal);
		armJoint0.setPosCmd(joint_angles[0]);
		armJoint1.setPosCmd(joint_angles[1]);
		armJoint2.setPosCmd(joint_angles[2]);
		palm.setPosCmd(joint_angles[3]);
	}
}

void grasp(const string cmd) {
	double grasp_dist;
	if (cmd == PICK_ID) {
		grasp_dist = 0.05;
	} else if (cmd == PLACE_ID) {
		grasp_dist = 0.15;
	}
	double half_dist = grasp_dist / 2 + gripper_radius;
	rightFinger.setPosCmd(-(gripper_range - half_dist)); // joint5
	leftFinger.setPosCmd(gripper_range - half_dist); // joint6
}

void moveArmDefault() {
	armJoint0.setPosCmd(60 * M_PI / 180);
	armJoint1.setPosCmd(60 * M_PI / 180);
	armJoint2.setPosCmd(0);
	palm.setPosCmd(0);
}

void scheduleAction() {
	while (ros::ok()) {
		while (!command_queue.empty()) {
			geometry_msgs::PoseStamped cmd = command_queue.front();
			if (cmd.header.frame_id == PICK_ID) {
				/*
				 * To execute a pick, we first open up the robot fingers, then move the arm in place,
				 * then grab object and finally lift arm up.
				 */
				grasp(PLACE_ID); // Using place because it opens up the fingers.
				moveArm(cmd);
				grasp(PICK_ID);
				moveArmDefault();

			}
			if (cmd.header.frame_id == PLACE_ID) {
				/*
				 * To execute a place object, we first move the arm in place,
				 * then open the fingers and finally lift arm up.
				 */
				moveArm(cmd);
				grasp(PLACE_ID);
				moveArmDefault();
			}

			command_queue.pop();
		}
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "arm_control");
	ros::NodeHandle nh;
	ros::Duration half_sec(0.5);

	// make sure apply_joint_effort service is ready
	bool service_ready = false;
	while (!service_ready) {
		service_ready = ros::service::exists("/gazebo/apply_joint_effort",
				true);
		ROS_INFO("waiting for apply_joint_effort service");
		half_sec.sleep();
	}
	ROS_INFO("apply_joint_effort service exists");
	// make sure get_joint_state_client service is ready
	service_ready = false;
	while (!service_ready) {
		service_ready = ros::service::exists("/gazebo/get_joint_properties",
				true);
		ROS_INFO("waiting for /gazebo/get_joint_properties service");
		half_sec.sleep();
	}
	ROS_INFO("/gazebo/get_joint_properties service exists");

	ros::CallbackQueue queue;

	// create options for subscriber and pass pointer to our custom queue
	ros::SubscribeOptions ops = ros::SubscribeOptions::create<
			geometry_msgs::PoseStamped>("/my_robot/goal", // topic name
			10, // queue length
			cmdCallback, // callback
			ros::VoidPtr(), // tracked object, we don't need one thus NULL
			&queue // pointer to callback queue object
			);
	ros::Subscriber sub1 = nh.subscribe(ops);
	// Spinnert
	ros::AsyncSpinner spinner(0, &queue);


	double dt = 0.01; // sample time for the controller

	// instantiate 6 joint instances
	armJoint0.initialise(nh, "arm_joint_0", dt);
	armJoint1.initialise(nh, "arm_joint_1", dt);
	armJoint2.initialise(nh, "arm_joint_2", dt);
	palm.initialise(nh, "palm_joint", dt);
	rightFinger.initialise(nh, "right_finger_joint", dt);
	leftFinger.initialise(nh, "left_finger_joint", dt);
	// with gravity -0.1
	armJoint0.kpkvSetting(1000, 300);
	armJoint1.kpkvSetting(1000, 300);
	armJoint2.kpkvSetting(500, 150);
	palm.kpkvSetting(30, 9);
	rightFinger.kpkvSetting(60, 18);
	leftFinger.kpkvSetting(60, 18);

	ros::Rate rate_timer(1 / dt);
	std::thread first (scheduleAction);
	spinner.start();
	while (ros::ok()) {
		// get joint state(pos, vel) and publish them
		armJoint0.getJointState();
		armJoint1.getJointState();
		armJoint2.getJointState();
		palm.getJointState();
		rightFinger.getJointState();
		leftFinger.getJointState();

		// calculate the torque for each joint and publish them
		armJoint0.jointTrqControl();
		armJoint1.jointTrqControl();
		armJoint2.jointTrqControl();
		palm.jointTrqControl();
		rightFinger.jointTrqControl();
		leftFinger.jointTrqControl();

		ros::spinOnce(); // update pos_cmd, kpkv
		rate_timer.sleep(); // sleep the sample time
	}
}

