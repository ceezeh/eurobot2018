/*
 * taskplanner.cpp
 *
 *  Created on: Jan 13, 2018
 *      Author: deeplearning
 */
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <unistd.h>
#include <sstream>
#include <cmath>
#include <vector>
#include "task_planner/helper.h"
#include <ros/spinner.h>
#include <ros/callback_queue.h>
using namespace std;

/*
 * TODO: Add more task bits
 * #define GOTO_BIT 0
 * #define PICKUP_BIT 1
 * #define PLACE_BIT 2
 */
#define GOTO_BIT 0

int status_flags = 0;

geometry_msgs::PoseStamped cmd;

void statusCallback(const std_msgs::Int8::ConstPtr& status) {

	/*
	 * Add more status call backs for other behaviours.
	 */
	switch (status->data) {
	case STATUS_NAV_COMPLETE:
		status_flags |= (1 << GOTO_BIT);
		cout << "Finished going to location" << endl;
		break;
	case 2:
		// status_bit =something else bit;
		break;
	case 3:
		// status_bit =something else again bit;
		break;
	default:
		// Do nothing
		break;
	}
}

geometry_msgs::PoseStamped getNextCommand() {
	cmd.header.stamp = ros::Time::now();
	cmd.header.frame_id = GOTO_ID;
	cmd.pose.position.x += 2;
	return cmd;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "planner");

	ros::NodeHandle n;

	ros::CallbackQueue queue;

	// create options for subscriber and pass pointer to our custom queue
	ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Int8>(
			"/task_status", // topic name
			10, // queue length
			statusCallback, // callback
			ros::VoidPtr(), // tracked object, we don't need one thus NULL
			&queue // pointer to callback queue object
			);
	// Task Status.
	ros::Subscriber sub = n.subscribe(ops);
	// Spinner
	ros::AsyncSpinner spinner(0, &queue);
	spinner.start();

	ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>(
			"/my_robot/goal", 10);

	ros::Rate loop_rate(10);

	int cmdPtr = 0;
	vector<geometry_msgs::PoseStamped> cmdArray;
	geometry_msgs::PoseStamped temp;
	temp.header.frame_id = GOTO_ID;
	temp.pose.position.x = 1.0;
	temp.pose.position.y = 0.5;
	temp.pose.orientation =  getQuaternion(0);
	cmdArray.push_back(geometry_msgs::PoseStamped(temp));
	temp.header.frame_id = GOTO_ID;
	temp.pose.position.x = 1.0;
	temp.pose.position.y = 1.5;
	temp.pose.orientation =  getQuaternion(M_PI/2);
	cmdArray.push_back(geometry_msgs::PoseStamped(temp));
	temp.header.frame_id = GOTO_ID;
	temp.pose.position.x = 0;
	temp.pose.position.y = 1.5;
	temp.pose.orientation = getQuaternion(M_PI);
	cmdArray.push_back(geometry_msgs::PoseStamped(temp));
	temp.header.frame_id = GOTO_ID;
	temp.pose.position.x = 0;
	temp.pose.position.y = 0.5;
	temp.pose.orientation = getQuaternion(M_PI *3/2);
	cmdArray.push_back(geometry_msgs::PoseStamped(temp));

	int count =0;
	while (ros::ok() && count++<50)
		loop_rate.sleep();

	while (ros::ok()) {
//		/*-------------------------------------------------
//		 * This block of code generates next command and then publishes it.
//		 * It also waits for command to be executed before issuing the next command.
//		 */
//
		cout << "Getting next task..." << endl;
//		geometry_msgs::PoseStamped cmd = getNextCommand();
		geometry_msgs::PoseStamped cmd = cmdArray[cmdPtr++];
		if (cmdPtr == cmdArray.size() ) {//
			cmdPtr = 0;
		}
		pub.publish(cmd);
//		// Wait for instruction completion
//
//		// Get task status bit.
		int status_bit = -1;
		if (cmd.header.frame_id == GOTO_ID) {
			status_bit = GOTO_BIT;
			cout << "Going to location..." << cmd<<endl;
		} else if (cmd.header.frame_id == "Something else") {
			// status_bit =something else bit;
			break;
		} else if (cmd.header.frame_id == "Something else again") {
			// status_bit =something else again bit;
		}
//
//		//----------------------------------End of block-----------------
//
//		// Check if task was completed.
		if (status_bit != -1) {
			while (ros::ok() && (status_flags & (1 << status_bit)) == 0) {
				loop_rate.sleep();
			}
			// Unset or rest flag.
			status_flags &= (0 << status_bit);
		}


		sleep(2);
		loop_rate.sleep();
	}

	return 0;
}
