/*
 * reactive.cpp
 *
 *  Created on: Dec 3, 2017
 *      Author: Chinemelu Ezeh
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <unistd.h>
#include <sstream>
#include <cmath>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <task_planner/helper.h> // Contains definitions of task ids

#define DIST_THRES 0.4
#define SENSOR_0_BIT 0
#define SENSOR_1_BIT 1
#define SENSOR_2_BIT 2
#define ODOM_BIT 3
using namespace std;

bool newGoal = false;
int executionCount = 10;
int odomflag = 0;
int dataflag = 0;

ros::Publisher cmd_pub;
ros::Publisher complete_pub;

geometry_msgs::Pose goal;
int sensorflag_new = 0;
int sensorflag = 0;
int lastright = 0;
int lastleft = 0;
int turnRightflag = 99;
int turnLeftflag = 0;

nav_msgs::Odometry odom;

float getAngDiff();

void initialiseGoal() {
	goal = geometry_msgs::Pose(odom.pose.pose);
}

// Breaks down trajectory to goal into achievable waypoints within a short time step.
// Way points are divided by distance and not time.

//void getWayPoints (vector<geometry_msgs::Pose>& wayPoints, geometry_msgs::Pose goal) {
//
//}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_t) {
	if (goal_t->header.frame_id == GOTO_ID) { // Only allow goto commands to pass.
		goal = geometry_msgs::Pose(goal_t->pose);
		cout << "Received new destination.." << endl;
		newGoal = true;
	}
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_t) {
	odom = nav_msgs::Odometry(*odom_t);
	dataflag |= (1 << ODOM_BIT);
}
void irCallback(const sensor_msgs::LaserScan::ConstPtr& ir) {

	float dist = ir->ranges[0];

	if (strcmp(ir->header.frame_id.c_str(), "ir0") == 0) {
		if (!isinf(dist))
			if (dist < DIST_THRES) {
				sensorflag |= (1 << SENSOR_0_BIT);
			}
		dataflag |= (1 << SENSOR_0_BIT);
	} else if (strcmp(ir->header.frame_id.c_str(), "ir4") == 0) {
		if (!isinf(dist))
			if (dist < DIST_THRES) {
				sensorflag |= (1 << SENSOR_1_BIT);
			}
		dataflag |= (1 << SENSOR_1_BIT);
	} else if (strcmp(ir->header.frame_id.c_str(), "ir1") == 0) {
		if (!isinf(dist))
			if (dist < DIST_THRES) {
				sensorflag |= (1 << SENSOR_2_BIT);
			}
		dataflag |= (1 << SENSOR_2_BIT);
	}
}

void irCallback0(const sensor_msgs::LaserScan::ConstPtr& ir) {

	float dist = ir->ranges[0];
	if (isinf(dist))
		return;
	if (dist < DIST_THRES) {
		sensorflag_new |= (1 << SENSOR_0_BIT);
	}
}

void irCallback1(const sensor_msgs::LaserScan::ConstPtr& ir) {
	float dist = ir->ranges[0];
	if (isinf(dist))
		return;
	if (dist < DIST_THRES) {
		sensorflag_new |= (1 << SENSOR_1_BIT);
	}
}
void irCallback2(const sensor_msgs::LaserScan::ConstPtr& ir) {
	float dist = ir->ranges[0];
	if (isinf(dist))
		return;
	if (dist < DIST_THRES) {
		sensorflag_new |= (1 << SENSOR_2_BIT);
	}
}

void align() {
	geometry_msgs::Twist cmd;
	float turn, target;
	target = getYaw(goal.orientation);

	turn = angDiff(target, getYaw(odom.pose.pose.orientation));

	cout  <<"turn by "<<turn<< " target: " << target << ", theta:"
					<< getYaw(odom.pose.pose.orientation) << endl;
	cmd.angular.z = -turn;
	cmd_pub.publish(cmd);
	float diff = turn;

	while (ros::ok() && abs(diff) > .3) {
		diff = angDiff(target, getYaw(odom.pose.pose.orientation));
		usleep(10000);
		cout << "Aligning Diff: " << diff << " target: " << target << ", theta:"
				<< getYaw(odom.pose.pose.orientation) << endl;
	}
	cout << "Completed aligning" << endl;
}

void turnLeft(bool withObst = 0) {

	geometry_msgs::Twist cmd;

	turnLeftflag = 0;
//	cmd.angular.z = .3;
//	cmd_pub.publish(cmd);
	cout << "Turn Left" << endl;
	float turn, target;
	if (withObst) {
		target = angDiff(getYaw(odom.pose.pose.orientation), -M_PI / 3);
		turn = M_PI / 3;
		//		while (ros::ok()
		//				& abs(angDiff(target, getYaw(odom.pose.pose.orientation))) > .1)
		//			;

		lastright = 1;
	} else {
		lastright = 0;
		target = atan2(goal.position.y - odom.pose.pose.position.y,
				goal.position.x - odom.pose.pose.position.x);
		turn = getAngDiff();

	}

	cmd.angular.z = turn;
	cmd_pub.publish(cmd);
	float diff = angDiff(target, getYaw(odom.pose.pose.orientation));

	while (ros::ok() && abs(diff) > .3) {
		diff = angDiff(target, getYaw(odom.pose.pose.orientation));
		usleep(10000);
		cout << "Left Diff: " << diff << " target: " << target << ", theta:"
				<< getYaw(odom.pose.pose.orientation) << endl;
	}
	cout << "Completed turn" << endl;
}

void turnRight(bool withObst = 0) {

	geometry_msgs::Twist cmd;

//	cmd.angular.z = -0.3;
//	cmd_pub.publish(cmd);
	cout << "Turn Right" << endl;
	turnRightflag = 0;

	float turn, target;
	if (withObst) {
		target = angDiff(getYaw(odom.pose.pose.orientation), M_PI / 3);
		turn = -M_PI / 3;
//		while (ros::ok()
//				& abs(angDiff(target, getYaw(odom.pose.pose.orientation))) > .1)
//			;

		lastright = 1;
	} else {
		lastright = 0;
		target = atan2(goal.position.y - odom.pose.pose.position.y,
				goal.position.x - odom.pose.pose.position.x);
		turn = getAngDiff();

	}

	cmd.angular.z = turn;
	cmd_pub.publish(cmd);
	float diff = angDiff(target, getYaw(odom.pose.pose.orientation));

	while (ros::ok() && abs(diff) > .3) {
		diff = angDiff(target, getYaw(odom.pose.pose.orientation));
		usleep(10000);
		cout << "Right Diff: " << diff << " target: " << target << ", theta:"
				<< getYaw(odom.pose.pose.orientation) << endl;
	}
	cout << "Completed turn" << endl;

}

void goStraight() {
	turnRightflag++;
	turnLeftflag++;
	geometry_msgs::Twist cmd;

	cmd.linear.x = .2;
	cmd_pub.publish(cmd);
	cout << "Go Straight" << endl;
}

void stop(){
	geometry_msgs::Twist cmd;
	cout << "Stopping" << endl;
	cmd_pub.publish(cmd);
}

float getAngDiff() {
	float heading = getYaw(odom.pose.pose.orientation);
	float bearingToGoal = atan2(goal.position.y - odom.pose.pose.position.y,
			goal.position.x - odom.pose.pose.position.x);
	float angdiff = angDiff(heading, bearingToGoal);
	return angdiff;
}
void moveToGoal() {
	if (!newGoal)
		return;
	std_msgs::Int8 complete_status;
	geometry_msgs::Twist cmd;
	// check if we have arrived at goal

	float d = sqrt(
			pow(odom.pose.pose.position.x - goal.position.x, 2)
					+ pow(odom.pose.pose.position.y - goal.position.y, 2));

	float angdiff = getAngDiff();
	cout << "Difference to goal. Dist:" << d << ", Angle:" << angdiff << endl;
	if (d < 0.04) { // Stop, have arrived.
		align();
		stop();
		complete_status.data = 1;
		newGoal = false;
		// indicate that we have reached.
		cout << "REACHED!!!:" << endl;
	} else {

		if (abs(angdiff) < 0.3) {
			if (sensorflag &= (1 << SENSOR_0_BIT)) {
				if (((sensorflag &= (1 << SENSOR_1_BIT)) == 0)
						&& (turnLeftflag > 20)) {

					turnRight(true);
				}
				if (((sensorflag &= (1 << SENSOR_2_BIT)) == 0)
						&& (turnRightflag > 20)) {
					turnLeft(true);
				}
			} else {
//				May need to modify move forward condition
				goStraight();
			}
		} else if (angdiff < 0) { // Turn  left.
			if (((sensorflag &= (1 << SENSOR_2_BIT)) == 0)
					&& (turnRightflag > 20)) {

				turnLeft(lastright ? true : false);
				lastright = false;
			} else if ((sensorflag &= (1 << SENSOR_0_BIT)) == 0) {
				goStraight();
			} else {
				stop();
				cout <<"Check2.1"<<endl;
			}
		} else if (angdiff > 0) { // Turn  right.
			if (((sensorflag &= (1 << SENSOR_1_BIT)) == 0)
					&& (turnLeftflag > 20)) {
				turnRight(lastleft ? true : false);
				lastleft = false;
			} else if ((sensorflag &= (1 << SENSOR_0_BIT)) == 0) {
				goStraight();
			} else {
				stop();
				cout <<"Check2"<<endl;
			}
		}
		complete_status.data = -1;
	}
	complete_pub.publish(complete_status);
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {

	ros::init(argc, argv, "Reactive");

	ros::NodeHandle n;

	cmd_pub = n.advertise<geometry_msgs::Twist>("/my_robot/cmd_vel", 10);
	complete_pub = n.advertise<std_msgs::Int8>("/task_status", 10);
	ros::CallbackQueue queue;

	// create options for subscriber and pass pointer to our custom queue
	ros::SubscribeOptions ops = ros::SubscribeOptions::create<
			geometry_msgs::PoseStamped>("/my_robot/goal", // topic name
			10, // queue length
			goalCallback, // callback
			ros::VoidPtr(), // tracked object, we don't need one thus NULL
			&queue // pointer to callback queue object
			);
// Goal
	ros::Subscriber sub = n.subscribe(ops);
	ops = ros::SubscribeOptions::create<sensor_msgs::LaserScan>("ir", // topic name
			1, // queue length
			irCallback, // callback
			ros::VoidPtr(), // tracked object, we don't need one thus NULL
			&queue // pointer to callback queue object
			);
// IRs
	ros::Subscriber sub1 = n.subscribe(ops);
	ops = ros::SubscribeOptions::create<nav_msgs::Odometry>("/my_robot/odom", // topic name
			1, // queue length
			odomCallback, // callback
			ros::VoidPtr(), // tracked object, we don't need one thus NULL
			&queue // pointer to callback queue object
			);

	// Odom
	ros::Subscriber sub2 = n.subscribe(ops);

	// Spinnert
	ros::AsyncSpinner spinner(0, &queue);
	spinner.start();

	ros::Rate loop_rate(10);
//	goal.position.x = 10;
	goal.orientation.w = 1;
	sleep(2);
	initialiseGoal();
	while (ros::ok()) {
		if (dataflag != 8) {
			stop();
			ros::spinOnce();
		} else {
			moveToGoal();
			dataflag = 0;
			ros::spinOnce();
		}

		loop_rate.sleep();
	}

	return 0;
}

