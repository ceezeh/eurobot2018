/*
 * reactive.cpp
 *
 *  Created on: Dec 3, 2017
 *      Author: deeplearning
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include <unistd.h>
#include <sstream>
#include <cmath>
#include "reactive_control/helper.h"

#define DIST_THRES 0.4
#define SENSOR_0_BIT 0
#define SENSOR_1_BIT 1
#define SENSOR_2_BIT 2
#define ODOM_BIT 3

int executionCount = 10;
int odomflag = 0;
int dataflag = 0;
using namespace std;
ros::Publisher cmd_pub;

geometry_msgs::Point goal;
int sensorflag_new = 0;
int sensorflag = 0;
int turnRightflag = 99;
int turnLeftflag = 0;

nav_msgs::Odometry odom;

void odomCallback(const nav_msgs::Odometry& odom_t) {
	odom = nav_msgs::Odometry(odom_t);
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

void turnLeft() {
	turnLeftflag = 0;
	geometry_msgs::Twist cmd;

	cmd.angular.z = -.1;
	cmd_pub.publish(cmd);
	cout << "Turn Left" << endl;
	sleep(10);
}
void turnFwdLeft() {
	geometry_msgs::Twist cmd;
	cmd.linear.x = .05;
	cmd.angular.z = -0.5;
	cmd_pub.publish(cmd);
	cout << "Turn Forward Left" << endl;
}
void turnRight() {
	turnRightflag = 0;
	geometry_msgs::Twist cmd;

	cmd.angular.z = 0.1;
	cmd_pub.publish(cmd);
	cout << "Turn Right" << endl;
	sleep(10);
}

void goStraight() {
	turnRightflag++;
	turnLeftflag++;
	geometry_msgs::Twist cmd;

	cmd.linear.x = .2;
	cmd_pub.publish(cmd);
	cout << "Go Straight" << endl;
}
void reverseAndLeft() {

}
void moveToGoal() {
	geometry_msgs::Twist cmd;
	// check if we have arrived at goal
	float d = sqrt(
			pow(odom.pose.pose.position.x - goal.x, 2)
					+ pow(odom.pose.pose.position.y - goal.y, 2));
	if (d < 0.1) { // Stop, have arrived.
		cmd_pub.publish(cmd);
	} else {
		float heading = getYaw(odom.pose.pose.orientation);
		float bearingToGoal = atan2(goal.y - odom.pose.pose.position.y,
				goal.x - odom.pose.pose.position.x);

		cout << "Difference to goal:" << angDiff(heading, bearingToGoal)
				<< endl;
		if (abs(angDiff(heading, bearingToGoal)) < 0.2) {
			if (sensorflag &= (1 << SENSOR_0_BIT)) {
				if (((sensorflag &= (1 << SENSOR_1_BIT)) == 0)
						&& (turnLeftflag > 20)) {
					turnRight();
				}
				if (((sensorflag &= (1 << SENSOR_2_BIT)) == 0)
						&& (turnRightflag > 20)) {
					turnLeft();
				}
			} else {
				goStraight();
			}

		} else if (angDiff(heading, bearingToGoal) < 0) { // Turn  left.
			if (((sensorflag &= (1 << SENSOR_2_BIT)) == 0)
					&& (turnRightflag > 20)) {
				turnLeft();
			} else if ((sensorflag &= (1 << SENSOR_0_BIT)) == 0) {
				goStraight();
			} else {
				cmd_pub.publish(cmd);
			}
		} else if (angDiff(heading, bearingToGoal) > 0) { // Turn  right.
			if (((sensorflag &= (1 << SENSOR_1_BIT)) == 0)
					&& (turnLeftflag > 20)) {
				turnRight();
			} else if ((sensorflag &= (1 << SENSOR_0_BIT)) == 0) {
				goStraight();
			} else {
				cmd_pub.publish(cmd);
			}
		}
	}

}
void selectBehaviour() {
	// Add all forces
	switch (sensorflag) {
	case 1: // Turn left
		turnFwdLeft();
		break;
	case 2: // Turn left
		turnLeft();
		break;
	case 3: // Turn left
		turnLeft();
		break;
	case 4: // Turn Right
		turnRight();
		break;
	case 5: // go straight
		goStraight();
		break;
	case 6: // Turn right
		turnRight();
		break;
	case 7: //ReverseAndLeft
		reverseAndLeft();
	default: //movetogoal
		cout << "Move To Goal" << endl;
		moveToGoal();
	}

}
void updateFlag() {
	executionCount--;
	cout << "0;Sensor flag: " << sensorflag << endl;
	cout << "0;Sensor flag new: " << sensorflag_new << endl;
	cout << "0;Execution count: " << executionCount << endl;
	if ((sensorflag != sensorflag_new) && (sensorflag_new != 0)) {
		executionCount = 10;
		sensorflag |= sensorflag_new;
	}
	if (executionCount <= 0) {
		sensorflag = sensorflag_new;
		executionCount = 10;
	}
	cout << "1;Sensor flag:" << sensorflag << endl;
	cout << "1;Execution count: " << executionCount << endl;
	sensorflag_new = 0;
}
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {

	ros::init(argc, argv, "Reactive");

	ros::NodeHandle n;

	cmd_pub = n.advertise < geometry_msgs::Twist > ("/my_robot/cmd_vel", 1);

	ros::Subscriber sub0 = n.subscribe("ir", 100, irCallback);
	ros::Subscriber odom_sub = n.subscribe("/my_robot/odom", 10, odomCallback);
//  ros::Subscriber sub3 = n.subscribe("ir3", 1, irCallback3);
//  ros::Subscriber sub4 = n.subscribe("ir4", 1, irCallback4);

	ros::Rate loop_rate(50);
	goal.x = 10;
	while (ros::ok()) {
		if (dataflag != 15) {
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

