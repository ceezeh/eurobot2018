/*
 * grasper.cpp
 *
 *  Created on: Apr 6, 2018
 *      Author: deeplearning
 */
#include "ros/ros.h"
#include "grasp/grasper.hpp"
#include <sstream>
#include <mutex>
#include <cmath>
#include "std_msgs/Int16MultiArray.h"
#include "grasp/wiringSerial.h"

using namespace grasper;
using namespace arm;
using namespace std;

mutex moveMutex, cubeMutex;

Grasper::Grasper(ros::NodeHandle &n) :
		ballHeight(.58) {
	this->nh = n;
	this->newCube = this->newGoal = this->moveComplete = this->newOdom = false;
	cubeTimeout = ros::Duration(5);
	// TODO: Load these dynamically.

	PLACE_ID = "place";
	PICK_ID = "pick";
	command = "";

	/*
	 * Load ROS Constants.
	 */

	this->move_pub = nh.advertise<geometry_msgs::PoseStamped>(
			"/my_robot/goal", 1);

	// create options for subscriber and pass pointer to our custom queue
	ros::SubscribeOptions ops = ros::SubscribeOptions::create<
			geometry_msgs::PoseStamped>("/my_robot/arm_instructions", // topic name
			10, // queue length
			boost::bind(Grasper::goalCallback, this, _1), // callback
			ros::VoidPtr(), // tracked object, we don't need one thus NULL
			&queue // pointer to callback queue object
			);

	this->goalSub = nh.subscribe(ops);

	ops = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>(
			"/cube_position", // topic name
			10, // queue length
			boost::bind(Grasper::cubeCallback, this, _1), // callback
			ros::VoidPtr(), // tracked object, we don't need one thus NULL
			&queue // pointer to callback queue object
			);

	this->cubeSub = nh.subscribe(ops);

	ops = ros::SubscribeOptions::create<std_msgs::Int8>("/task_status", // topic name
			10, // queue length
			boost::bind(Grasper::moveCallback, this, _1), // callback
			ros::VoidPtr(), // tracked object, we don't need one thus NULL
			&queue // pointer to callback queue object
			);
	this->statusSub = nh.subscribe(ops);
	this->spinner = new ros::AsyncSpinner(0, &this->queue);
	this->spinner->start();

	serialAddress = "/dev/ttyACM0"; // Name of the UART port on the Raspberry pi
	this->serialfd = serialOpen(serialAddress.c_str(), 9600);
	// Open serial port.
}

/*
 * Commands received are in the robot's body frame.
 *
 * We assume that the robot is not moving when it receives a plan.
 * Thus, as long as the object does not move, the command is valid.
 *
 */
void Grasper::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_t) {
	stringstream ss(goal_t->header.frame_id);
	string command, extraInfo;

	std::getline(ss, command, ',');
	std::getline(ss, extraInfo, ',');

	if (command == PLACE_ID || command == PICK_ID) { // Only allow goto commands to pass.
		cout << "Received new command.." << endl;
		newGoal = true;
		target.color = extraInfo;
		target.pose = geometry_msgs::Pose(goal_t->pose);
		this->command = command;
	}
}

void Grasper::cubeCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
	if (msg->header.frame_id == target.color) {
		cubeMutex.lock();
		newCube = 0;
		target.pixel = cv::Vec2i(msg->point.x, msg->point.y);
		newCube = true;
		cubeMutex.unlock();

	}
}
void Grasper::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_t) {
	odom = nav_msgs::Odometry(*odom_t);
	newOdom = true;
}

void Grasper::moveCallback(const std_msgs::Int8::ConstPtr& status) {
	if (status->data == 1) {
		moveMutex.lock();
		moveComplete = 1;
		moveMutex.unlock();
	}
}
bool Grasper::checkMoveComplete() {
	moveMutex.lock();
	bool res = moveComplete;
	moveComplete = 0;
	moveMutex.unlock();
}

/*
 * We want to get a cube that arrives after this function call.
 * The risk is that, we may not detect the cube again.
 * We will need to think of ways to mitigate this.
 * Return value of 1 signifies success.
 */
bool Grasper::getCube() {
	newCube = 0;
	bool timedOut = false;
	ros::Time starttime = ros::Time::now();
	while (!newCube) {
		if (ros::Time::now() - starttime < cubeTimeout) {
			timedOut = true;
		}
	}
	cubeMutex.lock();
	if (timedOut)
		return 0;
	else
		return 1;
	cubeMutex.unlock();
}

void Grasper::processCmd() {
	if (newGoal) {
		// Verify location of object.
		if (command == this->PICK_ID) { // TODO: Add time constraint.
			// Pick instruction.
			if (!getCube()) {
				// Do something to improve object detection
			} else {
				// Determine if the object is in workspace.
				Distance d = this->arm.checkReachability(
						Pose3D(target.pose.position.x, target.pose.position.y,
								target.pose.position.z));
				bool ret = 1;
				// If not move robot close enough.
				if (d != (Distance ) { 0, 0 }) {
					geometry_msgs::Pose2D p;
					p.x = d.lin;
					p.theta = d.ang;
					move_pub.publish(p);
					moveMutex.lock();
					moveComplete = 0;
					moveMutex.unlock();
					while (!moveComplete&& ros::ok())
						;
					ret = getCube();
				}

				if (ret) {
					// Calculate IK and send to arm.
					arm::Pose3D targetPose = pixelToBody(target.pixel,
							getDiscreteHeight(target.pose.position.z));
					arm::JointAngles j = this->arm.calculateIK(targetPose);
					double gripAngle = 0;
					j.gripper = gripAngle;
					// Send to Arduino node.
					sendJointAngles(j,true);

				} else {
					// Do something to improve object detection
				}
			}
		} else if (command == this->PLACE_ID) {
			// Fine motion adjustment before placing.
			// Convert pose to body frame.
			arm::Pose3D tpose = target.pose;
			globalToBody(&tpose);
			arm::Pose3D t = this->arm.checkReachability(tpose);
			bool ret = 1;
			// If not move robot close enough.
			if (t != (arm::Pose3D) { 0, 0 }) {
				geometry_msgs::PoseStamped p;
				bodyToGlobal(&t);
				p.pose.position.x = t.x;
				p.pose.position.y = t.y;
				p.pose.orientation = getQuaternion(t.theta);
				move_pub.publish(p);
				while (!moveComplete)
					;
			}
			arm::JointAngles j = this->arm.calculateIK(tpose);
			double gripAngle = 90;
			j.gripper = gripAngle;
			// Send to Arduino node
			sendJointAngles(j,false);
		}
	}
}

int Grasper::getDiscreteHeight(double height) {
	return std::floor(height / this->ballHeight);
}
const double realMap[3][6][2] = { { { 16, -12.5 }, { 16, -8.5 }, { 16, -5.5 }, {
		16, 0 }, { 16, 5.5 }, { 16, 9.5 } }, { { 25, -12.5 }, { 25, -8.5 }, {
		25, -5.5 }, { 25, 0 }, { 25, 5.5 }, { 25, 9.5 } }, { { 30, -12.5 }, {
		30, -8.5 }, { 30, -5.5 }, { 30, 0 }, { 30, 5.5 }, { 30, 9.5 } } };

const double pixelMap[3][6][3][2] = { { { { 537, 356 }, { 561, 336 },
		{ 600, 306 } }, { { 450, 360 }, { 461, 339 }, { 490, 310 } }, { { 383,
		361 }, { 394, 340 }, { 403, 309 } }, { { 262, 367 }, { 254, 345 }, {
		251, 314 } }, { { 138, 368 }, { 115, 349 }, { 88, 319 } }, {
		{ 60, 371 }, { 31, 350 }, { -20, 314 } } }, { { { 506, 188 },
		{ 522, 153 }, { 548, 105 } },
		{ { 430, 189 }, { 443, 153 }, { 458, 105 } }, { { 373, 186 },
				{ 379, 152 }, { 388, 104 } }, { { 271, 190 }, { 268, 156 }, {
				268, 106 } }, { { 167, 188 }, { 144, 152 }, { 130, 104 } }, { {
				91, 189 }, { 72, 155 }, { 42, 107 } } }, { { { 491, 110 }, {
		505, 72 }, { 531, 20 } }, { { 424, 111 }, { 434, 74 }, { 454, 20 } }, {
		{ 371, 113 }, { 382, 74 }, { 391, 21 } }, { { 276, 114 }, { 276, 75 }, {
		271, 22 } }, { { 180, 114 }, { 166, 75 }, { 155, 24 } }, { { 110, 115 },
		{ 95, 76 }, { 76, 24 } } } };

arm::Pose3D Grasper::pixelToBody(cv::Vec2i pixel, int height) {

	arm::Pose3D p = { 0, 0, 0 };
	// Interpolate.
	for (int j = 0; j < 5; j++) { // for pixel x axis.
		for (int i = 0; i < 2; i++) { // for pixel y axis.

			// At height 0, pixel corresponds to ?
			cout << "Val at [i:" << i << ", j:" << j << "] = ("
					<< pixelMap[i][j][height][0] << ","
					<< pixelMap[i][j][height][1] << ")" << endl;
			if (pixelMap[i][j][height][1] >= pixel[1]
					& pixelMap[i + 1][j][height][1] < pixel[1]
					& pixelMap[i][j][height][0] >= pixel[0]
					& pixelMap[i][j + 1][height][0] < pixel[0]) {
				// Get xratio

				float yratio = float(pixel[0] - pixelMap[i][j][height][0])
						/ float(
								pixelMap[i][j + 1][height][0]
										- pixelMap[i][j][height][0]);
				cout << "pixelMap[i][j][height][0]: "
						<< pixelMap[i][j][height][0]
						<< ", pixelMap[i][j+1][height][0]: "
						<< pixelMap[i][j + 1][height][0] << endl;
				float xratio = float(pixel[1] - pixelMap[i][j][height][1])
						/ float(
								pixelMap[i + 1][j][height][1]
										- pixelMap[i][j][height][1]);
				cout << "pixelMap[i][j][height][1]: "
						<< pixelMap[i][j][height][1]
						<< ", pixelMap[i+1][j][height][1]: "
						<< pixelMap[i + 1][j][height][1] << endl;

				float y = (realMap[i][j + 1][1] - realMap[i][j][1]) * yratio
						+ realMap[i][j][1];
				cout << "yratio: " << yratio << ", realMap[i][j][1]: "
						<< realMap[i][j][1] << ", realMap[i][j+1][1]: "
						<< realMap[i][j + 1][1] << endl;
				float x = (realMap[i + 1][j][0] - realMap[i][j][0]) * xratio
						+ realMap[i][j][0];
				cout << "xratio: " << xratio << ", realMap[i][j][0]: "
						<< realMap[i][j][0] << ", realMap[i+1][j][0]: "
						<< realMap[i + 1][j][0] << endl;

				p.x = x;
				p.y = y;
				p.z = target.pose.position.z;
				return p;
			}
		}
	}
	return arm::Pose3D(0, 0, 0);
}

void Grasper::sendJointAngles(JointAngles j, bool isClose) {
	unsigned char direction = 100; //open grip direction.
	unsigned char BASE = 254;
	unsigned char ELBOW0 = 253;
	unsigned char ELBOW1 = 252;
	unsigned char ELBOW2 = 251;
	unsigned char WRIST = 250;
	unsigned char GRASP = 249;

	// Ensure joints are well formatted.
	// First convert to degrees.
	j.shoulder1 *= (180 / M_PI);
	j.shoulder2 *= (180 / M_PI);
	j.elbow1 *= (180 / M_PI);
	j.elbow2 *= (180 / M_PI);
	j.wrist *= (180 / M_PI);
	j.gripper *= (180 / M_PI);

	int8_t s1 =
			(abs(j.shoulder1) > 180) ?
					std::copysign(127, j.shoulder1) : j.shoulder1 & 255;
	int8_t s2 =
			(abs(j.shoulder2) > 180) ?
					std::copysign(127, j.shoulder2) : j.shoulder2 & 255;
	int8_t e1 =
			(abs(j.elbow1) > 180) ?
					std::copysign(127, j.elbow1) : j.elbow1 & 255;
	int8_t e2 =
			(abs(j.elbow2) > 180) ?
					std::copysign(127, j.elbow2) : j.elbow2 & 255;
	int8_t w =
			(abs(j.wrist) > 180) ? std::copysign(127, j.wrist) : j.wrist & 255;
	int8_t g =
			(abs(j.gripper) > 180) ?
					std::copysign(127, j.gripper) : j.gripper & 255;

	if (isClose) { // Grasp 0 is open...
		unsigned char buffer[15] = { 49, GRASP, 0, BASE, s1, ELBOW0, s2, ELBOW1,
				e1, ELBOW2, e2, WRIST, w, GRASP, 255 };
		writeBytes(this->serialfd, buffer, 14);
	} else {
		unsigned char buffer[15] = { 49, BASE, s1, ELBOW0, s2,
				ELBOW1, e1, ELBOW2, e2, WRIST, w, GRASP, g, 100,100 }; /// 100is empty command just to make the command length uniform
		writeBytes(this->serialfd, buffer, 14);
	}


}
/*
 * T is the transformation matrix;
 */
void Grasper::globalToBody(Pose3D *rpoint) {
	Pose3D T = Pose3D(odom.pose.pose.position.x, odom.pose.pose.position.y,
			getYaw(odom.pose.pose.orientation));
	rpoint->x -= T.x;
	rpoint->y -= T.y;
	float x0 = cos(-T.theta) * rpoint->x - sin(-T.theta) * rpoint->y;
	float y0 = sin(-T.theta) * rpoint->x + cos(-T.theta) * rpoint->y;
	rpoint->x = x0;
	rpoint->y = y0;
	rpoint->theta = angDiff(rpoint->theta, getYaw(odom.pose.pose.orientation));
}

void Grasper::bodyToGlobal(Pose3D *rpoint) {
	Pose3D T = Pose3D(odom.pose.pose.position.x, odom.pose.pose.position.y,
			getYaw(odom.pose.pose.orientation));

	float x0 = cos(T.theta) * rpoint->x - sin(T.theta) * rpoint->y;
	float y0 = sin(T.theta) * rpoint->x + cos(T.theta) * rpoint->y;

	rpoint->x = T.x + x0;
	rpoint->y = T.y + y0;

	rpoint->theta = angDiff(rpoint->theta, -getYaw(odom.pose.pose.orientation));
}
