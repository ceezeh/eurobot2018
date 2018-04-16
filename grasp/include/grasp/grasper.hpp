/*
 * grasper.hpp
 *
 *  Created on: Apr 5, 2018
 *      Author: deeplearning
 */

#ifndef GRASP_INCLUDE_GRASP_GRASPER_HPP_
#define GRASP_INCLUDE_GRASP_GRASPER_HPP_
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"

#include "std_msgs/Int8MultiArray.h"
#include "nav_msgs/Odometry.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Int8.h"
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <vector>
#include <cmath>
#include <string>
#include "grasp/arm.hpp"
/*
 * Controls an arm object to successfully grip cubes.
 */
namespace grasper {

struct Point {
	float x;
	float y;
};

struct Cube {
	string color;
	geometry_msgs::Pose pose;
	cv::Vec2i pixel;
};

class Grasper {
public:
	Grasper(ros::NodeHandle &n);

	~Grasper() {
		delete spinner;
	}
private:

	/*
	 * Pose is in body frame.
	 * Return linear and angular distance to move by so that object in reachable.
	 */

	void globalToBody(arm::Pose3D *rpoint);
	void bodyToGlobal(arm::Pose3D *rpoint);
	arm::Pose3D pixelToBody(cv::Vec2i pixel, int height);
	void processCmd();
	bool getCube();
	int getDiscreteHeight(double height);
	void moveCallback(const std_msgs::Int8::ConstPtr& status);
	void cubeCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
	void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_t);
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_t);
	ros::CallbackQueue queue;
	ros::AsyncSpinner* spinner;
	ros::Publisher move_pub;

	int serialfd;
	string serialAddress;

	nav_msgs::Odometry odom;
	bool newCube, newGoal, moveComplete, newOdom;
	bool checkMoveComplete();

	arm::Arm arm;

	Cube target;
	string command;
	const float ballHeight;

	ros::Duration cubeTimeout;
	ros::NodeHandle nh;
	ros::Subscriber goalSub, cubeSub,statusSub;

	void sendJointAngles(arm::JointAngles j, bool isClose);
	void sendFlapCommands(string);
};

}

#endif /* GRASP_INCLUDE_GRASP_GRASPER_HPP_ */
