/*
 * grasp.cpp
 *
 *  Created on: Apr 5, 2018
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

#include "geometry_msgs/Pose2D.h"
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include "task_planner/helper.h"
#include "grasp/grasper.hpp"
#include <mutex>
using namespace std;

int main(int argc, char** argv) {
	ros::init(argc, argv, "grasp");

	ros::NodeHandle nh;

	grasper::Grasper grasper(nh);

	ros::Rate loop_rate(10);
	while (ros::ok())
		;
	return 0;
}
