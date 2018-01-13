/*
 * helper.cpp
 *
 *  Created on: Jan 13, 2018
 *      Author: deeplearning
 */

#ifndef TASK_PLANNER_INCLUDE_TASK_PLANNER_HELPER_H_
#define TASK_PLANNER_INCLUDE_TASK_PLANNER_HELPER_H_


//TODO Add more status flags
#define STATUS_NAV_COMPLETE 1

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include <sstream>
#include "string"
using namespace std;

const string GOTO_ID = "goto";


#endif /* TASK_PLANNER_INCLUDE_TASK_PLANNER_HELPER_H_ */
