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
#include "tf/transform_datatypes.h"
#include "nav_msgs/OccupancyGrid.h"
#include <unistd.h>
#include <cmath>
#define equals(x, y) (fabs(x-y) < 0.00001)

float angDiff(float a1, float a2);
geometry_msgs::Quaternion getQuaternion(float yaw);
float getYaw(geometry_msgs::Quaternion q);
float wraparound(float ang);

using namespace std;

const string GOTO_ID = "goto";
const string REVERSE_ID = "reverse";
const string PICK_ID = "pick";
const string PLACE_ID = "place";
const string READPLAN_ID = "read";
const string OPENFLAPS_ID = "openflaps";
const string CLOSEFLAPS_ID = "closeflaps";
const string FLAPHOLD_ID = "flaphold";
const string FLAPHOLD_ID = "SEARCHhold";

class ConstructionSequence {
public:
	int getNextReadPlanTask(geometry_msgs::PoseStamped &plan);
	int getNextPush1stBlockTask( geometry_msgs::PoseStamped &task);
	geometry_msgs::PoseStamped getNextReadPlanTask();
	geometry_msgs::PoseStamped getNextAssembleTask();
	ConstructionSequence();
private:
	int planPtr;
	int push1stBlocksPtr;
	int assemble1Ptr;
	vector<geometry_msgs::PoseStamped> readPlanTasks;
};

#endif /* TASK_PLANNER_INCLUDE_TASK_PLANNER_HELPER_H_ */
