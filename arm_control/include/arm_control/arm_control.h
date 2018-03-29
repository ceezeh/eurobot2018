/*
 * arm_control.h
 *
 *  Created on: Feb 6, 2018
 *      Author: deeplearning
 */

#ifndef ARM_CONTROL_INCLUDE_ARM_CONTROL_ARM_CONTROL_H_
#define ARM_CONTROL_INCLUDE_ARM_CONTROL_ARM_CONTROL_H_
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <string>
// important messages
#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/GetJointProperties.h>
#include "arm_control/joint.h"
#include "arm_control/helper.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <task_planner/helper.h> // Contains definitions of task ids
#include <queue>


void grasp(const string cmd);
void cmdCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_t);
void moveArm(const geometry_msgs::PoseStamped  goal);
void moveArmDefault();
void scheduleAction();

#endif /* ARM_CONTROL_INCLUDE_ARM_CONTROL_ARM_CONTROL_H_ */
