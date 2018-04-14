/*
 * helper.h
 *
 *  Created on: Feb 3, 2018
 *      Author: deeplearning
 */

#ifndef PS5_YXL1450_INCLUDE_PS5_YXL1450_HELPER_H_
#define PS5_YXL1450_INCLUDE_PS5_YXL1450_HELPER_H_

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "cppopt/optimization.h"
#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <string>
#include <gazebo_msgs/GetJointProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Point.h>


using namespace alglib;

void nlcfunc1_jac(const real_1d_array &x, real_1d_array &fi, real_2d_array &jac,
		void *ptr);
std::vector<double> calcInvK(geometry_msgs::Point gripper_pos);


#endif /* PS5_YXL1450_INCLUDE_PS5_YXL1450_HELPER_H_ */
