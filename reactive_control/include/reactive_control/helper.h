/*
 * helper.cpp
 *
 *  Created on: Jan 2, 2018
 *      Author: deeplearning
 */

#ifndef REACTIVE_CONTROL_INCLUDE_REACTIVE_CONTROL_HELPER_H_
#define REACTIVE_CONTROL_INCLUDE_REACTIVE_CONTROL_HELPER_H_
#include "tf/transform_datatypes.h"
#include <unistd.h>
#define equals(x, y) (fabs(x-y) < 0.00001)
float angDiff(float a1, float a2);
float getYaw(geometry_msgs::Quaternion q);
float wraparound(float ang);


#endif /* REACTIVE_CONTROL_INCLUDE_REACTIVE_CONTROL_HELPER_H_ */
