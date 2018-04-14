/*
 * arm.cpp
 *
 *  Created on: Apr 6, 2018
 *      Author: deeplearning
 */

#include "grasp/arm.hpp"
#include "grasp/helper.h"
using namespace arm;

Arm::Arm() {
	max_radius = .2;
	right_fov_lim_y = -0.1;
	arm_origin = {-0.1, -.05};
}

/*
 * Pose is in body frame.
 * Return linear and angular distance to move by so that object in reachable.
 */
// TODO: Include consideration for z axis.
Pose3D Arm::checkReachability(arm::Pose3D p) {
	float dist = sqrt(pow(p.x - arm_origin.x, 2) + pow(p.y - arm_origin.y, 2));
	if (dist < max_radius && p.y > -right_fov_lim_y) {
		return {0,0};
	} else {
		// Try to keep the object at the centre of the screen.
		// Find rotation.
		float bufferzone = 0.1;
		float bearingToGoal = atan2(p.y - arm_origin.y, p.x - arm_origin.x);

		float d = dist - max_radius + bufferzone;
		return {d*cos(bearingToGoal),d*sin(bearingToGoal),0, bearingToGoal};
	}
}

JointAngles Arm::calculateIK(Pose3D p) {
	geometry_msgs::Point point;
	point.x = p.x;
	point.y = p.y;
	point.z = p.z;
	vector<double> jointAngles = calcInvK(point);
	JointAngles res = { jointAngles[0], jointAngles[1], jointAngles[2],
			jointAngles[3], jointAngles[4], 0 };
	return res;
}
