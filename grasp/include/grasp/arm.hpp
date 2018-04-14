/*
 * arm.hpp
 *
 *  Created on: Apr 6, 2018
 *      Author: deeplearning
 */

#ifndef GRASP_INCLUDE_GRASP_ARM_HPP_
#define GRASP_INCLUDE_GRASP_ARM_HPP_
#include "geometry_msgs/Pose.h"
#include "task_planner/helper.h"

namespace arm {
struct Distance {
	float lin;
	float ang;

	bool operator==(const Distance &other) const {
		// Compare the values, and return a bool result.
		if (equals(this->lin, other.lin) && equals(this->ang, other.ang)) {
			return true;
		} else {
			return false;
		}
	}

	bool operator!=(const Distance &other) const {
		return !((*this) == other);
	}
};

struct JointAngles {
	float shoulder1;
	float shoulder2;
	float elbow1;
	float elbow2;
	float wrist;
	float gripper;
};
struct Pose3D {
	float x;
	float y;
	float z;
	float theta;
};

class Arm {

public:
	Arm ();
	JointAngles calculateIK(Pose3D);
	Pose3D Arm::checkReachability(Pose3D p);
private:
	Pose3D arm_origin;
	float max_radius;
	float right_fov_lim_y;
};
}
#endif /* GRASP_INCLUDE_GRASP_ARM_HPP_ */
