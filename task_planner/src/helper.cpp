/*
 * helper.cpp
 *
 *  Created on: Mar 20, 2018
 *      Author: deeplearning
 */

#include "task_planner/helper.h"


float angDiff(float a1, float a2) {
	float a = wraparound(a1) - wraparound(a2);
	a = fmod((a + M_PI), (2 * M_PI)) - M_PI;
	return wraparound(a);
}

geometry_msgs::Quaternion getQuaternion(float yaw) {
	tf::Quaternion qt;
	geometry_msgs::Quaternion q;

	tf::Matrix3x3 m;
	m.setEulerZYX(yaw, 0, 0);
	m.getRotation(qt);
	tf::quaternionTFToMsg(qt, q);
	return q;

}

float getYaw(geometry_msgs::Quaternion q) {
	tf::Quaternion qt;
	tf::quaternionMsgToTF(q, qt);
	tf::Matrix3x3 m(qt);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	return yaw;
}


float wraparound(float ang) { // [-pi, pi]
	if (equals(ang, 0) || equals(fabs(ang), M_PI))
		return ang;
	if ((ang <= M_PI) && (ang >= -M_PI))
		return ang;
	if (ang > M_PI) {
		ang -= 2 * M_PI;
	}
	if (ang < -M_PI) {
		ang += 2 * M_PI;
	}
	return wraparound(ang);
}
