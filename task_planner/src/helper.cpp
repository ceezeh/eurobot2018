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

ConstructionSequence::ConstructionSequence() {
	planPtr = push1stBlocksPtr = assemble1Ptr = 0;

	geometry_msgs::PoseStamped temp;
	temp.header.frame_id = GOTO_ID;
	temp.pose.position.x = 0.6;
	temp.pose.position.y = 1.5;
	temp.pose.orientation = getQuaternion(M_PI);
	readPlanTasks.push_back(temp);

	temp.header.frame_id = READPLAN_ID;
	readPlanTasks.push_back(temp);
}

int ConstructionSequence::getNextReadPlanTask(
		geometry_msgs::PoseStamped &plan) {
	if (planPtr >= readPlanTasks.size()) {
		return -1;
	} else {
		plan = readPlanTasks[planPtr++];
		return 0;
	}
}

/*
 * TODO: We may need a way to tell the robot to stop moving rather than
 * avoid obstacles if it is pushing some blocks.
 */
int ConstructionSequence::getNextPush1stBlockTask(
		geometry_msgs::PoseStamped &task) {
	geometry_msgs::PoseStamped temp;
	temp.header.frame_id = GOTO_ID;

	if (planPtr > 8) {
		return -1;
	} else {
		switch (push1stBlocksPtr++) {
		case 0: // 1st Task.
			temp.pose.position.x = 1.1;
			temp.pose.position.y = .85;
			temp.pose.orientation = getQuaternion(0);
			task = temp;
			break;
		case 1:
			temp.pose.position.x = 1.3;
			temp.pose.position.y = .3;
			temp.pose.orientation = getQuaternion(M_PI);
			task = temp;
			break;
		case 2:
			temp.header.frame_id = OPENFLAPS_ID;
			task = temp;
			break;
		case 3:
			temp.pose.position.x = .7;
			temp.pose.position.y = .3;
			temp.pose.orientation = getQuaternion(M_PI);
			task = temp;
			break;
		case 4:
			temp.header.frame_id = FLAPHOLD_ID;
			task = temp;
			break;
		case 5:
			temp.pose.position.x = .3;
			temp.pose.position.y = .21;
			temp.pose.orientation = getQuaternion(M_PI);
			task = temp;
			break;
		case 6:
			temp.header.frame_id = REVERSE_ID;
			temp.pose.position.x = .5;
			temp.pose.position.y = .21;
			temp.pose.orientation = getQuaternion(M_PI);
			task = temp;
			break;
		case 7:
			temp.header.frame_id = CLOSEFLAPS_ID;
			task = temp;
			break;
		case 8:
			temp.pose.position.x = .5;
			temp.pose.position.y = .21;
			temp.pose.orientation = getQuaternion(M_PI);
			task = temp;
			break;
		case 9:
			temp.header.frame_id = PICK_ID;
			task = temp;
			break;
		}
		return 0;
	}
}
