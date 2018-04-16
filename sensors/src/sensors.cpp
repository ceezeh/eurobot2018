/*
 * sensorsnflaps.cpp
 *
 *  Created on: Apr 16, 2018
 *      Author: deeplearning
 */
#include <ros/ros.h>
#include  "sensor_msgs/Image.h"
#include "geometry_msgs/PoseStamped.h"
#include "task_planner/helper.h"
#include "grasp/wiringSerial.h"
#include <string>
#include <vector>
#include <sstream>

#define MAX_SIZE 50
using namespace std;

int serialfd;
char buffer[2] = { 49, 0 };

//void flapsCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
//	if (msg->header.frame_id == OPENFLAPS_ID) {
//		buffer[1] = 100;
//		writeBytes(serialfd, buffer, 2);
//	} else if (msg->header.frame_id == CLOSEFLAPS_ID) {
//		buffer[1] = 250;
//		writeBytes(serialfd, buffer, 2);
//	} else if (msg->header.frame_id == FLAPHOLD_ID) {
//		buffer[1] = 115;
//		writeBytes(serialfd, buffer, 2);
//	}
//}

vector<int> readLine(int descriptor) {
	vector<int> result;
	stringstream ss;

	char temp = 0;
	for (int i = 0; i < MAX_SIZE; i++) {
		readBytes(descriptor, &temp, 1);
		if (temp == '\n') {
			break;
		}
		ss << temp;
	}
	while (ss.rdbuf()->in_avail()) {
		int res = 0;
		ss >> res;
		result.push_back(res);
	}
}

vector<int> readSensors() {
	vector<int> values = readLine(serialfd);
	while (values.size() != 8) {
		values = readLine(serialfd);
	}
	return values;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "sensorsnflaps");

	string serialAddress = "/dev/ttyACM0"; // Name of the UART port on the Raspberry pi
	serialfd = serialOpen(serialAddress.c_str(), 9600);

	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/my_robot/ir", 10);
//	ros::Subscriber sub = nh.subscribe("/my_robot/flaps", 10, flapsCallback);
	ros::Rate r(10); // 10 hz
	while (ros::ok()) {
		vector<int>  values = readSensors();
		sensor_msgs::Image sensor;
		for (int i = 0; i < 8; i++) {
			sensor.data.push_back(values[i]);
		}
		sensor.header.stamp = ros::Time();
		pub.publish(sensor);
		ros::spinOnce();
		r.sleep();
	}
}
