/*
 * grab_images.cpp
 *
 *  Created on: Mar 9, 2018
 *      Author: deeplearning
 */
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;
int main(int argc, char** argv) {
	ros::init(argc, argv, "grab_images");
	ros::NodeHandle nh;
	cv::VideoCapture leftcap("/dev/video2");
	cv::VideoCapture rightcap("/dev/video3");

	Mat left, right;
	cv_bridge::CvImage left_msg;
	cv_bridge::CvImage right_msg;

	image_transport::ImageTransport it(nh);
	image_transport::Publisher left_pub = it.advertise("camera/left/image_raw",
			1);
	image_transport::Publisher right_pub = it.advertise(
			"camera/right/image_raw", 1);

	ros::Rate loop_rate(5);

	cv::namedWindow("Left", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("Right", CV_WINDOW_AUTOSIZE);

	while (ros::ok()) {
		leftcap >> left;
		rightcap >> right;

		cv::imshow("Left", left);
		cv::imshow("Right", right);

		cv::waitKey(20);
		sensor_msgs::ImagePtr left_msg = cv_bridge::CvImage(std_msgs::Header(),
				"bgr8", left).toImageMsg();
		sensor_msgs::ImagePtr right_msg = cv_bridge::CvImage(std_msgs::Header(),
				"bgr8", right).toImageMsg();

		left_pub.publish(left_msg);
		right_pub.publish(right_msg);

		loop_rate.sleep();

	}
}

