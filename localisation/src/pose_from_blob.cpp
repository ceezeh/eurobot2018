/*
 * pose_from_blob.cpp
 *
 *  Created on: Mar 9, 2018
 *      Author: Chinemelu Ezeh
 */

/**
 * OpenCV SimpleBlobDetector Example
 *
 * Copyright 2015 by Satya Mallick <spmallick@gmail.com>
 *
 */
#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <cstdio>
#include <math.h>
#include <vector>
#include <cstdlib>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <geometry_msgs/Point.h>
#include <boost/circular_buffer.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "localisation/helper.h"

using namespace cv;
using namespace std;
cv::VideoCapture leftcap;
cv::VideoCapture rightcap;
int lowH = 21;							// Set Hue
int highH = 30;

int lowS = 156;							// Set Saturation
int highS = 255;

int lowV = 121;							// Set Value
int highV = 225;

float rads_per_pixel_left = 0.00189226;
float rads_per_pixel_right = 0.00232038;
boost::circular_buffer<float> cb(10);
cv::Mat leftImg, rightImg;		// Input image

bool newLeft, newRight;

// Todo: Update Campus dynamically!!!!
int getBeacon(TrackerHelper tracker, Mat img, string title, float& xaxis) {

	IplImage* img_t = cvCreateImage(cvSize(img.cols, img.rows),
	IPL_DEPTH_8U, img.channels());

	int newResult = tracker.getNumDetected(img_t, tracker.tracker, img.cols,
			img.rows, (char *) title.c_str());

	Mat campose = (Mat_<float>(8, 1) << 0, 0, 0, 0, 0, 0, 1, 1);
	Mat pose = Mat::zeros(8, 1, CV_32F);
	if (newResult > 0) {
		tracker.get6DOFMarkerPose(tracker.tracker, campose, pose, 0);
		xaxis = pose.at<float>(0, 0);
		return 0;
	} else {
		return 1;
	}
}

int getCircle(cv::Vec3f &vcircle, Mat img, string title) {
	cv::Mat hsvImg;				// HSV Image
	cv::Mat threshImg;			// Thresh Image
	std::vector<cv::Vec3f> v3fCircles;
	cv::cvtColor(img, threshImg, CV_BGR2GRAY);
	cv::threshold(threshImg, threshImg, 220, 255, THRESH_BINARY);
	cv::GaussianBlur(threshImg, threshImg, cv::Size(1, 1), 0);	//Blur Effect
	cv::dilate(threshImg, threshImg, 0);	// Dilate Filter Effect
	cv::erode(threshImg, threshImg, 0);	// Erode Filter Effect

	// fill circles vector with all circles in processed image
	cv::HoughCircles(threshImg, v3fCircles, CV_HOUGH_GRADIENT, 2,
			threshImg.rows / 4, 100, 20, 0, 0);	// algorithm for detecting circles

	for (int i = 0; i < v3fCircles.size(); i++) {// We assume our circle is the largest one detected.

//		std::cout << "Ball position X = " << v3fCircles[i][0]// x position of center point of circle
//				<< ",\tY = " << v3fCircles[i][1]// y position of center point of circle
//				<< ",\tRadius = " << v3fCircles[i][2] << "\n";// radius of circle
		if (i == 0) {
			vcircle = v3fCircles[i];
		} else if (v3fCircles[i][2] > vcircle[2]) {
			vcircle = v3fCircles[i];
		}

		// draw small green circle at center of object detected
		cv::circle(img,	// draw on original image
				cv::Point((int) v3fCircles[i][0], (int) v3fCircles[i][1]),// center point of circle
				3,	// radius of circle in pixels
				cv::Scalar(0, 255, 0),	// draw green
				CV_FILLED);	// thickness

		// draw red circle around object detected
		cv::circle(img,	// draw on original image
				cv::Point((int) v3fCircles[i][0], (int) v3fCircles[i][1]),// center point of circle
				(int) v3fCircles[i][2],	// radius of circle in pixels
				cv::Scalar(0, 0, 255),	// draw red
				3);	// thickness

	}
	string t = title + " threshold";
	cv::namedWindow(title, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(t, CV_WINDOW_AUTOSIZE);
	cv::imshow(title, img);	// show windows
	cv::imshow(t, threshImg);
	cv::waitKey(1);
	if (v3fCircles.empty())
		return 1;
	else
		return 0;

}
int getUndistortedImage(Mat& im, cv::VideoCapture & cap, Mat cameraMatrix,
		Mat distCoeffs) {
	Mat im1;
	bool blnFrameReadSuccessfully = cap.read(im1); // get next frame

	if (!blnFrameReadSuccessfully || im1.empty()) { // if frame read unsuccessfully
		std::cout << "error: frame can't read \n";	// print error message
		return 1;	// jump out of loop
	}

// -----------------Undistort Image Here-----------------
	Mat map1, map2;
	initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
			getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, im1.size(), 1,
					im1.size(), 0), im1.size(),
			CV_16SC2, map1, map2);
	remap(im1, im, map1, map2, INTER_LINEAR);
	return 0;
}

int getimages() {
	while (ros::ok() && (!newRight || !newLeft)) {
		ros::spinOnce();
		usleep(1000);
	}
	newRight = newLeft = 0;
}
int getRectangleWidth(Mat im, string title = "default") {
	cv::Mat threshImg, img2;			// Thresh Image
	int length = 0;
	bool first = true;
	cv::cvtColor(im, threshImg, CV_BGR2GRAY);
	adaptiveThreshold(threshImg, img2, 255, ADAPTIVE_THRESH_GAUSSIAN_C,
			THRESH_BINARY, 11, 0);

	vector < vector<Point> > contours;
	vector<Point> approx;
	cv::findContours(img2.clone(), contours, CV_RETR_EXTERNAL,
			CV_CHAIN_APPROX_SIMPLE);

	cout << "Number of contours" << contours.size() << endl;
	for (int i = 0; i < contours.size(); i++) {
		// Approximate contour with accuracy proportional
		// to the contour perimeter
		cv::approxPolyDP(cv::Mat(contours[i]), approx,
				cv::arcLength(cv::Mat(contours[i]), true) * 0.02, true);

		// Skip small or non-convex objects
		if (std::fabs(cv::contourArea(contours[i])) < 1500
				|| !cv::isContourConvex(approx))
			continue;

		if (approx.size() == 4) {
			cv::Rect r = cv::boundingRect(contours[i]);
			if (first) {
				length = r.width;
				first = false;
			} else if (length < r.width) {
				length = r.width;
			}
			int sz = (int) contours[i].size();
			const Point* p = &contours[i][0];
			polylines(im, &p, &sz, 1, true, Scalar(0, 255, 0), 3, LINE_AA);
		}
	}
	cv::imshow(title, im);
	cv::waitKey(1);
	return length;
}

void leftCb(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	leftImg = cv_ptr->image;
	newLeft = 1;
}

void rightCb(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	rightImg = cv_ptr->image;
	newRight = 1;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "Beacon");
	ros::NodeHandle nh;

	image_transport::ImageTransport it_(nh);
	image_transport::Subscriber right_sub = it_.subscribe(
			"/right/image_rect_color", 1, rightCb);
	image_transport::Subscriber left_sub = it_.subscribe(
			"/left/image_rect_color", 1, leftCb);

	cv::Mat hsvImg;				// HSV Image
	cv::Mat threshImg;			// Thresh Image

	std::vector<cv::Vec3f> v3fCircles;// 3 element vector of floats, this will be the pass by reference output of HoughCircles()

	if (argc > 1) {
		std::cout << argv[1] << endl;
		if ((strcmp(argv[1], "c") == 0) || (strcmp(argv[1], "b") == 0)) {

			char charCheckForEscKey = 0;

			// HUE for YELLOW is 21-30.
			// Adjust Saturation and Value depending on the lighting condition of the environment as well as the surface of the object.

			while (charCheckForEscKey != 27 && ros::ok()) {	// until the Esc is pressed or webcam connection is lost
				getimages();

				if (strcmp(argv[1], "b") == 0) {
					cv::cvtColor(leftImg, threshImg, CV_BGR2GRAY);
					cv::threshold(threshImg, threshImg, 220, 255,
							THRESH_BINARY);
				} else {
					//-------------------------------------------------------

					cv::cvtColor(leftImg, hsvImg, CV_BGR2HSV);// Convert Original Image to HSV Thresh Image

					cv::inRange(hsvImg, cv::Scalar(lowH, lowS, lowV),
							cv::Scalar(highH, highS, highV), threshImg);
				}
				cv::GaussianBlur(threshImg, threshImg, cv::Size(1, 1), 0);//Blur Effect
				cv::dilate(threshImg, threshImg, 0);	// Dilate Filter Effect
				cv::erode(threshImg, threshImg, 0);	// Erode Filter Effect

				// fill circles vector with all circles in processed image
				cv::HoughCircles(threshImg, v3fCircles, CV_HOUGH_GRADIENT, 2,
						threshImg.rows / 4, 100, 50, 10, 800);// algorithm for detecting circles

				for (int i = 0; i < v3fCircles.size(); i++) {// for each circle

					std::cout << "Ball position X = " << v3fCircles[i][0]// x position of center point of circle
							<< ",\tY = " << v3fCircles[i][1]// y position of center point of circle
							<< ",\tRadius = " << v3fCircles[i][2] << "\n";// radius of circle

					// draw small green circle at center of object detected
					cv::circle(
							leftImg,	// draw on original image
							cv::Point((int) v3fCircles[i][0],
									(int) v3fCircles[i][1]),// center point of circle
							3,	// radius of circle in pixels
							cv::Scalar(0, 255, 0),	// draw green
							CV_FILLED);	// thickness

					// draw red circle around object detected
					cv::circle(
							leftImg,	// draw on original image
							cv::Point((int) v3fCircles[i][0],
									(int) v3fCircles[i][1]),// center point of circle
							(int) v3fCircles[i][2],	// radius of circle in pixels
							cv::Scalar(0, 0, 255),	// draw red
							3);	// thickness
				}

				// declare windows
				cv::namedWindow("left", CV_WINDOW_AUTOSIZE);
				cv::namedWindow("threshImg", CV_WINDOW_AUTOSIZE);

				/* Create trackbars in "threshImg" window to adjust according to object and environment.*/
				cv::createTrackbar("LowH", "threshImg", &lowH, 359);//Hue (0 - 179)
				cv::createTrackbar("HighH", "threshImg", &highH, 359);
				cv::createTrackbar("LowS", "threshImg", &lowS, 255);//Saturation (0 - 255)
				cv::createTrackbar("HighS", "threshImg", &highS, 255);

				cv::createTrackbar("LowV", "threshImg", &lowV, 255);//Value (0 - 255)
				cv::createTrackbar("HighV", "threshImg", &highV, 255);

				cv::imshow("left", leftImg);	// show windows
				cv::imshow("threshImg", threshImg);

				charCheckForEscKey = cv::waitKey(1);// delay and get key press
			}
			cout << "lowH:" << lowH << ", highH:" << highH << ", lowS:" << lowS
					<< ", highS:" << highS << ", lowV:" << lowV << ", highV: "
					<< highV << endl;
		} else if (strcmp(argv[1], "angle") == 0) {
			// calculate pixel per degree.
			// insert distance.
			// insert width.
			// output pixel per degree.
			int dist, width, left_pixel_width, right_pixel_width;
			left_pixel_width = right_pixel_width = 0;
			cout << "Insert distance to rectangle in mm" << endl;
			cin >> dist;

			cout << "Insert width of rectangle in mm" << endl;
			cin >> width;

			while ((left_pixel_width == 0 || right_pixel_width == 0)
					&& (ros::ok())) {
				getimages();

				// Detect rectangle from left camera.
				left_pixel_width = getRectangleWidth(leftImg, "Leftcheck");
				// Detect rectangle from right camera.
				right_pixel_width = getRectangleWidth(rightImg, "rightcheck");

				cv::imshow("left", leftImg);
				cv::imshow("right", rightImg);
				cv::waitKey(1);

			}
			cout << "Left Pixel Width" << left_pixel_width << endl;
			cout << "Right Pixel Width" << right_pixel_width << endl;
			// Calculate degree per pixel.
			float ang = 2 * atan2f(width, (2 * dist));
			rads_per_pixel_left = float(ang) / float(left_pixel_width);
			rads_per_pixel_right = float(ang) / float(right_pixel_width);
			cout << "rads_per_pixel_left" << rads_per_pixel_left << endl;
			cout << "rads_per_pixel_right" << rads_per_pixel_right << endl;
		}
	}

	ros::Publisher pub = nh.advertise<geometry_msgs::Point>("/beacon", 10);

	ros::Rate loop_rate(10);

	ofstream outputFile;
	// create a name for the file output
	std::string filename = "data.csv";
	outputFile.open(filename);
	outputFile << "interoccular , horzpixel, diff , distance, truth"
			<< std::endl;

	TrackerHelper helper;
	getimages();

	string calibpath =
			"~/packages/ARToolKitPlus-2.2.1/sample/data/no_distortion.cal";
	string ar_configpath =
			"~/packages/ARToolKitPlus-2.2.1/sample/data/markerboard_480-499.cfg";
	// Get the camera's pose

	if (helper.configTracker(leftImg.cols, leftImg.rows, calibpath.c_str(),
			ar_configpath.c_str()) != 0) {
		cout << "Error! Cannot configure tracker." << endl;
		exit (EXIT_FAILURE);
	}

	while (ros::ok()) {

		cout << "Check1" << endl;
		getimages();
//		cout << "Check" << endl;
		cv::Vec3f leftCircle, rightCircle;
//		if (getCircle(leftCircle, leftImg, "Left") != 0)
//			continue;
//		if (getCircle(rightCircle, rightImg, "Right") != 0)
//			continue;

		float leftX, rightX;
		leftX=rightX=0;
		if (getBeacon(helper, leftImg, "Left", leftX) != 0)
			continue;
		if (getBeacon(helper, rightImg, "Right", rightX) != 0)
					continue;
		// Calculate distance (mm) to object.
		float B = 60;
		float x0 = leftImg.cols;
		float Phi0 = rads_per_pixel_left * x0;
		float diff = leftX - rightX;
		float distance = B * x0 / (2 * tan(Phi0 / 2) * diff);
		distance = 0.2471 * distance + 207.8;
//		float ang_left = rads_per_pixel_left * float(leftCircle.val[0]);
//		float ang_right = rads_per_pixel_right * float(rightCircle.val[0]);
//
//		cout << "ang_left:" << ang_left << "ang_right:" << ang_right << endl;
//
//		float distance = interocular * sin(ang_left) * sin(ang_right)
//				/ (sin(M_PI - ang_left - ang_right));

		cb.push_back(distance);
		float max = 0;
		float min = 1e9;
		float ave = 0;
		for (int i = 0; i < cb.size(); i++) {
			if (cb[i] > max)
				max = cb[i];
			if (cb[i] < min)
				min = cb[i];
			ave += cb[i];
		}
		ave /= cb.size();
		float sd = 0;
		for (int i = 0; i < cb.size(); i++) {
			sd += pow(cb[i] - ave, 2);
		}
		sd = sqrt(sd / cb.size());
		cout << "max: " << max << ", min: " << min << ", sd: " << sd
				<< ", ave: " << ave << ", last:" << distance << endl;

		float truth = 0;
		cout << "insert distance: " << endl;
		cin >> truth;

		outputFile << B << "," << leftImg.cols << "," << diff << "," << distance
				<< "," << truth << std::endl;
		geometry_msgs::Point p;
		p.x = distance;
		pub.publish(p);
//		cout << "Distance" << distance << endl;

		loop_rate.sleep();
	}
	outputFile.close();
	return (0);
}
