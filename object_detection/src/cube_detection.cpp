/*
 * cube_detection.cpp
 *
 *  Created on: Mar 26, 2018
 *      Author: deeplearning
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
#include <iostream>
#include <stdio.h>
#include <thread>
#include <stdlib.h>
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <termios.h>    // POSIX terminal control definitions
#include <geometry_msgs/PointStamped.h>
#include "object_detection/object_detector.hpp"

using namespace cv;
using namespace std;
/*
 * Colors for competition
 *
 * Yellow #f7b500	HSV: 43.97º	100%	96.86
 * Green #61993b	HSV: 95.74º	61.44%	60%
 * Black 0e0e10		HSV: 240º	12.5%	6.27
 * Blue #007cb0 	HSV: 197.73º	100%	69.02%
 * Orange #d05d28	HSV: 18.93º	80.77%	81.57
 *
 *
 *
 * Orange cube detection.
 *
 int lowH = 8;							// Set Hue
 int highH = 15;

 int lowS = 0;							// Set Saturation
 int highS = 245;

 int lowV = 63;							// Set Value
 int highV = 237;

 Orange 2:

 * Wood brown cube detection.(TBC)
 *
 int lowH = 13;							// Set Hue
 int highH = 26;

 int lowS = 0;							// Set Saturation
 int highS = 177;

 int lowV = 99;							// Set Value
 int highV = 193;
 */

int main(int argc, char** argv) {
	ros::init(argc, argv, "cube_detection");
	cv::namedWindow("Original", CV_WINDOW_AUTOSIZE);

	ros::NodeHandle nh;
	image_transport::ImageTransport it_ = image_transport::ImageTransport(nh);

	int minArea = 4000;
	int r = 9;
	int ct1 = 50;
	int ct2 = 150;
	int blurSize = 7;

	objectDetector::ObjectDetector detector = objectDetector::ObjectDetector(nh,
			it_, ct1, ct2, blurSize, minArea, r);

	if (argc > 1) {
		if (strcmp(argv[1], "c") == 0) {
			detector.calibrateThreshold();
		}
	}

//	objectDetector::ColorProfile orange = { Vec2d(18, 163), "orange" };
//	objectDetector::ColorProfile white = { Vec2d(0, 0), "white" };

	detector.runCubeDetection();
	while (ros::ok())
		;
	return 0;
}

