/*
 * detector.hpp
 *
 *  Created on: Mar 29, 2018
 *      Author: deeplearning
 */

#ifndef OBJECT_DETECTION_INCLUDE_OBJECT_DETECTION_OBJECT_DETECTOR_HPP_
#define OBJECT_DETECTION_INCLUDE_OBJECT_DETECTION_OBJECT_DETECTOR_HPP_
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>
#include <boost/circular_buffer.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <thread>
#include <cstddef>
#include <iostream>
#include <string.h>
using namespace cv;

namespace objectDetector {

struct ColorProfile {
	ColorProfile() {
		hs = Vec2d(0,0);
		strcpy(this->name,"");
	}
	ColorProfile(Vec2d hs,char* name){
		this->hs = hs;
		strcpy(this->name,name);
	}
	Vec2d hs;
	char name[10];
	friend std::ostream& operator<<(std::ostream& os, const ColorProfile & c);

};
std::ostream& operator<<(std::ostream& os, const ColorProfile & c);
class ObjectDetector {
public:
	ObjectDetector(ros::NodeHandle &nh, image_transport::ImageTransport &it, int ct1, int ct2, int blurSize,
			int minArea, int r);
	~ObjectDetector() {
		delete cubeWorker;
		delete planWorker;
	}
	void calibrateThreshold();

	/*
	 * Returns an ordered list of the construction plan.
	 */

	/*
	 * Run threads for the following:
	 * 	1. Detecting each color from image.
	 * 	2. Extracting construction plan from image.
	 */

	void runCubeDetection();
	void runPlanDetection();
private:

	void getImage();
	void getColorCube(Mat img, ColorProfile c);
	std::vector<ColorProfile> detectPlan(Mat);
	void cubeDetection();
	void planDetection();
	bool detectingCube, detectingPlan, newImg;
	Vec3i getCube(Mat img, ColorProfile c);
	int ct1, ct2, blurSize, minArea, r;
	std::vector<ColorProfile>  colorProfiles;
	std::thread *cubeWorker, *planWorker;
	ros::NodeHandle nh;
	ros::Publisher imgPub;
	ros::Publisher planPub;
	ros::Duration planAcquisitionTime;
	image_transport::ImageTransport it_;
	image_transport::Subscriber imgSub;
	void imageCb(const sensor_msgs::ImageConstPtr& msg);

	Mat img;
};
}

#endif /* OBJECT_DETECTION_INCLUDE_OBJECT_DETECTION_OBJECT_DETECTOR_HPP_ */
