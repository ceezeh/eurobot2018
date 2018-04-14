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
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include "std_msgs/Int8.h"
#include <vector>
#include <thread>
#include <cstddef>
#include <iostream>
#include <string.h>
#include <set>
#include "task_planner/helper.h"
using namespace cv;

namespace objectDetector {
Vec2f convertPixelToBody(Vec2i pixel, int height);
struct ColorProfile {
	ColorProfile() {
		hs = Vec2d(0, 0);
		strcpy(this->name, "");
	}
	ColorProfile(Vec2d hs, char* name) {
		this->hs = hs;
		strcpy(this->name, name);
	}
	Vec2d hs;
	char name[10];
	bool operator==(const ColorProfile& c) {
		int ret = strcmp(this->name, c.name);
		if (ret == 0) {
			return true;
		} else {
			return false;
		}
	}
	friend std::ostream& operator<<(std::ostream& os, const ColorProfile & c);

};
std::ostream& operator<<(std::ostream& os, const ColorProfile & c);
class ObjectDetector {
public:
	ObjectDetector(ros::NodeHandle &nh, image_transport::ImageTransport &it,
			int ct1, int ct2, int blurSize, int minArea, int r);
	~ObjectDetector() {
		delete cubeWorker;
		delete planWorker;
		delete spinner;
	}
	void calibrateThreshold();
	void calibrateDistanceDetection();

	/*
	 * Returns an ordered list of the construction plan.
	 */

	/*
	 * Run threads for the following:
	 * 	1. Detecting each color from image.
	 * 	2. Extracting construction plan from image.
	 */

	void runCubeDetection();
	void stopCubeDetection();
	void runPlanDetection();
private:

	void getImage();
	void getColorCube(Mat img, ColorProfile c);
	void detectPlan(Mat img, std::vector<ColorProfile>& result,
			std::set<int>& xposes);
	void cubeDetection();
	void planDetection();

	Vec2i getCube(Mat img, ColorProfile c);
	int ct1, ct2, blurSize, minArea, r;
	std::vector<ColorProfile> colorProfiles;
	std::thread *cubeWorker, *planWorker;
	ros::NodeHandle nh;
	ros::Publisher imgPub;
	ros::Publisher planPub;
	ros::Duration planAcquisitionTime;
	image_transport::ImageTransport it_;
	image_transport::Subscriber imgSub;
	void imageCb(const sensor_msgs::ImageConstPtr& msg);
	void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_t);
	ros::AsyncSpinner* spinner;
	ros::CallbackQueue queue;
	ros::Subscriber plan_sub;

	Mat img, depthImg;

	bool detectingCube, detectingPlan, newImg;
	int colorDistThreshold;
};
}

#endif /* OBJECT_DETECTION_INCLUDE_OBJECT_DETECTION_OBJECT_DETECTOR_HPP_ */
