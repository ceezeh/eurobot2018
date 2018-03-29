/*
 * object_detector.cpp
 *
 *  Created on: Mar 29, 2018
 *      Author: deeplearning
 */

#include "object_detection/object_detector.hpp"
#include <set>
#include <algorithm>
#include <ros/ros.h>
using namespace objectDetector;
using namespace std;
using namespace cv;

int lowH = 8;							// Set Hue
int highH = 15;

int lowS = 0;							// Set Saturation
int highS = 245;

int lowV = 84;							// Set Value
int highV = 237;

std::ostream& objectDetector::operator<<(std::ostream& os, const objectDetector::ColorProfile & c) {
	os << "[Color: " << c.name << ", H: " << c.hs[0] << ",S: " << c.hs[1]
			<< "]";
	return os;
}
ObjectDetector::ObjectDetector(ros::NodeHandle &nh,image_transport::ImageTransport &it, int ct1, int ct2,
		int blurSize, int minArea, int r):it_(it) {
	this->nh = nh;
	this->ct1 = ct1;
	this->ct2 = ct2;
	this->blurSize = blurSize;
	this->minArea = minArea;
	this->r = r;
	colorProfiles.push_back( { Vec2d(18, 163), "Orange" });
	colorProfiles.push_back( { Vec2d(0, 0), "White" });
	colorProfiles.push_back( { Vec2d(44, 255), "Yellow" });

	colorProfiles.push_back( { Vec2d(48, 157), "Green" });
	colorProfiles.push_back( { Vec2d(98, 255), "Blue" });
	colorProfiles.push_back( { Vec2d(119, 32), "Black" });

	cubeWorker = planWorker = NULL;
	detectingPlan = detectingCube = newImg = false;

	this->imgSub = this->it_.subscribe("/right/image_rect_color", 2,
			&ObjectDetector::imageCb, this);
	imgPub = this->nh.advertise<geometry_msgs::PointStamped>("/cube_position",
			10);
	planPub = this->nh.advertise<std_msgs::String>("/construction_plan", 10);
	planAcquisitionTime = ros::Duration(10);
}
void ObjectDetector::getColorCube(Mat img, ColorProfile c) {

	cout << "Check2" << endl;
	Vec3i p = this->getCube(img, c);
	cout << "Check2" << endl;
	if (p[0] > 0 && p[1] > 0) {
		putText(img, c.name, cvPoint(p[0], p[1]), FONT_HERSHEY_COMPLEX_SMALL,
				0.8, cvScalar(200, 200, 250), 1, CV_AA);
	}

}

Vec3i ObjectDetector::getCube(Mat img, ColorProfile c) {
	RNG rng(12345);
	Mat threshImg;
	Canny(img.clone(), threshImg, ct1, ct2, 3);

	cv::GaussianBlur(threshImg, threshImg, cv::Size(blurSize, blurSize), 0); //Blur Effect
	cv::dilate(threshImg, threshImg, 0);	// Dilate Filter Effect
	cv::erode(threshImg, threshImg, 0);	// Erode Filter Effect

//
	Mat drawing = Mat::zeros(img.size(), CV_8UC3);
	vector<vector<Point> > contours0, contours;
	vector<Vec4i> hierarchy;

	findContours(threshImg, contours0, hierarchy, RETR_TREE,
			CHAIN_APPROX_SIMPLE, Point(0, 0));
//	contours.resize(contours0.size());

	for (size_t i = 0; i < contours0.size(); i++) {

		vector<Point> cnt;
		approxPolyDP(Mat(contours0[i]), cnt,
				cv::arcLength(cv::Mat(contours0[i]), true) * r / 255, true);

		if (std::fabs(cv::contourArea(contours0[i])) < minArea
				|| !cv::isContourConvex(cnt)
				|| std::fabs(cv::contourArea(contours0[i])) > 2e4)
			continue;

		if ((cnt.size() < 4) || (cnt.size() > 7)) // Remove non-cube shapes
			continue;

		contours.push_back(cnt);

	}

	Mat hsvImg;
	Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
			rng.uniform(0, 255));
	for (size_t i = 0; i < contours.size(); i++) {
//		drawContours(drawing, contours, (int) i, color, 1, 8, vector<Vec4i>(),
//				0, Point());
		/*
		 * Identify color of contour.
		 */
		Mat mask = Mat::zeros(img.size(), CV_8UC1);
		drawContours(mask, contours, (int) i, Scalar(255), CV_FILLED);
		cv::cvtColor(img.clone(), hsvImg, CV_BGR2HSV);
		Scalar average = mean(hsvImg, mask);
		cout << average << endl;

		Vec2d t;
		t[0] = average[0];
		t[1] = average[1];
		double n = cv::norm(t, c.hs);
		cout << "Norm to " << c.name << ": " << n << endl;
		if (n < 15) {
			Moments mu = moments(contours[i], false);
			return {mu.m10/mu.m00 , mu.m01/mu.m00, 0};
		}
	}
	return {-999,-999,-999};

}

std::vector<objectDetector::ColorProfile> ObjectDetector::detectPlan(Mat img) {

	RNG rng(12345);
	Mat threshImg;
	Canny(img.clone(), threshImg, ct1, ct2, 3);

	cv::GaussianBlur(threshImg, threshImg, cv::Size(blurSize, blurSize), 0); //Blur Effect
	cv::dilate(threshImg, threshImg, 0);	// Dilate Filter Effect
	cv::erode(threshImg, threshImg, 0);	// Erode Filter Effect

//
	Mat drawing = Mat::zeros(img.size(), CV_8UC3);
	vector<vector<Point> > contours0, contours;
	vector<Vec4i> hierarchy;

	findContours(threshImg, contours0, hierarchy, RETR_TREE,
			CHAIN_APPROX_SIMPLE, Point(0, 0));
//	contours.resize(contours0.size());

	for (size_t i = 0; i < contours0.size(); i++) {

		vector<Point> cnt;
		approxPolyDP(Mat(contours0[i]), cnt,
				cv::arcLength(cv::Mat(contours0[i]), true) * r / 255, true);

		if (std::fabs(cv::contourArea(contours0[i])) < minArea
				|| !cv::isContourConvex(cnt)
				|| std::fabs(cv::contourArea(contours0[i])) > 2e4) // TODO; Examine this upperbound
			continue;

		if (cnt.size() != 4)  // Remove non-cube shapes
			continue;

		contours.push_back(cnt);

	}

	Mat hsvImg;
	Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
			rng.uniform(0, 255));

// Create a set to stor color order.
	std::set<int> xposes;

	std::vector<ColorProfile> result = { { Vec2d(-1, -1), "" }, { Vec2d(-1, -1),
			"" }, { Vec2d(-1, -1), "" } };
	for (size_t i = 0; i < contours.size(); i++) {
//		drawContours(drawing, contours, (int) i, color, 1, 8, vector<Vec4i>(),
//				0, Point());
		/*
		 * Identify color of contour.
		 */
		Mat mask = Mat::zeros(img.size(), CV_8UC1);
		drawContours(mask, contours, (int) i, Scalar(255), CV_FILLED);
		cv::cvtColor(img.clone(), hsvImg, CV_BGR2HSV);
		Scalar average = mean(hsvImg, mask);
		cout << average << endl;

		Vec2d t;
		t[0] = average[0];
		t[1] = average[1];
		for (int i = 0; i < this->colorProfiles.size(); i++) {
			ColorProfile c = this->colorProfiles[i];
			double n = cv::norm(t, c.hs);
			cout << "Norm to " << c.name << ": " << n << endl;
			if (n < 15) {
				Moments mu = moments(contours[i], false);
				int x = int(mu.m10 / mu.m00); // We assume camera's roll is horizontal and not slanted or vertical.
				xposes.insert(x);
				int indx = std::distance(xposes.begin(),
						std::find(xposes.begin(), xposes.end(), x));

				if (indx < 3) {
					result[indx] = c;
					cout << "inserting color: " << c << "at index: " << indx
							<< endl;
				}

			}
		}

	}
	return result;
}

void ObjectDetector::imageCb(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	img = cv_ptr->image;
	newImg = 1;
}
void ObjectDetector::calibrateThreshold() {
	RNG rng(12345);
	char charCheckForEscKey = 0;

	cv::namedWindow("Result", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("Color Threshold", CV_WINDOW_AUTOSIZE);
	cv::Mat hsvImg;				// HSV Image
	cv::Mat threshImg;			// Thresh Image

	int count = 0;
	cout << "Save image? Y N";

	while (charCheckForEscKey != 27 && ros::ok()) {

		blurSize = (blurSize % 2) ? blurSize : blurSize + 1;
		getImage();

		Mat threshold_output;
		vector < vector<Point> > contours;
		vector<Vec4i> hierarchy;

//			adaptiveThreshold(threshImg, threshImg, 255, ADAPTIVE_THRESH_GAUSSIAN_C,
//					THRESH_BINARY, 11, 0);
		Canny(img.clone(), threshImg, ct1, ct2, 3);
//		cv::cvtColor(rightImg.clone(), hsvImg, CV_BGR2HSV); // Convert Original Image to HSV Thresh Image
//
//		cv::inRange(hsvImg, cv::Scalar(lowH, lowS, lowV),
//				cv::Scalar(highH, highS, highV), threshImg);
		cv::GaussianBlur(threshImg, threshImg, cv::Size(blurSize, blurSize), 0); //Blur Effect
		cv::dilate(threshImg, threshImg, 0);	// Dilate Filter Effect
		cv::erode(threshImg, threshImg, 0);	// Erode Filter Effect

		Mat drawing = Mat::zeros(threshImg.size(), CV_8UC3);
		vector < vector<Point> > contours0;

		findContours(threshImg, contours0, hierarchy, RETR_TREE,
				CHAIN_APPROX_SIMPLE, Point(0, 0));
//		contours.resize(contours0.size());

		for (size_t i = 0; i < contours0.size(); i++) {

			vector<Point> cnt;
			approxPolyDP(Mat(contours0[i]), cnt,
					cv::arcLength(cv::Mat(contours0[i]), true) * r / 255, true);
			if (std::fabs(cv::contourArea(contours0[i])) < minArea
					|| !cv::isContourConvex(cnt)
					|| std::fabs(cv::contourArea(contours0[i])) > 2e4)
				continue;

			if ((cnt.size() < 4))//|| (cnt.size() > 5)) // Remove non-cube shapes
				continue;
			contours.push_back(cnt);

		}

		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
				rng.uniform(0, 255));
		for (size_t i = 0; i < contours.size(); i++) {
			drawContours(drawing, contours, (int) i, color, 1, 8,
					vector<Vec4i>(), 0, Point());
			/*
			 * Identify color of contour.
			 */
			Mat mask = Mat::zeros(threshImg.size(), CV_8UC1);
			drawContours(mask, contours, (int) i, Scalar(255), CV_FILLED);
			cv::cvtColor(img.clone(), hsvImg, CV_BGR2HSV);
			Scalar average = mean(hsvImg, mask);
			cout << average << endl;
		}

		cv::createTrackbar("CT1", "Result", &ct1, 255);	//Hue (0 - 179)
		cv::createTrackbar("CT2", "Result", &ct2, 255);
		cv::createTrackbar("minArea", "Result", &minArea, 5000);//Saturation (0 - 255)
		cv::createTrackbar("Arc Ratio", "Result", &r, 255);
		cv::createTrackbar("Blur Size", "Result", &blurSize, 255);

		/* Create trackbars in "threshImg" window to adjust according to object and environment.*/
		cv::createTrackbar("LowH", "Result", &lowH, 359);	//Hue (0 - 179)
		cv::createTrackbar("HighH", "Result", &highH, 359);
		cv::createTrackbar("LowS", "Result", &lowS, 255);//Saturation (0 - 255)
		cv::createTrackbar("HighS", "Result", &highS, 255);

		cv::createTrackbar("LowV", "Result", &lowV, 255);	//Value (0 - 255)
		cv::createTrackbar("HighV", "Result", &highV, 255);

		/*
		 * Change canny thresholds, minarea, arc ratio,
		 */
		cv::imshow("Original", img);
		cv::imshow("Color Threshold", threshImg);
		cv::imshow("Result", drawing);
		charCheckForEscKey = cv::waitKey(1);
	}
}

void ObjectDetector::getImage() {
	while (ros::ok() && !newImg) {
		ros::spinOnce();
		usleep(1000);
	}
	newImg = 0;
}

void ObjectDetector::cubeDetection() {
	while (ros::ok() && detectingCube) {
		this->getImage();
		for (int i = 0; i < this->colorProfiles.size(); i++) {
			objectDetector::ColorProfile c = this->colorProfiles[i];
			Vec3i p = this->getCube(this->img, c);
			if (p[0] > 0 && p[1] > 0) {
				//publish result.
				geometry_msgs::PointStamped pose;
				pose.header.stamp = ros::Time::now();
				pose.header.frame_id = c.name;
				pose.point.x = p[0];
				pose.point.y = p[1];
				pose.point.z = p[2];
				// Transform this point from body to global.

				imgPub.publish(pose);
			}
		}
	}
}
void ObjectDetector::runCubeDetection() {
	detectingCube = false;
	usleep(10000);
	delete cubeWorker;
	detectingCube = true;
	cubeWorker = new std::thread(&ObjectDetector::cubeDetection, this);
}

void ObjectDetector::planDetection() {
	ros::Time start = ros::Time::now();
	while (ros::ok() && detectingPlan
			&& (ros::Time::now() - start < this->planAcquisitionTime)) {
		bool complete = true;
		this->getImage();
		std::vector<ColorProfile> profiles = this->detectPlan(this->img);
		std::string plan;
		for (int i = 0; i < profiles.size(); i++) {
			ColorProfile p = profiles[i];
			plan = plan + "," + p.name;
			if (strcmp(p.name, "") == 0) {
				cout << "Plan is incomplete" << endl;
				complete = false;
				break;
			}
		}
		if (complete) {
			detectingPlan = false;
			//publish profile;
			std_msgs::String p;
			p.data = plan;
			planPub.publish(p);
		}
	}
}

void ObjectDetector::runPlanDetection() {
	detectingPlan = false;
	usleep(10000);
	delete planWorker;
	detectingPlan = true;
	planWorker = new std::thread(&ObjectDetector::planDetection, this);
}
