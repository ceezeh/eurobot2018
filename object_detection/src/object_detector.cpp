/*
 * object_detector.cpp
 *
 *  Created on: Mar 29, 2018
 *      Author: deeplearning
 */

#include "object_detection/object_detector.hpp"

#include <algorithm>
#include <ros/ros.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>

using namespace objectDetector;
using namespace std;
using namespace cv;

int lowH = 8;							// Set Hue
int highH = 15;

int lowS = 0;							// Set Saturation
int highS = 245;

int lowV = 84;							// Set Value
int highV = 237;

void ObjectDetector::goalCallback(
		const geometry_msgs::PoseStamped::ConstPtr& goal_t) {
	if (goal_t->header.frame_id == READPLAN_ID) { // Only allow goto commands to pass.
		runPlanDetection();
	}
}

std::ostream& objectDetector::operator<<(std::ostream& os,
		const objectDetector::ColorProfile & c) {
	os << "[Color: " << c.name << ", H: " << c.hs[0] << ",S: " << c.hs[1]
			<< "]";
	return os;
}
const double realMap[3][6][2] = { { { 16, -12.5 }, { 16, -8.5 }, { 16, -5.5 }, {
		16, 0 }, { 16, 5.5 }, { 16, 9.5 } }, { { 25, -12.5 }, { 25, -8.5 }, {
		25, -5.5 }, { 25, 0 }, { 25, 5.5 }, { 25, 9.5 } }, { { 30, -12.5 }, {
		30, -8.5 }, { 30, -5.5 }, { 30, 0 }, { 30, 5.5 }, { 30, 9.5 } } };

const double pixelMap[3][6][3][2] = { { { { 537, 356 }, { 561, 336 },
		{ 600, 306 } }, { { 450, 360 }, { 461, 339 }, { 490, 310 } }, { { 383,
		361 }, { 394, 340 }, { 403, 309 } }, { { 262, 367 }, { 254, 345 }, {
		251, 314 } }, { { 138, 368 }, { 115, 349 }, { 88, 319 } }, {
		{ 60, 371 }, { 31, 350 }, { -20, 314 } } }, { { { 506, 188 },
		{ 522, 153 }, { 548, 105 } },
		{ { 430, 189 }, { 443, 153 }, { 458, 105 } }, { { 373, 186 },
				{ 379, 152 }, { 388, 104 } }, { { 271, 190 }, { 268, 156 }, {
				268, 106 } }, { { 167, 188 }, { 144, 152 }, { 130, 104 } }, { {
				91, 189 }, { 72, 155 }, { 42, 107 } } }, { { { 491, 110 }, {
		505, 72 }, { 531, 20 } }, { { 424, 111 }, { 434, 74 }, { 454, 20 } }, {
		{ 371, 113 }, { 382, 74 }, { 391, 21 } }, { { 276, 114 }, { 276, 75 }, {
		271, 22 } }, { { 180, 114 }, { 166, 75 }, { 155, 24 } }, { { 110, 115 },
		{ 95, 76 }, { 76, 24 } } } };

Vec2f objectDetector::convertPixelToBody(Vec2i pixel, int height) {

	// Interpolate.
	for (int j = 0; j < 5; j++) { // for pixel x axis.
		for (int i = 0; i < 2; i++) { // for pixel y axis.

			// At height 0, pixel corresponds to ?
			cout << "Val at [i:" << i << ", j:" << j << "] = ("
					<< pixelMap[i][j][height][0] << ","
					<< pixelMap[i][j][height][1] << ")" << endl;
			if (pixelMap[i][j][height][1] >= pixel[1]
					& pixelMap[i + 1][j][height][1] < pixel[1]
					& pixelMap[i][j][height][0] >= pixel[0]
					& pixelMap[i][j + 1][height][0] < pixel[0]) {
				// Get xratio

				float yratio = float(pixel[0] - pixelMap[i][j][height][0])
						/ float(
								pixelMap[i][j + 1][height][0]
										- pixelMap[i][j][height][0]);
				cout << "pixelMap[i][j][height][0]: "
						<< pixelMap[i][j][height][0]
						<< ", pixelMap[i][j+1][height][0]: "
						<< pixelMap[i][j + 1][height][0] << endl;
				float xratio = float(pixel[1] - pixelMap[i][j][height][1])
						/ float(
								pixelMap[i + 1][j][height][1]
										- pixelMap[i][j][height][1]);
				cout << "pixelMap[i][j][height][1]: "
						<< pixelMap[i][j][height][1]
						<< ", pixelMap[i+1][j][height][1]: "
						<< pixelMap[i + 1][j][height][1] << endl;

				float y = (realMap[i][j + 1][1] - realMap[i][j][1]) * yratio
						+ realMap[i][j][1];
				cout << "yratio: " << yratio << ", realMap[i][j][1]: "
						<< realMap[i][j][1] << ", realMap[i][j+1][1]: "
						<< realMap[i][j + 1][1] << endl;
				float x = (realMap[i + 1][j][0] - realMap[i][j][0]) * xratio
						+ realMap[i][j][0];
				cout << "xratio: " << xratio << ", realMap[i][j][0]: "
						<< realMap[i][j][0] << ", realMap[i+1][j][0]: "
						<< realMap[i + 1][j][0] << endl;

				return Vec2f(x, y);
			}
		}
	}
	return Vec2f(0, 0);
}

ObjectDetector::ObjectDetector(ros::NodeHandle &nh,
		image_transport::ImageTransport &it, int ct1, int ct2, int blurSize,
		int minArea, int r) :
		it_(it) {
	this->nh = nh;
	this->ct1 = ct1;
	this->ct2 = ct2;
	this->blurSize = blurSize;
	this->minArea = minArea;
	this->r = r;
	this->colorDistThreshold = 20;
//	colorProfiles.push_back( { Vec2d(18, 163), "My Orange" });
	colorProfiles.push_back( { Vec2d(9, 205), "Orange" });
	colorProfiles.push_back( { Vec2d(0, 0), "White" });
	colorProfiles.push_back( { Vec2d(22, 255), "Yellow" });

	colorProfiles.push_back( { Vec2d(48, 157), "Green" });
	colorProfiles.push_back( { Vec2d(98, 255), "Blue" });
	colorProfiles.push_back( { Vec2d(119, 32), "Black" });

	cubeWorker = planWorker = NULL;
	detectingPlan = detectingCube = newImg = false;

	this->imgSub = this->it_.subscribe("/image_rect_color", 2,
			&ObjectDetector::imageCb, this);

	imgPub = this->nh.advertise<geometry_msgs::PointStamped>("/cube_position",
			10);
	planPub = this->nh.advertise<std_msgs::String>("/construction_plan", 10);
	planAcquisitionTime = ros::Duration(10);

	ros::SubscribeOptions ops = ros::SubscribeOptions::create<
			geometry_msgs::PoseStamped>("/my_robot/goal", // topic name
			10, // queue length
			boost::bind(&ObjectDetector::goalCallback, this, _1), // callback
			ros::VoidPtr(), // tracked object, we don't need one thus NULL
			&queue // pointer to callback queue object
			);

	// Odom
	this->plan_sub = this->nh.subscribe(ops);

	// Spinner
	this->spinner = new ros::AsyncSpinner(0, &queue);
	this->spinner->start();

}

void ObjectDetector::getColorCube(Mat img, ColorProfile c) {

	cout << "Check2" << endl;
	Vec2i p = this->getCube(img, c);
	cout << "Check2" << endl;
	if (p[0] > 0 && p[1] > 0) {
		putText(img, c.name, cvPoint(p[0], p[1]), FONT_HERSHEY_COMPLEX_SMALL,
				0.8, cvScalar(200, 200, 250), 1, CV_AA);
	}

}

Vec2i ObjectDetector::getCube(Mat img, ColorProfile c) {
	cv::namedWindow("Original", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("Threshold", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("Result", CV_WINDOW_AUTOSIZE);
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
	bool foundCube = false;
	int cubeIndx = -1;
	const int scaling = 2;
//	Mat drawing = Mat::zeros(img.size(), CV_8UC1);
	Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
			rng.uniform(0, 255));

	Vec2d hs = c.hs;
	hs[1] /= scaling;
	for (size_t i = 0; i < contours.size(); i++) {
//		drawContours(drawing, contours, (int) i, color, 1, 8, vector<Vec4i>(),
//				0, Point());
		/*
		 * Identify color of contour.
		 */
		Mat mask = Mat::zeros(img.size(), CV_8UC1);
		drawContours(mask, contours, (int) i, Scalar(255), CV_FILLED);
		drawContours(drawing, contours, (int) i, Scalar(255), CV_FILLED);

		cv::cvtColor(img.clone(), hsvImg, CV_BGR2HSV);
		Scalar average = mean(hsvImg, mask);

		cout << average << endl;

		Vec2d t;
		t[0] = average[0];
		t[1] = average[1] / scaling;
		double n = cv::norm(t, hs);
		cout << "Norm to " << c.name << ": " << n << endl;
		if (n < this->colorDistThreshold) {
			foundCube = true;
			cubeIndx = i;
		}
	}
	cv::imshow("Original", img);
	cv::imshow("Threshold", threshImg);
	cv::imshow("Result", drawing);
	cv::waitKey(1);
	if (foundCube) {
		Moments mu = moments(contours[cubeIndx], false);

		Vec2i v = { mu.m10 / mu.m00, mu.m01 / mu.m00 };
//		-0.1098x + 66.97
//		Vec2i v = {(mu.m01  / mu.m00)*-0.1098 + 66.97, (mu.m10 / mu.m00)*-0.1098 + 66.97};

		cout << "Cube coordinate: " << v << endl;
		return v;
	} else {
		return {-999,-999};
	}
}

void ObjectDetector::detectPlan(Mat img, std::vector<ColorProfile>& result,
		std::set<int>& xposes) {

	cv::namedWindow("Original", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("Threshold", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("Result", CV_WINDOW_AUTOSIZE);

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

	Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
			rng.uniform(0, 255));
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
		drawContours(drawing, contours, (int) contours.size() - 1, color, 1, 8,
				vector<Vec4i>(), 0, Point());

	}

	Mat hsvImg;

// Create a set to stor color order.

	int scaling = 2;
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
		t[1] = average[1] / scaling;
		int minNorm = 9e3;
		int colorIndx = -1;
		for (int j = 0; j < this->colorProfiles.size(); j++) {
			ColorProfile c = this->colorProfiles[j];

			c.hs[1] /= scaling;
			double n = cv::norm(t, c.hs);
			cout << "Norm to " << c.name << ": " << n << endl;

			if (n < this->colorDistThreshold) {
				if (n < minNorm) {
					minNorm = n;
					colorIndx = j;
					cout << "MIN NORM!: " << minNorm << endl;
				}
			}
		}

		if (colorIndx < 0) {
			cout << "Color index less than 0" << endl;
			continue;
		}
		ColorProfile c = this->colorProfiles[colorIndx];
		if (c == result[0] || c == result[1] || c == result[2]) {
			cout << "Color already in list..." << endl;
			continue;
		}

		if (minNorm < this->colorDistThreshold) {
			cout << "Check.." << endl;
			Moments mu = moments(contours[i], false);
			int x = int(mu.m10 / mu.m00); // We assume camera's roll is horizontal and not slanted or vertical.
			xposes.insert(x);
			int indx = std::distance(xposes.begin(),
					std::find(xposes.begin(), xposes.end(), x));
			cout << "x: " << x << " at index: " << indx << endl;
			if (indx < 3 && indx >= 0) {

				result[indx] = c;
				cout << "inserting color: " << c << "at index: " << indx
						<< endl;
			}
		}
		cout << "Check2.5" << endl;
	}

	cv::imshow("Original", img);
	cv::imshow("Threshold", threshImg);
	cv::imshow("Result", drawing);
	cv::waitKey(1);
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
	cv::namedWindow("Original", CV_WINDOW_AUTOSIZE);
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
		vector<vector<Point> > contours;
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
		vector<vector<Point> > contours0;

		findContours(threshImg, contours0, hierarchy, RETR_TREE,
				CHAIN_APPROX_SIMPLE, Point(0, 0));
//		contours.resize(contours0.size());

		for (size_t i = 0; i < contours0.size(); i++) {

			vector<Point> cnt;
			approxPolyDP(Mat(contours0[i]), cnt,
					cv::arcLength(cv::Mat(contours0[i]), true) * r / 255, true);
			if (std::fabs(cv::contourArea(contours0[i])) < minArea
					|| !cv::isContourConvex(cnt)
					|| std::fabs(cv::contourArea(contours0[i])) > 6e4)
				continue;

			if ((cnt.size() < 4) || (cnt.size() > 6)) // Remove non-cube shapes
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
			Vec2i p = this->getCube(this->img, c);
			if (p[0] > 0 && p[1] > 0) {
				//publish result.
				geometry_msgs::PointStamped pose;
				pose.header.stamp = ros::Time::now();
				pose.header.frame_id = c.name;
				pose.point.x = p[0];
				pose.point.y = p[1];
//				Vec2f pt = convertPixelToBody(p,
//						Vec2d(this->img.cols, this->img.rows));
				// Transform this point from body to global.
//				cout << "Real pose: " << pt << endl;
				imgPub.publish(pose);
			}
		}
	}
}
void ObjectDetector::runCubeDetection() {
	stopCubeDetection();
	detectingCube = true;
	cubeWorker = new std::thread(&ObjectDetector::cubeDetection, this);
}
void ObjectDetector::stopCubeDetection() {
	detectingCube = false;
	usleep(10000);
	delete cubeWorker;
}

void ObjectDetector::planDetection() {
	ros::Time start = ros::Time::now();

	std::set<int> xposes;

	std::vector<ColorProfile> profiles = { { Vec2d(-1, -1), "" }, { Vec2d(-1,
			-1), "" }, { Vec2d(-1, -1), "" } };

	while (ros::ok() && detectingPlan) {
//			&& (ros::Time::now() - start < this->planAcquisitionTime)) {
		bool complete = true;
		this->getImage();
		this->detectPlan(this->img, profiles, xposes);
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
		cout << "Current plans: " << profiles[0] << ", " << profiles[1] << ", "
				<< profiles[2] << endl;
		if (complete) {
			cout << "Publishing plan" << endl;
			detectingPlan = false;
			//publish profile;
			std_msgs::String p;
			p.data = plan;
			planPub.publish(p);
		}
	}
	cout << "End of detection" << endl;
}

void ObjectDetector::runPlanDetection() {
	detectingPlan = false;
	usleep(10000);
	delete planWorker;
	detectingPlan = true;
	planWorker = new std::thread(&ObjectDetector::planDetection, this);
}

void ObjectDetector::calibrateDistanceDetection() {
// Just output an image with lines at predetermined locations.
	cv::namedWindow("Original", CV_WINDOW_AUTOSIZE);
	char escapeKey = 0;
	while (ros::ok() && escapeKey != 27) {
		getImage();

		int stepr = this->img.rows / 20;
		int stepc = this->img.cols / 30;

		int lineThickness = 1;
		for (int i = 0; i < this->img.rows; i += stepr) {
			cv::line(this->img, Point(0, i), Point(this->img.cols - 1, i),
					(255, 255, 0), lineThickness);
		}

		for (int i = 0; i < this->img.cols; i += stepc) {
			cv::line(this->img, Point(i, 0), Point(i, this->img.rows - 1),
					(255, 255, 0), lineThickness);
		}
		imshow("Original", this->img);
		escapeKey = cv::waitKey(1);
	}
}
