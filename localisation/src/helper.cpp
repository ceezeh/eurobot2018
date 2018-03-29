/*
 * helper.cpp
 *
 *  Created on: Mar 15, 2018
 *      Author: deeplearning
 */


#include "localisation/helper.h"


MyTimer::MyTimer() {
	maxduration = 0;
	aveduration = 0;
	last_duration = 0;
	count = 0;
}

void MyTimer::start() {
	t1 = high_resolution_clock::now();
}

void MyTimer::stop() {
	t2 = high_resolution_clock::now();
	last_duration = duration_cast < microseconds > (t2 - t1).count();
	update();
}

int MyTimer::getElapsed() {
	high_resolution_clock::time_point t = high_resolution_clock::now();
	int dur = duration_cast < microseconds > (t - t1).count();
	return dur;
}

int MyTimer::restart() {
	this->stop();
	int dur = this->getLastDuration();
	this->start();
	return dur;
}

void MyTimer::update() {

	if (last_duration > 14) {
		count++;

		aveduration = aveduration + (last_duration - aveduration) / count;
	}
	if (last_duration > maxduration) {
		maxduration = last_duration;
	}
}

int TrackerHelper::configTracker(int width, int height, const char* cal, const char* cfg ) {
	// write a function get camera frame size.
	tracker = new TrackerMultiMarker(width, height, 8, 6, 6, 6, 0);
	tracker->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);

	// load a camera file.
	if (!tracker->init(cal,	cfg, 1.0f, 1000.0f))
			{
		printf("ERROR: init() failed\n");
		exit(EXIT_FAILURE);
	}

	// tracker->getCamera()->printSettings();

	// the marker in the BCH test image has a thin border...
	tracker->setBorderWidth(0.125);

	// set a threshold. alternatively we could also activate automatic thresholding
	tracker->setThreshold(120);
	 // tracker->activateAutoThreshold(true);

	// let's use lookup-table undistortion for high-speed
	// note: LUT only works with images up to 1024x1024
	tracker->setUndistortionMode(ARToolKitPlus::UNDIST_LUT);

	// switch to simple ID based markers
	// use the tool in tools/IdPatGen to generate markers
	tracker->setMarkerMode(ARToolKitPlus::MARKER_ID_SIMPLE);
	return 0;
}

int TrackerHelper::getNumDetected(IplImage *img, TrackerMultiMarker *tracker,
		int width, int height, char * name) {
	int numDetected = 0;
	IplImage *greyImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

	IplImage *tempImg = cvCreateImage(cvSize(width, height), img->depth, 1);
	cvCvtColor(img, tempImg, CV_RGB2GRAY);
	cvAdaptiveThreshold(tempImg, greyImg, 255.0, CV_ADAPTIVE_THRESH_GAUSSIAN_C,
			CV_THRESH_BINARY, 51);
	numDetected = tracker->calc((unsigned char*) greyImg->imageData);

	IplImage* new_img = cvCreateImage(cvSize(640, 480), greyImg->depth, greyImg->nChannels);
	cvResize(greyImg, new_img);

	cvShowImage(name, new_img);
	cvWaitKey(1); // Wait for image to be rendered on screen. If not included, no image is shown.

	cvReleaseImage(&new_img);
	cvReleaseImage(&greyImg);
	cvReleaseImage(&tempImg);
	return numDetected;
//
}

void TrackerHelper::get6DOFMarkerPose(TrackerMultiMarker* tracker,	 Mat cam_pose,	Mat &marker_pose, int index) {
	ARToolKitPlus::ARMarkerInfo markerInfo = tracker->getDetectedMarker(index);
	marker_pose = Mat::zeros(8,1, CV_32FC1);
	ARFloat nOpenGLMatrix[16];
	ARFloat patternCentre[2] = { 0.0f, 0.0f };
	tracker->calcOpenGLMatrixFromMarker(&markerInfo, patternCentre, markerWidth,
			nOpenGLMatrix);
	Mat T = Mat(4, 4, CV_32FC1, (float *) nOpenGLMatrix);

	// Construct a rotation matrix from global to camera frame.
	// First construct rotation matrix from global to camera frame
	Mat Rc2g;

	Rc2g.create(3, 3, CV_32FC1);

	calRotYXZ(cam_pose.at<float>(3, 0), cam_pose.at<float>(4, 0),
			cam_pose.at<float>(5, 0), cam_pose.at<float>(6, 0),
			cam_pose.at<float>(7, 0), Rc2g);

	Mat Trans_t;
	Trans_t.create(3, 4, CV_32FC1);
	hconcat(Rc2g, cam_pose.rowRange(0, 3), Trans_t);
	Mat Tc2g;
	 Tc2g.create(4, 4, CV_32FC1);
	Mat off = (Mat_<float>(1, 4) << 0, 0, 0, 1);
	vconcat(Trans_t, off,  Tc2g);

	// cout << "marker pose: " <<endl<<marker_pose<<endl;

	Mat Tm2c = T.t();
	Mat Tm2g = Tc2g * Tm2c;
	Mat Rm2g = Tm2g.rowRange(0, 3).colRange(0, 3);
	getAnglesYXZ(marker_pose.at<float>(3, 0), marker_pose.at<float>(4, 0),
				marker_pose.at<float>(5, 0), marker_pose.at<float>(6, 0),marker_pose.at<float>(7, 0),Rm2g);
	cout << "check12 " << endl;

	// Calculate Translation
	Mat m = Tm2g.col(3).rowRange(0,3);
	cout << "check13 " << endl;
	cout << "marker pose: " <<endl<<marker_pose<<endl;
	Mat mTrans = marker_pose.rowRange(0, 3);
	cout << "check14 " << endl;
	m.copyTo(mTrans);

	cout << "check15 " << endl;
	cout << "Tm2g" << endl<<Tm2g << endl;
//	cout << "trans" << endl << Tm2c << endl << endl << "T" << endl<< T << endl<< endl<< "Rm2c" << endl << Rm2c<< endl<< "Rm2c_t" << endl << Rm2c_t<< endl << endl;
	// cout <<"markerpose: " << endl << marker_pose << endl << endl;

}
void TrackerHelper::calRotYXZ(float pitch, float yaw, float roll, float Ys, float Zs, Mat &R) {
	// Todo: Notice the transformation.
	float yd = sin(pitch);
	float C = abs(tan(yaw));
	float zd = sqrt((1-yd*yd)/(C*C + 1));
	float xd = 0;
	// Careful recovery of direction from tan
	if ((yaw >= 0) & (yaw < CV_PI / 2)) {
		xd = C* yd;; // xd is the sin part.
	} else if ((yaw >= CV_PI / 2) & (yaw < CV_PI)) {
		xd = C*zd;
		zd *= -1;
	} else if ((yaw >= -CV_PI) & (yaw < -CV_PI / 2)) {
		xd = -C*zd;
		zd *= -1;
	} else if ((yaw >= -CV_PI / 2) & (yaw < 0)) {
		xd = -C* zd;
	}

	// cout << "Zs "<< Zs << endl;
	Mat Zm = (Mat_<float>(3, 1) << xd, yd, zd);
	Zm*= Zs;
	zd = Zm.at<float>(2,0);
	xd = Zm.at<float>(0,0);
	Mat X0 = (Mat_<float>(3, 1) << zd, 0, -xd);
	Mat Y0 = Zm.cross(X0);

	Mat Ym = Y0 * cos(roll) + (Y0.cross(Zm) * sin(roll))
			+ (Zm * Zm.dot(Y0) * (1 - cos(roll)));
	Ym *= Ys;
	// cout << "Cal Rot:" << endl<< "Zm" << Zm << endl<<"X0" << X0 << endl << "Y0" << Y0 <<
			// endl << "Ym" << Ym<< endl;
	Mat Xm = Ym.cross(Zm);
	R.at<float>(0, 0) = Xm.at<float>(0, 0);
	R.at<float>(1, 0) = Xm.at<float>(1, 0);
	R.at<float>(2, 0) = Xm.at<float>(2, 0);

	R.at<float>(0, 1) = Ym.at<float>(0, 0);
	R.at<float>(1, 1) = Ym.at<float>(1, 0);
	R.at<float>(2, 1) = Ym.at<float>(2, 0);

	R.at<float>(0, 2) = Zm.at<float>(0, 0);
	R.at<float>(1, 2) = Zm.at<float>(1, 0);
	R.at<float>(2, 2) = Zm.at<float>(2, 0);


}

void TrackerHelper::getAnglesYXZ(float &pitch, float &yaw, float &roll, float &Ys, float &Zs, Mat R) {

	Mat Zm;
	float yd = R.at<float>(1,2);

	normalize(R.col(2), Zm);

	yaw = atan2(Zm.at<float>(0, 0), Zm.at<float>(2, 0));
	pitch = asin(Zm.at<float>(1, 0));

//		cout << "Zm" << Zm << endl;
	Mat X0 = (Mat_<float>(3, 1) << Zm.at<float>(2, 0), 0, -Zm.at<float>(0, 0));
//		cout << "X0" << X0 << endl;
	Mat Ym = R.col(1);
	Mat Y0 = Zm.cross(X0);
//		cout << "Ym" << Ym << endl;
//		cout << "X0 dot Y" << Xm.dot(Y)<< endl;
//		cout << "norm Xm" << norm(Xm)<< endl;
//		cout << "Ym dot Y" << Ym.dot(Y)<< endl;
//		cout << "norm Ym" << norm(Ym)<< endl;
	roll = atan2(X0.dot(Ym) / norm(X0), Y0.dot(Ym) / norm(Y0));
	Ys = norm(Ym)/norm(Y0);
	Zs = yd/Zm.at<float>(1,0);
	// cout << "Get Angles:" << endl<< "Zm" << Zm << endl<< "X0" << X0 << endl << "Y0" << Y0 <<
				// endl << "Ym" << Ym<< endl;
}

