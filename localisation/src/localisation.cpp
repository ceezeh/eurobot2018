/*
 * localisation.cpp
 *
 *  Created on: Mar 11, 2018
 *      Author: deeplearning
 */
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Point.h>
#include <localisation/helper.h>
#include "localisation/particle_filter.h"
#include <chrono>
#include <random>
#include <time.h>
#include <mutex>

using namespace std;

ParticleFilter pf;
double sigma_pos[3] = { .10, .10, 0.002 }; // odometry uncertainty [x [m], y [m], theta [rad]]
double sigma_beacon[2] = { .060, .060 }; // TODO: model the stationary error.
default_random_engine gen;
normal_distribution<double> N_x_init(0, sigma_pos[0]);
normal_distribution<double> N_y_init(0, sigma_pos[1]);
normal_distribution<double> N_theta_init(0, sigma_pos[2]);
MyTimer t; // Measures in microseconds.
mutex m;
geometry_msgs::Twist vel;

void odomCallback(const geometry_msgs::Twist::ConstPtr& odom_t) {
	// Prediction is done at this stage.
	m.lock();
	double n_x, n_y, n_theta, n_range;
	// Initialize particle filter if this is the first time step.
	if (!pf.initialized()) {
		pf.init(0, 0, 0, sigma_pos);
		t.start();
	} else {
		double delta_t = t.restart()/1e6;
		// Predict the vehicle's next state (noiseless).
		pf.prediction(delta_t, sigma_pos, odom_t->linear.x,
				odom_t->angular.z);
		vel = geometry_msgs::Twist(*odom_t);
	}
	m.unlock();
}

void beaconCallback(const geometry_msgs::Point::ConstPtr& point) {
	// Update is done at this stage.

	m.lock();
	// Update the weights and resample
	BeaconObs beacon = { 0, point->x, point->y };
	pf.updateWeights( sigma_beacon, beacon);
	pf.resample();
	m.unlock();
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "localisation");
	ros::NodeHandle n;

	ros::CallbackQueue queue1, queue2;
	// Create spinner for Odometry
	ros::SubscribeOptions ops =
			ros::SubscribeOptions::create<geometry_msgs::Twist>("/base_speed", // topic name
					10, // queue length
					odomCallback, // callback
					ros::VoidPtr(), // tracked object, we don't need one thus NULL
					&queue1 // pointer to callback queue object
					);

	// Odom
	ros::Subscriber sub1 = n.subscribe(ops);

	// Spinner
	ros::AsyncSpinner spinner(0, &queue1);
	spinner.start();

	// Create spinner for beacon
	ops = ros::SubscribeOptions::create<geometry_msgs::Point>(
			"/beacon", // topic name
			10, // queue length
			beaconCallback, // callback
			ros::VoidPtr(), // tracked object, we don't need one thus NULL
			&queue2 // pointer to callback queue object
			);

	// Odom
	ros::Subscriber sub2 = n.subscribe(ops);

	// Spinner
	ros::AsyncSpinner spinner2(0, &queue2);
	spinner2.start();

	double n_x, n_y, n_theta, n_range;
	ros::Publisher pub = n.advertise<nav_msgs::Odometry>("/my_robot/odom", 10);
//	TODO: Write code to update theta.
	ros::Rate loop_rate(5);
	while (ros::ok()) {
		m.lock();
		geometry_msgs::Pose p = pf.getPosition();
		nav_msgs::Odometry odom;
		odom.pose.pose = p;
		odom.twist.twist = vel;
		m.unlock();
		pub.publish(odom);
		loop_rate.sleep();
	}
}
