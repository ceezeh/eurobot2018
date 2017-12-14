/*
 * reactive.cpp
 *
 *  Created on: Dec 3, 2017
 *      Author: deeplearning
 */


#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <unistd.h>
#include <sstream>
#include <cmath>
#define MAX_DIST 999
using namespace std;
ros::Publisher cmd_pub;
float distances[5] ={MAX_DIST,MAX_DIST,MAX_DIST,MAX_DIST,MAX_DIST};
float sensorAngs[5] = {0,1.2566370614,2.5132741229,-2.5132741229,-1.2566370614 };
/*
 * Let f = k/d, where f = [v,w]. To compute k,
 * we recall that max v is 0.4; max w ~0.7.
 * Thus, we want a k that completely stops the robot when the obstacle
 * is about 0.1m away.
 * Thus kv = .04 and kw = 0.07
 */
#define Kv 0.08
#define Kw 0.14
float vt = 0.4;
float wt = 0.0;

void irCallback0(const sensor_msgs::LaserScan::ConstPtr& ir)
{

	float dist = ir->ranges[0];

	if (dist < .5) {
		distances[0] = dist;
//		// Slow to a stop first.
//		geometry_msgs::Twist cmd;
//		cmd_pub.publish(cmd);
//		sleep(1);
//		// Turn away from obstacle.
//		geometry_msgs::Twist cmd;
//		cmd.angular.z = .3;
//		cmd_pub.publish(cmd);
//		sleep(2);
	} else {
		distances[0] = MAX_DIST;
	}

}

void irCallback1(const sensor_msgs::LaserScan::ConstPtr& ir)
{
	float dist = ir->ranges[0];
	if (dist < .5) {
		distances[1] = dist;
	} else {
		distances[1] = MAX_DIST;
	}
}
void irCallback2(const sensor_msgs::LaserScan::ConstPtr& ir)
{
	float dist = ir->ranges[0];
	if (dist < .5) {
		distances[2] = dist;
	} else {
		distances[2] = MAX_DIST;
	}
}

void irCallback3(const sensor_msgs::LaserScan::ConstPtr& ir)
{
	float dist = ir->ranges[0];
	if (dist < .5) {
		distances[3] = dist;
	} else {
		distances[3] = MAX_DIST;
	}
}

void irCallback4(const sensor_msgs::LaserScan::ConstPtr& ir)
{
	float dist = ir->ranges[0];
	if (dist < .5) {
		distances[4] = dist;
	} else {
		distances[4] = MAX_DIST;
	}
}

void computeForce(float &vop,float &wop) {
	// Add all forces
	 float Fx = 0;
	 float Fy = 0;
	for (int i = 0;i<5; i++) {
		// Consume information after using it.
		if (distances[i] < MAX_DIST){
			Fx += distances[i]*cos(sensorAngs[i]);
			Fy += distances[i]*sin(sensorAngs[i]);
			distances[i] = MAX_DIST;
		}
	}
	if (abs(Fx)>.05) vop = Kv/Fx;
	if (abs(Fy)>.05) wop = Kw/Fy;
	cout << "Fx"<< Fx<<", Fy"<< Fy<<", Vop:" << vop <<", Wop: "<< wop <<endl;

}
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "Reactive");


  ros::NodeHandle n;

  cmd_pub = n.advertise<geometry_msgs::Twist>("/my_robot/cmd_vel", 1);

  ros::Subscriber sub0 = n.subscribe("ir0", 1, irCallback0);
  ros::Subscriber sub1 = n.subscribe("ir1", 1, irCallback1);
  ros::Subscriber sub2 = n.subscribe("ir2", 1, irCallback2);
  ros::Subscriber sub3 = n.subscribe("ir3", 1, irCallback3);
  ros::Subscriber sub4 = n.subscribe("ir4", 1, irCallback4);

  ros::Rate loop_rate(50);

  float vop; float wop;
  vop=wop=0;
  while (ros::ok())
  {
	  computeForce(vop, wop);
	  geometry_msgs::Twist cmd;
	  cmd.linear.x = vt-vop;
	  cmd.angular.z= wt-wop;
	  cmd_pub.publish(cmd);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

