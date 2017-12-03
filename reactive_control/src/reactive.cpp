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
ros::Publisher cmd_pub;


void irCallback(const sensor_msgs::LaserScan::ConstPtr& ir)
{

	float dist = ir->ranges[0];

	if (dist < .5) {

		// Slow to a stop first.
		geometry_msgs::Twist cmd;
		cmd_pub.publish(cmd);
		sleep(1);
		// Turn away from obstacle.
		geometry_msgs::Twist cmd;
		cmd.angular.z = .3;
		cmd_pub.publish(cmd);
		sleep(2);
	}

}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "Reactive");


  ros::NodeHandle n;

  cmd_pub = n.advertise<geometry_msgs::Twist>("/my_robot/cmd_vel", 1);

  ros::Subscriber sub = n.subscribe("ir0", 1, irCallback);

  ros::Rate loop_rate(10);


  while (ros::ok())
  {

	  geometry_msgs::Twist cmd;
	  cmd.linear.x = .1;
	  cmd_pub.publish(cmd);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

