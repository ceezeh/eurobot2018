/*
 * main.cpp
 *
 *  Created on: 22 Mar 2018
 *      Author: ceezeh
 */
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include "reactive_control/astar.h"
#include "reactive_control/helper.h"
using namespace std;

int main(int argc, char **argv) {

	astar::AStar astar;

	astar.replan(Pose_t(0, 0, 0), Pose_t(1, 0, M_PI/2));
	while(true);
	return 0;
}
