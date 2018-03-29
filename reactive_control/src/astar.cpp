/*
 * astar.cpp

 *
 *  Created on: 22 Mar 2018
 *      Author: ceezeh
 */
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "reactive_control/astar.h"
#include <algorithm>
#include <mutex>
#include <math.h>
#include <cmath>
#include <thread>
using namespace std::placeholders;
using namespace std;
std::mutex trajMutex;

astar::Point operator +(const astar::Point& left_, const astar::Point& right_) {
	return astar::Point(left_.x + right_.x, left_.y + right_.y);
}

bool astar::Node::operator==(const astar::Node& node) {
	return (this->coordinates.x == node.coordinates.x
			&& this->coordinates.y == node.coordinates.y);
}

astar::Node::Node(astar::Point coordinates_, Node *parent_) {
	parent = parent_;
	coordinates = coordinates_;
	G = H = 0;
}

astar::uint astar::Node::getScore() {
	return G + H;
}

astar::AStar::AStar() {
	resolution = 0.1;
	beginReplan = 0;
	isInitialised = false;
	this->setWorldSize(Point(30, 20));
	 directions = 8;
	direction = {
		{	0, 1}, {1, 0}, {0, -1}, {-1, 0},
		{	-1, -1}, {1, 1}, {-1, 1}, {1, -1}
	};

	// Begin replan thread here in non-blocking mode.
//	planner = std::thread(&astar::AStar::_replan, this);
}
void astar::AStar::replan(Pose_t currentPose, Pose_t goalPose) {

	trajMutex.lock();
	setCurrentPose(currentPose);
	setGoalPose(goalPose);
	trajMutex.unlock();
	this->setBeginReplan(true);
	_replan();

}

void astar::AStar::setWorldSize(astar::Point worldSize_) {
	worldSize = worldSize_;
}

void astar::AStar::addCollision(astar::Point coordinates_) {
	walls.push_back(coordinates_);
}

void astar::AStar::clearCollisions() {
	walls.clear();
}

astar::CoordinateList astar::AStar::findPath(astar::Point source_,
		astar::Point target_) {
	Node *current = nullptr;
	NodeSet openSet, closedSet;
	openSet.insert(new Node(source_));

	while (!openSet.empty()) {
		current = *openSet.begin();
		for (auto node : openSet) {
			if (node->getScore() <= current->getScore()) {
				current = node;
			}
		}

		if (current->coordinates == target_) {
			break;
		}

		closedSet.insert(current);
		openSet.erase(std::find(openSet.begin(), openSet.end(), current));

		for (uint i = 0; i < directions; ++i) {

			astar::Point newCoordinates(current->coordinates + direction[i]);

			if (detectCollision(newCoordinates)
					|| findNodeOnList(closedSet, newCoordinates)) {
				continue;
			}

			uint totalCost = current->G + ((i < 4) ? 10 : 14);

			Node *successor = findNodeOnList(openSet, newCoordinates);
			if (successor == nullptr) {
				successor = new Node(newCoordinates, current);
				successor->G = totalCost;
				successor->H = euclidean(successor->coordinates, target_);
				openSet.insert(successor);
			} else if (totalCost < successor->G) {
				successor->parent = current;
				successor->G = totalCost;
			}
		}
	}

	CoordinateList path;
	while (current != nullptr) {
		path.push_back(current->coordinates);
		current = current->parent;
	}

	releaseNodes(openSet);
	releaseNodes(closedSet);

	return path;
}

astar::Node* astar::AStar::findNodeOnList(NodeSet& nodes_,
		astar::Point coordinates_) {
	for (auto node : nodes_) {
		if (node->coordinates == coordinates_) {
			return node;
		}
	}
	return nullptr;
}

void astar::AStar::releaseNodes(NodeSet& nodes_) {
	for (auto it = nodes_.begin(); it != nodes_.end();) {
		delete *it;
		it = nodes_.erase(it);
	}
}

bool astar::AStar::detectCollision(astar::Point coordinates_) {
	if (coordinates_.x < 0 || coordinates_.x >= worldSize.x
			|| coordinates_.y < 0 || coordinates_.y >= worldSize.y
			|| std::find(walls.begin(), walls.end(), coordinates_)
					!= walls.end()) {
		return true;
	}
	return false;
}

/*
 * Use function in its own thread.
 */
void astar::AStar::_replan() {
// Calculate World Size
	while (true) {
		if (beginReplan) {
			trajMutex.lock();
			astar::Point currentNode = convertToPoint(currentPose);
			astar::Point targetNode = convertToPoint(goalPose);

			CoordinateList path = findPath(currentNode, targetNode);
			trajectory = convertPathToRealTrajectory(path, currentPose,
					goalPose);
			trajMutex.unlock();
			beginReplan = false;
			this->isInitialised = true;
		}
		usleep(1000);
	}
}
void astar::AStar::setBeginReplan(bool replan) {
	this->beginReplan = replan;
}
astar::RealTrajectoryList astar::AStar::convertPathToRealTrajectory(
		CoordinateList path, Pose_t start, Pose_t stop) {

	int num = path.size();

	// We want to dilute the angles for the set point.
	float deltaAngle = angDiff(stop.th, start.th) / num;

	astar::RealTrajectoryList trajectory;

	for (int i = 0; i < num; i++) {
		astar::Point p = path[i];
		Pose_t pose = convertToRealPose(p);
		pose.th = angDiff(stop.th, i * deltaAngle);
		trajectory.push_front(pose);
	}
	return trajectory;

}
Pose_t astar::AStar::convertToRealPose(astar::Point p) {
	float x = p.x * this->resolution;
	float y = p.y * this->resolution;
	return Pose_t(x, y, 0);
}

astar::Point astar::AStar::convertToPoint(Pose_t pose) {
	int x = std::ceil((pose.x) / resolution);
	int y = std::ceil((pose.y) / resolution);
	return astar::Point(x, y);
}

Pose_t astar::AStar::getSubGoal(Pose_t currentLocation) {
	if (!isInitialised || trajectory.empty()) {
		cout << "[1] No Trajectory found" << endl;
		return currentLocation;
	} else {

		if (trajectory.front() == goalPose) {
			return goalPose;
		}

		while (currentLocation.closeTo(trajectory.front())
				&& !trajectory.empty()) {
			trajectory.pop_front();
		}
		if (trajectory.empty()) {
			cout << "[2] No Trajectory found" << endl;
			return currentLocation;
		} else {

			return trajectory.front();
		}
	}
}

astar::Point astar::getDelta(astar::Point source_, astar::Point target_) {
	return astar::Point(std::abs(source_.x - target_.x), std::abs(source_.y - target_.y));
}

astar::uint astar::euclidean(astar::Point source_, astar::Point target_) {
	auto delta = std::move(getDelta(source_, target_));
	return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}
