#ifndef ODOMETRY_HELPER_H_
#define ODOMETRY_HELPER_H_

#include <unistd.h>
#include "string"
#include <chrono>
#include <math.h>
#include <cmath>
using namespace std;
using namespace std::chrono;
//TODO Add more status flags

#define STATUS_NAV_COMPLETE 1
#define equals(x, y) (fabs(x-y) < 0.00001)

const string GOTO_ID = "goto";
float angDiff(float a1, float a2);
float wraparound(float ang);

struct Pose_t {
	float x, y, th;
	bool closeTo(Pose_t p) { // TODO: Be mindful of the threshold. May affect odometry accuracy.
		if ((sqrt(pow(this->x - p.x, 2) + pow(this->y - p.y, 2)) < .04)
				&& (abs(angDiff(this->th, p.th)) < 0.3)) {
			return true;
		} else {
			return false;
		}
	}
	Pose_t() {
		x = y = th = 0;
	}
	Pose_t(float xt, float yt, float tht) {
		x = xt;
		y = yt;
		th = tht;
	}
	bool operator==(const Pose_t& p) {
		return (equals(this->x, p.x) && equals(this->y, p.y)
				&& equals(angDiff(this->th, p.th), 0));
	}
};

struct Velocity {
	float v;
	float w;
	Velocity() {
		v = w = 0;
	}
	Velocity(float vt, float wt) {
		v = vt;
		w = wt;
	}
};

class MyTimer {
private:
	int maxduration;
	int aveduration;
	high_resolution_clock::time_point t1;
	high_resolution_clock::time_point t2;
	int last_duration;
	int count;
	// Updates stores the last duration
	// and updates the maximum and average duration.
	void update();
public:
	MyTimer();
	void start();
	void stop(); // stops and gets duration.
	int restart();
	int getElapsed();

	int getLastDuration() const {
		return last_duration;
	}

	int getMaxDuration() const {
		return maxduration;
	}

	int getAveDuration() const {
		return aveduration;
	}
};

#endif  /*ODOMETRY_HELPER_H_*/
