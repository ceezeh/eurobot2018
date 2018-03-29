/*
 * particle_filter.cpp
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include "localisation/particle_filter.h"
#include "task_planner/helper.h"
using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	num_particles = 729; //set to number of files in observation directory

	weights.resize(num_particles);
	particles.resize(num_particles);

	double std_x, std_y, std_theta; // Standard deviations for x, y, and theta
	std_x = std[0];
	std_y = std[1];
	std_theta = std[2];

	// Normal distribution for x, y and theta
	normal_distribution<double> dist_x(x, std_x); // mean is centered around the new measurement
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	default_random_engine gen; //http://www.cplusplus.com/reference/random/default_random_engine/

	// create particles and set their values
	for (int i = 0; i < num_particles; ++i) {
		Particle p;
		p.id = i;
		p.x = dist_x(gen); // take a random value from the Gaussian Normal distribution and update the attribute
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
		p.weight = 1;

		particles[i] = p;
		weights[i] = p.weight;
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
		double velocity, double yaw_rate) {
	// Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	double std_x, std_y, std_theta; // Standard deviations for x, y, and theta
	std_x = std_pos[0];
	std_y = std_pos[1];
	std_theta = (yaw_rate==0.0&& velocity ==0.0)? 0:std_pos[2];

//	default_random_engine gen;

	for (int i = 0; i < num_particles; ++i) {
		Particle *p = &particles[i]; // get address of particle to update
		// use the prediction equations from the Lesson 14
		double new_x = p->x +velocity* delta_t*cos(p->theta+yaw_rate * delta_t);
		double new_y = p->y + velocity* delta_t*sin(p->theta+yaw_rate * delta_t);
		double new_theta = p->theta + (yaw_rate * delta_t);

			// update the particle attributes
		p->x =new_x ;
		p->y = new_y;
		p->theta = new_theta;

	}
}

void ParticleFilter::updateWeights(double std_beacon[], BeaconObs observation) {
	// Update the weights of each particle using a multi-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution

	double std_x = std_beacon[0];
	double std_y = std_beacon[1];
	double weights_sum = 0;

	for (int i = 0; i < num_particles; ++i) {
		Particle *p = &particles[i];
		double wt = 1.0;

		// update weights using Multivariate Gaussian Distribution
		// equation given in Transformations and Associations Quiz
		double num =
				exp(
						-0.5
								* (pow((p->x - observation.x), 2)
										/ pow(std_x, 2)
										+ pow((p->y - observation.y), 2)
												/ pow(std_y, 2)));
		double denom = 2 * M_PI * std_x * std_y;
		wt *= num / denom;

		weights_sum += wt;
		p->weight = wt;
	}
	// normalize weights to bring them in (0, 1]
	for (int i = 0; i < num_particles; i++) {
		Particle *p = &particles[i];
		p->weight /= weights_sum;
		weights[i] = p->weight;
	}
}

void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	default_random_engine gen;

	// Random integers on the [0, n) range
	// the probability of each individual integer is its weight of the divided by the sum of all weights.
	discrete_distribution<int> distribution(weights.begin(), weights.end());
	vector<Particle> resampled_particles;

	for (int i = 0; i < num_particles; i++) {
		resampled_particles.push_back(particles[distribution(gen)]);
	}

	particles = resampled_particles;

}
/*
 * This function is used after resampling so that each particle has the same weight.
 */
geometry_msgs::Pose ParticleFilter::getPosition() {
	double x, y, theta; // x(m)
	x = y = theta = 0;
	double sineAcc, cosineAcc;
	sineAcc = cosineAcc = 0;
	for (int i = 0; i < num_particles; i++) {
		x += particles[i].x;
		y += particles[i].y;
		sineAcc += sin(particles[i].theta);
		cosineAcc += cos(particles[i].theta);

	}
	x /= num_particles;
	y /= num_particles;

	theta = atan2(sineAcc / num_particles, cosineAcc / num_particles);
	geometry_msgs::Pose p;
	p.position.x = x;
	p.position.y = y;
	cout <<"x: "<<x<<", y: "<<y<<", th: "<<theta<<endl;
	p.orientation = getQuaternion(theta);
	return p;
}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " "
				<< particles[i].theta << "\n";
	}
	dataFile.close();
}
