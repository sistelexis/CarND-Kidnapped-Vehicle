/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO-START: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	num_particles = 100;
	default_random_engine gen;

	normal_distribution<double> N_x(x, std[0]);
	normal_distribution<double> N_y(y, std[1]);
	normal_distribution<double> N_theta(theta, std[2]);

	for (int i = 0; i < num_particles; i++) {
		Particle particle;
		particle.id = i;
		particle.x = N_x(gen);
		particle.y = N_y(gen);
		particle.theta = N_theta(gen);
		particle.weight = 1;

		particles.push_back(particle);
		weights.push_back(1);
	}

	is_initialized = true;

	// cout << " x " << x << " y " << y << " yaw_rate " << theta << endl;

	// TODO-END: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO-START: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine gen;

	for (int i = 0; i < num_particles; i++) {

		double new_theta;

		if (yaw_rate == 0){
			new_theta = particles[i].theta;
			particles[i].x += velocity * delta_t * cos(new_theta);
			particles[i].y += velocity * delta_t * sin(new_theta);
			
		}
		else{
			new_theta = particles[i].theta + yaw_rate * delta_t;
			particles[i].x += velocity / yaw_rate * (sin(new_theta) - sin(particles[i].theta)); 
			particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(new_theta)); 
		}

		normal_distribution<double> N_x(particles[i].x, std_pos[0]);
		normal_distribution<double> N_y(particles[i].y, std_pos[1]);
		normal_distribution<double> N_theta(new_theta, std_pos[2]);

		particles[i].x = N_x(gen);
		particles[i].y = N_y(gen);
		particles[i].theta = N_theta(gen);

		// cout << " x " << particles[i].x << " y " << particles[i].y <<  "yaw_rate " << particles[i].theta << endl << endl;
	}

	// TODO-END: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

}

std::vector<LandmarkObs> ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations, double sensor_range) {
	// TODO-START: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	
	std::vector<LandmarkObs> association;
	LandmarkObs closest;
	
	for (auto obs: observations){
		
		double closest_dis = sensor_range; // some number larger than any possible measurement 
		
		for (auto pred: predicted){
			double calc_dist = dist(obs.x,obs.y,pred.x,pred.y);
			if (calc_dist < closest_dis) {
				closest_dis = calc_dist;
				closest = pred;
			}
		}

		association.push_back(closest);
	}
	
	return association;

	// TODO-END: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.S
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO-START: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	for(int p=0; p < num_particles; ++p) {
	    // perform the space transformation from vehicle to map
		vector<LandmarkObs> trans_observations;
		
		for (auto obs: observations){
			
			LandmarkObs trans_obs;
			trans_obs.x = particles[p].x + obs.x * cos(particles[p].theta) - obs.y * sin(particles[p].theta);
			trans_obs.y = particles[p].y + obs.x * sin(particles[p].theta) + obs.y * cos(particles[p].theta);
			trans_obs.id = obs.id;

			trans_observations.push_back(trans_obs);
		}

		// get landmarks within particle range
		vector<LandmarkObs> predicted;
		for (auto landmark: map_landmarks.landmark_list){

			double distance = dist(particles[p].x,particles[p].y,landmark.x_f,landmark.y_f);
			if (distance < sensor_range) {
				LandmarkObs one_landmark;
				one_landmark.id = landmark.id_i;
				one_landmark.x = landmark.x_f;
				one_landmark.y = landmark.y_f;
				predicted.push_back(one_landmark);		
			}
		}

		// associate nearest landmark to each particle observation   
		vector<LandmarkObs> associated_landmarks;
		associated_landmarks = dataAssociation(predicted, trans_observations, sensor_range);

		double probability = 1;		
		for (int al=0; al < associated_landmarks.size(); ++al){

			double dx = trans_observations.at(al).x - associated_landmarks.at(al).x;
			double dy = trans_observations.at(al).y - associated_landmarks.at(al).y;
			probability *= 1.0/(2*M_PI*std_landmark[0]*std_landmark[1]) * exp(-dx*dx / (2*std_landmark[0]*std_landmark[0]))* exp(-dy*dy / (2*std_landmark[1]*std_landmark[1]));			
		}
		particles[p].weight = probability;
		weights[p] = probability;
	}

	return;

	// TODO-END: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
}

void ParticleFilter::resample() {
	// TODO-START: Resample particles with replacement with probability yroportional to their weight. y	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	default_random_engine gen;
	discrete_distribution<int> distribution(weights.begin(),weights.end());

	vector<Particle> resample_particles;

	for (int i = 0; i < num_particles; i++){
		resample_particles.push_back(particles[distribution(gen)]);
	}

	particles = resample_particles;

	// TODO-END: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    //Clear the previous associations
    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
