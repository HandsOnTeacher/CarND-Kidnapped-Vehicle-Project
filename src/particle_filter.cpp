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

const bool DEBUG = false;
const double MIN_YAW_RATE = 1.0e-04;

using namespace std;

static int time_step = 0;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	// Random engine to generate gaussian noise
	default_random_engine gen;

	// Number of particles in the filter
	num_particles = 10;

	// Create normal distribution for x, y and theta
	// According to program quiz in lesson

	if (DEBUG)
	{
		cout << "\n-------------------- Initial GPS Data ------------------------" << "\n"
	 		 	 << "\t(" << x << ", " << y << ", " << theta << ")\n";
	}


	// Normal distributions for GPS data
	// x,y and theta std en std array
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);	

	// Reserve memory for all particles
	particles.reserve(num_particles);

	if (DEBUG)
	{
		cout << "\n-------------------- List of particles ------------------------" << "\n";
	}

	for (unsigned int i=0; i < num_particles; i++)
	{
		Particle new_particle;

		new_particle.x = dist_x(gen);
		new_particle.y = dist_y(gen);
		new_particle.theta = dist_theta(gen);
		new_particle.weight = 1.0;

		particles.push_back(new_particle);

		if (DEBUG)
		{
			cout << "\t" << i << ": " << "\t(" << new_particle.x << ", " << new_particle.y << ", " 
												<< new_particle.theta << ")\n";
		}

	}

	// Initialize all weights to 1 in weights vector
	weights.resize(num_particles);
	std::fill(weights.begin(), weights.end(), 1.0);

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// Random engine to generate gaussian noise
	default_random_engine gen;

	if (DEBUG)
	{
		cout << "\n-------------------- Particles predictions ------------------------" << "\n";
	}

	for (unsigned int i=0; i < particles.size(); i++)
	{

		// Variables used to debug
		float x_0 = particles[i].x;
		float y_0 = particles[i].y;
		float theta_0 = particles[i].theta;

		float theta = particles[i].theta;

		// If linear movement (yaw_rate = 0)
		if (yaw_rate < MIN_YAW_RATE)
		{
			particles[i].x += velocity * cos(theta) * delta_t;
			particles[i].y += velocity * sin(theta) * delta_t; 
		}
		else
		{
			particles[i].x += velocity / yaw_rate * (sin(theta + yaw_rate*delta_t) - sin(theta));
			particles[i].y += velocity / yaw_rate * (cos(theta) - cos(theta + yaw_rate*delta_t)); 
			particles[i].theta += yaw_rate * delta_t;
		}

		normal_distribution<double> dist_x(particles[i].x, std_pos[0]);
		normal_distribution<double> dist_y(particles[i].y, std_pos[1]);
		normal_distribution<double> dist_theta(particles[i].theta, std_pos[2]);		

		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
		
		if (DEBUG)
		{
		cout << "\tParticle " << i << ":\t" << "(" << x_0 << ", " << y_0 << ", " << theta_0 << ")\t--->\t"
				 << "(" << particles[i].x << ", " << particles[i].y << ", " << particles[i].theta << ")\n";  	
		}

	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.


	if (DEBUG)
	{
		cout << "\n------------------------------------------------------------------\n";
		cout << "                             DATA ASSOCIATION\n";
	}

	for (unsigned int i = 0; i < observations.size(); ++i)	
	{
		float distance = 9e99;
		float min_distance = 9e99;
		int min_dist_id = -1;
		int id = -1;

		for (unsigned int j = 0; j < predicted.size(); ++j)
		{
			distance = dist(observations[i].x, observations[i].y,
											predicted[j].x, predicted[j].y);
			if (distance < min_distance)
			{
				min_distance = distance;
				min_dist_id = predicted[j].id;
				id = j;
			}

		}

		observations[i].id = min_dist_id;

		if (DEBUG)
		{
			cout << "Observation " << i << " associated to LM " << min_dist_id
					 << "\t(" << observations[i].x << ", " << observations[i].y << ") --> ("
					 << predicted[id].x << ", " << predicted[id].y << ")\n";
		}
	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html


	for (unsigned int i = 0; i < particles.size(); ++i)
	{
		std::vector<LandmarkObs> trans_observations;
		LandmarkObs trans_observation;
		
		std::vector<LandmarkObs> predicted;
		LandmarkObs prediction;

		float distance = 0;

		// Calculate Landmarks in sensor range for every particle
		for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); ++j)
		{

			distance = dist(particles[i].x, particles[i].y,
								 map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f);	
			
			if (distance <= sensor_range)
			{
				prediction.x = map_landmarks.landmark_list[j].x_f;
				prediction.y = map_landmarks.landmark_list[j].y_f;
				prediction.id = map_landmarks.landmark_list[j].id_i;

				predicted.push_back(prediction);
			}

		}

		if (DEBUG)
		{
			cout << "--------------------------- LANDMARKS WITHIN RANGE PARTICLE << " << i
					 << "---------------------------\n";

			cout << "[";
			for (unsigned j = 0; j < predicted.size(); ++j)
			{
				cout << predicted[j].id << " ,";
			}

			cout << "]";
		}

		if (DEBUG)
		{
			cout << "\n--- PARTICLE\t" << i << " ---\n";

			cout << "\n--------------- Observations and Transformations ---------------\n";	
		}

		for (unsigned int j = 0; j < observations.size(); j++)
		{
			if (DEBUG)
			{
			// Print the value of all observations
				cout << "part" << i << "obs" << j << "(" << observations[j].x << "," << observations[j].y << ")   \t-->\t";
			}

			// Translation and rotation in one step
			float x_trans = particles[i].x + cos(particles[i].theta) * observations[j].x - 
				sin(particles[i].theta) * observations[j].y;
			float y_trans = particles[i].y + sin(particles[i].theta) * observations[j].x + 
				cos(particles[i].theta) * observations[j].y;

			trans_observation.id = j;
			trans_observation.x = x_trans;
			trans_observation.y = y_trans;

			trans_observations.push_back(trans_observation);

			if (DEBUG)
			{
				cout << "trans(" << x_trans << "," << y_trans << ")\n";	
			}
	
		}

		dataAssociation (predicted, trans_observations);

		weights[i] = 1;
		particles[i].weight = 1;

		double gauss_norm= (1/(2 * M_PI * std_landmark[0] * std_landmark[1]));
  	
  	std::cout.precision(5);
		std::cout << std::scientific;
		
		if (DEBUG)
		{
			cout << "\n--------- Weights --------\n";
		}

		for (unsigned int j = 0; j < trans_observations.size(); j++)
		{
			double probabilidad = 1;
			float predicted_x = map_landmarks.landmark_list[trans_observations[j].id - 1].x_f;
			float predicted_y = map_landmarks.landmark_list[trans_observations[j].id - 1].y_f;

			double exponent = pow(trans_observations[j].x - predicted_x,2)/(2*pow(std_landmark[0],2))+
								 pow(trans_observations[j].y - predicted_y,2)/(2*pow(std_landmark[1],2));
			
			if (DEBUG)
			{
				cout << "Obs " << j << " --> Exponent: " << exponent << " --> Obsertion probability: " << gauss_norm * exp(-exponent) << "\n";
				cout << "(" << trans_observations[j].x << "," << trans_observations[j].y << ") - (" 
						 << predicted_x << ", " << predicted_y << ")\n";
			}
			particles[i].weight *= gauss_norm * exp(-exponent);
		}
		
		weights[i] = particles[i].weight;

		if (DEBUG)
		{
			cout << "\nParticle weight: " << particles[i].weight <<"\n";
		}
	}	
	if (DEBUG)
	{
		for (unsigned int i = 0; i < particles.size(); i++)
		{
			cout << weights[i] <<"\n";
		}
	}

	std::cout << std::defaultfloat;

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	// Define a discrete distribution
	std::discrete_distribution<int> d(weights.begin(), weights.end()); 
	// Resampled particles holder
	std::vector<Particle> new_particles; 
	// Random engine
	default_random_engine gen;

	int index = 0;
	std::vector<int> indexes;

	// Generate index based on probability distribution
	// Add the particle to the new particle container
	for (int i = 0; i< num_particles; i++){
		index = d(gen);
		if (DEBUG)
		{
			indexes.push_back(index);
		}
		new_particles.push_back(std::move(particles[index]));
	}

	if (DEBUG)
	{
		for (int i = 0; i< num_particles; i++)
		{
			cout << indexes[i];
		}
	}
	particles = new_particles;

	if (DEBUG)
	{


		cout << "\nTime step:" << time_step << "\n";
	
		if (time_step >= 230)
		{
			cin.get();
		}
	}

	++time_step;
}


Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

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
