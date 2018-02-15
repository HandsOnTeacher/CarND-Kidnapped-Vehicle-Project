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

const bool DEBUG = true;

using namespace std;

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

	cout << "\n-------------------- Initial GPS Data ------------------------" << "\n"
	 		 << "\t(" << x << ", " << y << ", " << theta << ")\n";


	// Normal distributions for GPS data
	// x,y and theta std en std array
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);	

	// Reserve memory for all particles
	particles.reserve(num_particles);

	cout << "\n-------------------- List of particles ------------------------" << "\n";

	for (unsigned int i=0; i < num_particles; i++)
	{
		Particle new_particle;

		new_particle.x = dist_x(gen);
		new_particle.y = dist_y(gen);
		new_particle.theta = dist_theta(gen);
		new_particle.weight = 1.0;

		particles.push_back(new_particle);

		cout << "\t" << i << ": " << "\t(" << new_particle.x << ", " << new_particle.y << ", " 
											<< new_particle.theta << ")\n";

	}

	// Initialize all weights to 1 in weights vector
	weights.resize(num_particles);
	std::fill(weights.begin(), weights.end(), 1.0);


	//cout << "Initial values:" << "\n";
	//cout << "x: " << x << ", y:" << y << ", theta: " << theta << "\n";

	// for (unsigned int i=0; i < num_particles; i++)
	// {
	// 	cout << "Particle " << i << ":" << particles[i].x << ", " << particles[i].y << ", " << particles[i].theta << "\n"; 
	// }

	// for (unsigned int j=0; j < num_particles; j++)
	// {
	// 	cout << "Weight " << j << ":" << weights[j] << "\n";
	// }		

	// cout << "Initalization run without errors ;-)!\n";
	is_initialized = true;
	// cin.get();
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// Random engine to generate gaussian noise
	default_random_engine gen;

	// for (int i = 0; i< particles.size(); i++){
	// 	particles[i].weight = 1;
	// 	cout << "New particle " << i << "(" << particles[i].x << ", " << particles[i].y << ")\n";
	// }

	// cin.get();


	for (unsigned int i=0; i < particles.size(); i++)
	{
		// float x_0 = particles[i].x;
		// float y_0 = particles[i].y;
		float theta = particles[i].theta;

		// float x_f = 0;
		// float y_f = 0;
		// float theta_f = 0;

		if (yaw_rate < 1.0e-03)
		{
			particles[i].x += velocity * cos(theta) * delta_t;
			particles[i].y += velocity * sin(theta) * delta_t; 
			// I think this last one could be mantained
			//theta_f = theta_0 + yaw_rate * delta_t;
		}
		else
		{
			particles[i].x += velocity / yaw_rate * (sin(theta + yaw_rate*delta_t) - sin(theta));
			particles[i].y += velocity / yaw_rate * (cos(theta) - cos(theta + yaw_rate*delta_t)); 
			particles[i].theta += yaw_rate * delta_t;
		}

		// normal_distribution<double> dist_x(x_f, std_pos[0]);
		// normal_distribution<double> dist_y(y_f, std_pos[1]);
		// normal_distribution<double> dist_theta(theta_f, std_pos[2]);		

		// particles[i].x = dist_x(gen);
		// particles[i].y = dist_y(gen);
		// particles[i].theta = dist_theta(gen);

		normal_distribution<double> dist_x(0, std_pos[0]);
		normal_distribution<double> dist_y(0, std_pos[1]);
		normal_distribution<double> dist_theta(0, std_pos[2]);		

		particles[i].x += dist_x(gen);
		particles[i].y += dist_y(gen);
		particles[i].theta += dist_theta(gen);
		
		cout << "\n-------------------- Particles predictions ------------------------" << "\n";

		cout << "\tParticle " << i << ":\t" << "(" << particles[i].x << ", " << particles[i].y << ", " << particles[i].theta << ")\n";  
		// cin.get();

	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

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

	// First step: transform meauserements 
	// Data could be modified in the observations vector. I create anyway a new one.



	// Se me va de las manos el tamaño, hay que definirlo dentro, ahora mismo añade de un monton
	// de partiulas diferentes

	std::vector<double> pesos;

	for (unsigned int i = 0; i < particles.size(); i++)
	{
		std::vector<LandmarkObs> trans_observations;
		LandmarkObs trans_observation;
		
		std::vector<LandmarkObs> predicted;
		LandmarkObs prediction;
		// Transform and print out the transformed value
		// Using equations explained in the course
		// We need to use x, y and theta values from the particle

		//cout << "Particle number: " << i << "\n";
		//cout << "Particle x" <<particles[i].x << "Particle y" << particles[i].y<< "Particle theta: "<< particles[i].theta<< endl;
		cout << "\n--- PARTICLE\t" << i << " ---\n";

		cout << "\n--------------- Observations and Transformations ---------------\n";
		for (unsigned int j = 0; j < observations.size(); j++)
		{
			// Print the value of all observations
			cout << "part" << i << "obs" << j << "(" << observations[j].x << "," << observations[j].y << ")   \t-->\t";

			// Translation and rotation in one step
			float x_trans = particles[i].x + cos(particles[i].theta) * observations[j].x - 
				sin(particles[i].theta) * observations[j].y;
			float y_trans = particles[i].y + sin(particles[i].theta) * observations[j].x + 
				cos(particles[i].theta) * observations[j].y;

			trans_observation.id = j;
			trans_observation.x = x_trans;
			trans_observation.y = y_trans;

			trans_observations.push_back(trans_observation);
			cout << "trans(" << x_trans << "," << y_trans << ")\n";
			//cout << "Tranns Observations number " << trans_observation.id << " :";
			// cin.get();		
		}

		// All measurements have been translated for the actual particle
		// Select the closest landmark in the map  to each particle (associate)

		// Para cada observacion:
		// Compara la distancia con todos los landmarks
		// Asocialo a la mejor

		//cout << "Trans Observations size: " << trans_observations.size() << "\n";
		//cout << "landmark_list size: " << map_landmarks.landmark_list.size() << "\n";

		cout << "\n--------------- Transformations and Asignations ---------------\n";

		for (unsigned int k = 0; k < trans_observations.size(); k++)
		{

			int index_mindistance = -1;
			float mindistance = 9e+99;
			float distance = 0;

			for (unsigned l = 0; l < map_landmarks.landmark_list.size(); l++)
			{
				

				distance = dist(trans_observations[k].x, trans_observations[k].y,
			 									map_landmarks.landmark_list[l].x_f, map_landmarks.landmark_list[l].y_f);

				if (distance < mindistance)
				{
					mindistance = distance;
					index_mindistance = l;
				}

				
				// cout << "Distance " << k << " - " << l << " :" << distance << endl; 

			}

			//cout << "Asociated with landmark: " << index_mindistance << endl;
			//cout << "Whit a distance of : " << mindistance << endl;

			// Adjust index_mindistance with id_i to landmarks map
			prediction.id = index_mindistance + 1;
			prediction.x = map_landmarks.landmark_list[index_mindistance].x_f;
			prediction.y = map_landmarks.landmark_list[index_mindistance].y_f;


			predicted.push_back(prediction);

			cout << "Observacion " << k << "--> trans(" << trans_observations[k].x << "," << trans_observations[k].y << ")   \t-->\t";

			cout << "predicted(" << prediction.x << "," << prediction.y << ")\tDistance:" << mindistance << "  \tLM:" << index_mindistance << "\n";

			cout << "Landmark[" << index_mindistance <<"]:\t" << "ID:" << map_landmarks.landmark_list[index_mindistance].id_i << "\t" 
					 << "x: " << map_landmarks.landmark_list[index_mindistance].x_f << "\t"
					 << "y: " << map_landmarks.landmark_list[index_mindistance].y_f << "\n";

			particles[i].associations.push_back(index_mindistance + 1);

		}


		// cin.get();

		// cout << "Vector predicted con las predicciones para cada observacion\n";
		// for (unsigned int i = 0; i < predicted.size(); i++)
		// {
		// 	cout << "Observation " << i << ":" << predicted[i].id;
		// }

		// cin.get();
		// Calculate the weights
		// Es necesario tener en cuenta:
		// 1. Cada observacion transformada
		// 2. El landmark asociado a cada observación transformada

		std::vector<double> probalidades;

		double probability = 1;
		double gauss_norm= (1/(2 * M_PI * std_landmark[0] * std_landmark[1]));
  	
  	std::cout.precision(5);
		std::cout << std::scientific;

		cout << "\n--------- Weights --------\n";

		for (unsigned int i = 0; i < trans_observations.size(); i++)
		{
			double exponent = pow(trans_observations[i].x - predicted[i].x,2)/(2*pow(std_landmark[0],2))+
								 pow(trans_observations[i].y - predicted[i].y,2)/(2*pow(std_landmark[1],2));

			
			probalidades.push_back(gauss_norm * exp(-exponent));
			cout << "Obs " << i << " --> Exponent: " << exponent << " --> Obsertion probability: " << probalidades[i] << "\n";
			cout << "(" << trans_observations[i].x << "," << trans_observations[i].y << ") - (" 
					 << predicted[i].x << ", " << predicted[i].y << ")\n";
		}

		for (int i = 0; i < probalidades.size(); i++)
		{
			probability = probability * probalidades[i];
			// cout << "Multiplicando una a una: " << probability; 
		}	
		// cout << "probability final: " << probability;
		std::cout << std::defaultfloat;
		pesos.push_back(probability);
		// cin.get();
		cout << "\nParticle weight: " << probability <<"\n";
	}	

	cout << "\n[";
	for (unsigned int i = 0; i < particles.size(); i++)
	{
		cout << pesos[i] << ", ";
	}
	cout << "]\n";
	weights = pesos;
	cout << "\n[";
	for (unsigned int i = 0; i < particles.size(); i++)
	{
		cout << weights[i] << ", ";
	}
	cout << "]\n";	
	// cin.get();

	double sum_of_elems = 0;

	for (unsigned int i = 0; i < particles.size(); i++)
	{
		sum_of_elems += weights[i];
	}

	cout << "Suma: " << sum_of_elems << "\n\n";

	cout << "\nNormalized = [";
  for (unsigned int i = 0; i < particles.size(); i++)
	{
	 	particles[i].weight = weights[i]/sum_of_elems;
	 	cout << particles[i].weight << ", ";
	}

	cout << "]\n";



}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	// int main()
	// {
 //    std::random_device rd;
 //    std::mt19937 gen(rd());
 //    std::discrete_distribution<> d({40, 10, 10, 40});
 //    std::map<int, int> m;
 //    for(int n=0; n<10000; ++n) {
 //        ++m[d(gen)];
 //    }
 //    for(auto p : m) {
 //        std::cout << p.first << " generated " << p.second << " times\n";
 //    }
	// }
	std::discrete_distribution<int> d(weights.begin(), weights.end()); // Define a discrete distribution
	std::vector<Particle> new_particles; // Resampled particles holder
	std::default_random_engine gen3;

	for (int i = 0; i< num_particles; i++){
		auto index = d(gen3);
		new_particles.push_back(std::move(particles[index]));
	}

	particles = new_particles;

	for (int i = 0; i< particles.size(); i++){
		particles[i].weight = 1;
		cout << "New particle " << i << "(" << particles[i].x << ", " << particles[i].y << ")\n";
	}

	//cin.get();


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
