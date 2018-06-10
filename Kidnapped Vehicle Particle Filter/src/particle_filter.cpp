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
#include <cfloat>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  default_random_engine gen;
  normal_distribution<double> dist_x(x,std[0]);
  normal_distribution<double> dist_y(y,std[1]);
  normal_distribution<double> dist_theta(theta,std[2]);

  num_particles = 8;
  for(int i=0; i < num_particles; i++)
  {
    Particle pt(i,dist_x(gen),dist_y(gen),dist_theta(gen),1.0);
    particles.push_back(pt);
    weights.push_back(1.0);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
  default_random_engine gen;
  for(int i=0; i < num_particles; i++)
  {
    if( fabs(yaw_rate) < 1e-10 )
    {
      particles[i].x += velocity*cos(particles[i].theta)*delta_t;
      particles[i].y += velocity*sin(particles[i].theta)*delta_t;
    }
    else
    {
      particles[i].x += (velocity/yaw_rate)*( sin( particles[i].theta + yaw_rate*delta_t ) - sin( particles[i].theta ) );
      particles[i].y += (velocity/yaw_rate)*( cos( particles[i].theta ) -cos( particles[i].theta + yaw_rate*delta_t ) );
      particles[i].theta += yaw_rate*delta_t;
    }

    normal_distribution<double> dist_x(0,std_pos[0]);
    normal_distribution<double> dist_y(0,std_pos[1]);
    normal_distribution<double> dist_theta(0,std_pos[2]);

    //Add Noise to the predicted values due to noise in estimation of velocity/yaw_rate.
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
  }

}

std::vector<LandmarkObs> ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {

  std::vector<LandmarkObs> associated_lmark_diffs;
  int min_dist_lmark_id = 0;
  /* Debug code
  std::ofstream debfile;
  debfile.open("deb.txt",ios::app);
  */

  for(int i=0; i < observations.size(); i++)
  {
    LandmarkObs det_lmark;
    LandmarkObs diff_lmark;
    double dist_lmk_obs_min = DBL_MAX;

    //Loop through all observations to find the observation that is closest to the landmark
    for(int j=0; j < predicted.size(); j++)
    {
      double dist_lmrk_obs = (observations[i].x - predicted[j].x)*(observations[i].x - predicted[j].x) +
                             (observations[i].y - predicted[j].y)*(observations[i].y - predicted[j].y);

      if(dist_lmrk_obs < dist_lmk_obs_min)
      {
        det_lmark = predicted[j];
        dist_lmk_obs_min = dist_lmrk_obs;
        min_dist_lmark_id = predicted[j].id;
      }
    }

    diff_lmark.x = det_lmark.x - observations[i].x;
    diff_lmark.y = det_lmark.y - observations[i].y;
    associated_lmark_diffs.push_back(diff_lmark);
    //Observation i is associated with min_dist_lmark_id
    observations[i].id = min_dist_lmark_id;

    /* Debug code
    debfile << "Observation coords:(" << observations[i].x<<" "<<observations[i].y<<")\n";
    debfile << "Landmark coords:(" << det_lmark.x <<" "<<det_lmark.y<<")\n";
    debfile << "\n ";
    */
  }

  /* Debug code
  debfile.close();
  */
  return associated_lmark_diffs;

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {


  for(int i=0; i < num_particles; i++){
    vector<LandmarkObs> mapped_obs;
    //Transform Observation to Map coordinate system
    for(int j=0; j < observations.size(); j++)
    {
      LandmarkObs obs;
      obs.x = particles[i].x + observations[j].x*cos(particles[i].theta) - observations[j].y*sin(particles[i].theta);
      obs.y = particles[i].y + observations[j].x*sin(particles[i].theta) + observations[j].y*cos(particles[i].theta);
      mapped_obs.push_back(obs);
    }

    //Filter out landmarks which are beyond the sensor range distance from the particle.
    vector<LandmarkObs> map_landmarks_filt;
    for(int k=0; k < map_landmarks.landmark_list.size(); k++)
    {
      LandmarkObs temp_landmark;
      double dist_lmark_part = sqrt( pow( ( map_landmarks.landmark_list[k].x_f - particles[i].x),2 ) + pow( ( map_landmarks.landmark_list[k].y_f - particles[i].y ),2 ) );
      if( dist_lmark_part <= sensor_range)
      {
        temp_landmark.id = map_landmarks.landmark_list[k].id_i;
        temp_landmark.x = map_landmarks.landmark_list[k].x_f;
        temp_landmark.y = map_landmarks.landmark_list[k].y_f;
        map_landmarks_filt.push_back(temp_landmark);
      }
    }

    //Convert map landmarks into a vector of predicted landmarks for calling the above func
    vector<LandmarkObs> associated_lmark_diffs = dataAssociation(map_landmarks_filt,mapped_obs);

    //Calculate the weights based on multivariate distribution and update the weights
    double wt_part=1.0;
    for(int p=0; p < associated_lmark_diffs.size(); p++)
    {
      double x_d = pow(associated_lmark_diffs[p].x,2)/(2*pow(std_landmark[0],2));
      double y_d = pow(associated_lmark_diffs[p].y,2)/(2*pow(std_landmark[1],2));
      double e_pow = exp(-(x_d + y_d ) );
      wt_part *= (1/(2*3.14*std_landmark[0]*std_landmark[1]))*e_pow;
    }

    /* Debug code
    std::ofstream debfile;
    debfile.open("deb.txt",ios::app);
    */
    particles[i].weight = wt_part;
    weights[i] = wt_part;

    //Prep for set of associations for sake of completeness
    vector<int> assoc_ids;
    vector<double> sense_x;
    vector<double> sense_y;
    for(int q=0; q < mapped_obs.size(); q++)
    {
      assoc_ids.push_back(mapped_obs[q].id);
      sense_x.push_back(mapped_obs[q].x);
      sense_y.push_back(mapped_obs[q].y);
    }

    SetAssociations(particles[i],assoc_ids,sense_x,sense_y);
    /*
    debfile<< "Particle weight is:"<< wt_part<< "\n"<<"\n";
    debfile.close();
    */
  }
}

void ParticleFilter::resample() {

  vector<Particle> resampled_particles (num_particles);

  // Use discrete distribution to return particles by weight
  default_random_engine gen;
  discrete_distribution<int> part_idx(weights.begin(), weights.end());

  for (int i = 0; i < num_particles; ++i) {
    resampled_particles[i] = particles[part_idx(gen)];
  }

  // Replace old particles with the resampled particles
particles = resampled_particles;

}

void ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

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
