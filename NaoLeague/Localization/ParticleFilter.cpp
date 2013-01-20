/*
 * ParticleFilter.cpp
 *
 *  Created on: Jan 12, 2013
 *      Author: owner
 */

#include "ParticleFilter.h"
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <assert.h>
#include <time.h>
#include <vector>

#define PI 3.14159

using namespace std;


double ParticleFilter::random_uniform ()
{
  return ((double) rand())/RAND_MAX;
}
/*
 * box muller transform
 */
double ParticleFilter::random_gaussian ()
{
  double u = random_uniform(), v = random_uniform();
  return sqrt(-2*log(u))*cos(2*M_PI*v);
}
/*
 * computes the probability of a point in a normal distribution
 */
double ParticleFilter::prob_gaussian(double val,double var_sq){
	return(1/sqrt(2*PI*var_sq)*exp(- ( val * val )/ (2 * var_sq) ));
}

ParticleFilter::ParticleFilter(){
	//random seed:
	srand(time(NULL));
	this->set_params();
}
void ParticleFilter::set_params(){
	this->error_range = 0.1;
	this->error_bearing = 0.2;

	this->resample_variance_pos = 15;
	this->resample_variance_rot = 1;

	this->measurement_factor = 1;

	this->variance_range = 10000;
	this->variance_bearing = 100;

	this->x_dim = 740;
	this->y_dim = 540;
}
ParticleFilter::ParticleFilter(int width, int height,int number_of_particles){
	//random seed:
	srand(time(0));
	this->set_params();
}
ParticleFilter::~ParticleFilter(){

}
/*
 * Debug function
 */
void ParticleFilter::test() {
	std::cout << "hello from particleFilter!" << std::endl;
}
/*
 * deletes the linked list of particles and all its descendants
 */
int ParticleFilter::delete_particles(){

}
/*
 * resamples all particles with low variance resampling algorithm. (creates new particle list)
 */
int ParticleFilter::resample() {

	////low variance resampling:

	//draw random number
	// add fixed amount, to each bin we step into, sample from that particle


	vector<Particle> resampled_particles;

	//first bin border
	double c = this->particles[0].weight;
	double u = 0;

	assert(this->particles.size() >= 0);


	//to make this foolproof, even if we dont use a probabilistic totally sound model
	double step_size = sum_weights()/(double) this->particles.size();

	//TODO: might want to remove this
	assert(step_size >= 0);

	//fixed offset to reach the next bin
	double offset = ((double)rand() / (double)RAND_MAX) * step_size;

	//TODO: might want to remove that
	assert(offset > 0);


	int j = 0;
	for(int i = 1; i <= this->particles.size(); i++){
		u =  offset + (i-1) * step_size;
		while(u >= c){
			j++;
			c += this->particles[j].weight;
		}
		//add particle and add uncertainty
		Particle p(this->particles[j].x + random_gaussian() * this->resample_variance_pos, this->particles[j].y + random_gaussian() * this->resample_variance_pos,this->particles[j].rot + random_gaussian() * this->resample_variance_rot,1);
		resampled_particles.push_back(p);
	}

	//save new particles
	this->particles = resampled_particles;

}
int ParticleFilter::dynamic(OdometryInformation odo_inf,vector<VisualFeature> vis_feats){

	//get odometry information
	for(int i = 0; i < this->particles.size(); i++)
	{
		sample_motion_model_simple(odo_inf,&this->particles[i]);
		this->particles[i].weight = this->measurement_model(vis_feats,&this->particles[i]);
	}
	int no_particles_weight = 0;
	for (int k= 0; k< this->particles.size(); k++){
		if(this->particles[k].weight > 0){no_particles_weight ++;}
	}
	//cout<<" no particles weight:"<<no_particles_weight<<endl;
	//for 0..M
		// pose = sample_sample_motion_model
		// weight = measurement_model

	//CHECK FOR PARTICLES OUTSIDE THE FIELD
}

/*
 * simple method to incorporate new odometry data
 */
int ParticleFilter::sample_motion_model_simple(OdometryInformation odometry_information,Particle* last_pose)
{
	//rotate odometry_vector, according to last pose (transform to local coordinate system):
	double x  = odometry_information.x * cos(last_pose->rot) - odometry_information.y * sin(last_pose->rot);
	double y  = odometry_information.x * sin(last_pose->rot) + odometry_information.y * cos(last_pose->rot);

	//add noise to vector
	double error_x =  this->random_gaussian() * this->error_range * x;
	double error_y = this->random_gaussian() * this->error_range * y  ;
	double error_rot =  this->random_gaussian() * odometry_information.rot  * this->error_bearing;

	//cout<<"motion model : "<<x<<" "<<error_x<<" "<<last_pose->x<<" | "<<y<<" "<<error_y<<" "<<last_pose->y<<" | "<<last_pose->rot<<" "<<error_rot<<endl;
	last_pose->x =last_pose->x +  x + error_x;
	last_pose->y = last_pose->y + y + error_y;
	//cout<<"LASTROT:"<<last_pose->rot<<" "<<odometry_information.rot<<" "<<error_rot<<endl;
	last_pose->rot = last_pose->rot + odometry_information.rot + error_rot;

	//TODO: reject particles outside the field !!
}
int ParticleFilter::find_landmark(VisualFeature feature, Particle* current_pose,double* dist){
	//iterate through all features of that type, calculate distance to feature observed in map

	//TODO: Include mahalanobis distance (takes into account that measurement error in different dimensions of the data is different) i.e. range and bearing.

	//calculate position on map(assuming we are in the current pose):
	int x_feat = current_pose->x + cos(feature.bearing+current_pose->rot)*feature.range;
	int y_feat = current_pose->y + sin(feature.bearing+current_pose->rot)*feature.range;

	//choose feature with smallest distance and then proceed
		double min_dist = 1000; //10 meter
		int feature_index = 0;

		switch (feature.type){
		case 0: //L crossing observed 8
		{
			for(int i = 0;i < 8; i++){
				//calculate distance
				double delta_x =  x_feat - this->feature_map.l_cross[i].x ;
				double delta_y = y_feat - this->feature_map.l_cross[i].y ;
				double dist =  sqrt(delta_x * delta_x + delta_y * delta_y);

				if(dist < min_dist){
					min_dist = dist;
					feature_index = i;

				}
			}
			break;
		}
		case 1: //T crossing observed 7
		{

			for(int i = 0; i<7; i++){
				//calculate distance
				double delta_x = x_feat - this->feature_map.t_cross[i].x ;
				double delta_y = y_feat - this->feature_map.t_cross[i].y ;
				double dist =  sqrt(delta_x * delta_x + delta_y * delta_y);

				if(dist < min_dist){
					min_dist = dist;
					feature_index = i;
				}
			}
			break;
		}
		case 2: //X crossing observed 5
		{
			for(int i = 0;i<5;i++){
				//dist calculation
				double delta_x = x_feat - this->feature_map.x_cross[i].x ;
				double delta_y = y_feat - this->feature_map.x_cross[i].y ;
				double dist =  sqrt(delta_x * delta_x + delta_y * delta_y);

				if(dist < min_dist){
					min_dist = dist;
					feature_index = i;
				}
			}
			break;
		}
		case 3: //Goal post crossing crossing observed 4
		{
			for(int i = 0;i<4;i++){
				//dist calculation
				double delta_x = x_feat - this->feature_map.g_cross[i].x ;
				double delta_y = y_feat - this->feature_map.g_cross[i].y ;
				double dist =  sqrt(delta_x * delta_x + delta_y * delta_y);

				if(dist < min_dist){
					min_dist = dist;
					feature_index = i;
				}
			}
			break;
		}
		default:
			cout<<"something went terribly wrong in measurement model!" <<endl;
			break;
		}

		//save distance and return index of found landmark
		*dist = min_dist;
		return(feature_index);
}

/*
 * receives all visual features seen on the map and computes likelihood of a landmark measurment (with known correspondence)
 */
double ParticleFilter::measurement_model(vector<VisualFeature> features,Particle* current_pose)
{
	//TODO if this does not work, change !!
	//assume the nearest landmark is the one observed (ML approach)

	//TODO: think about interpreting several landmarks aggregated to 1, (e.g. if there is one goal post, calculate the expected position of the other)
	// lfet right corners, can be described as aggregation of L and T crossings

	//distances to observed landmarks to landmarks on the map
	vector<double> dists;

	//index, landmark type, landmark index
	vector<vector<int> > landmarks;

	//for each feature found, try to map it with features on the map
	for(int i = 0; i < features.size();i++){
		double dist = 0;

		int landmark_index = find_landmark(features[i],current_pose, &dist);

		vector<int> feat;
		feat.push_back(features[i].type);
		feat.push_back(landmark_index);
		feat.push_back(dist);

		landmarks.push_back(feat);
		dists.push_back(dist);
	}


	//thrun et al "Alorithm landmark_mode_known_correspondence(fi,ci,x,m) pp 179:
	//for every assigned landmark (no unassigned landmarks possible at the moment)
	vector<double> q;

	for(int i = 0; i < dists.size(); i++){
		double ra = -1;
		double phi = -1;
		double res = -1;

		switch(landmarks[i][0]){
		case 0: //L crossing
		{
			double delta_x = this->feature_map.l_cross[landmarks[i][1]].x - current_pose->x;
			double delta_y = this->feature_map.l_cross[landmarks[i][1]].y - current_pose->y;

			//range
			ra = sqrt( delta_x * delta_x + delta_y * delta_y );//landmarks[i][2];
			//bearing
			phi = atan2(abs(this->feature_map.l_cross[landmarks[i][1]].y) - abs(current_pose->y ),abs(this->feature_map.l_cross[landmarks[i][1]].x )  -  abs(current_pose->x ));

			//how good is our estimate?
			res = this->measurement_factor * prob_gaussian(abs(features[i].range) - abs(ra),this->variance_range) * prob_gaussian(abs(features[i].bearing)  - abs(phi),this->variance_bearing) * prob_gaussian(0,1); //prob(si-sj,sigma_s) is last probability, how likeli ist that feature that landmark?

			break;
		}
		case 1: //T crossings
		{
			double delta_x = this->feature_map.t_cross[landmarks[i][1]].x - current_pose->x;
			double delta_y = this->feature_map.t_cross[landmarks[i][1]].y - current_pose->y;

			//range
			ra = sqrt( delta_x * delta_x + delta_y * delta_y );//landmarks[i][2];
			//bearing
			phi = atan2(abs(this->feature_map.t_cross[landmarks[i][1]].y) - abs(current_pose->y ),abs(this->feature_map.t_cross[landmarks[i][1]].x )  -  abs(current_pose->x ));

			//how good is our estimate?
			res = this->measurement_factor * prob_gaussian(abs(features[i].range) - abs(ra),this->variance_range) * prob_gaussian(abs(features[i].bearing)  - abs(phi),this->variance_bearing) * prob_gaussian(0,1); //prob(si-sj,sigma_s) is last probability, how likeli ist that feature that landmark?
			break;
		}
		case 2: //X crossings
		{
			double delta_x = this->feature_map.x_cross[landmarks[i][1]].x - current_pose->x;
			double delta_y = this->feature_map.x_cross[landmarks[i][1]].y - current_pose->y;
			//range
			ra = sqrt( delta_x * delta_x + delta_y * delta_y );//landmarks[i][2];
			//bearing
			phi = atan2(abs(this->feature_map.x_cross[landmarks[i][1]].y) - abs(current_pose->y ),abs(this->feature_map.x_cross[landmarks[i][1]].x )  -  abs(current_pose->x ));
			//how good is our estimate?
			res = this->measurement_factor * prob_gaussian(abs(features[i].range) - abs(ra),this->variance_range) * prob_gaussian(abs(features[i].bearing)  - abs(phi),this->variance_bearing) * prob_gaussian(0,1); //prob(si-sj,sigma_s) is last probability, how likeli ist that feature that landmark?

			break;
		}
		case 3: //Goal Post crossings
		{
			double delta_x = this->feature_map.g_cross[landmarks[i][1]].x - current_pose->x;
			double delta_y = this->feature_map.g_cross[landmarks[i][1]].y - current_pose->y;
			//range
			ra = sqrt( delta_x * delta_x + delta_y * delta_y );//landmarks[i][2];
			//bearing
			phi = atan2(abs(this->feature_map.g_cross[landmarks[i][1]].y) - abs(current_pose->y ),abs(this->feature_map.g_cross[landmarks[i][1]].x )  -  abs(current_pose->x ));
			//how good is our estimate?
			res = this->measurement_factor * prob_gaussian(abs(features[i].range) - abs(ra),this->variance_range) * prob_gaussian(abs(features[i].bearing)  - abs(phi),this->variance_bearing) * prob_gaussian(0,1); //prob(si-sj,sigma_s) is last probability, how likeli ist that feature that landmark?
			break;
		}
		default:
			cout<<"something went wrong in measurement model!"<<endl;
			break;
		}
		//we dont need smaller numbers
		if(res < 0.0000000001){ //if really small, just set to small value TODO: is there something wrong ?
			res = 0.0000000001;
		}
		//save result
		q.push_back(res);
	}

	//combining results for measurement via product
	double prod = 1;
	for (int i = 0; i < q.size(); i++){
		if(q[i] == 0){q[i] = 0.0001;}
		prod = prod * q[i];
	}
	return(prod);
}
/*
 * creates a number of particles with fixed poses:
 */
int ParticleFilter::add_particles(int number,double x, double y, double rot, double weight){

	//no comment.
	for (int i = 0; i< number; i++){
		Particle p;
		p.x = x;
		p.y = y;
		p.rot = rot;
		p.weight = 1;
		this->particles.push_back(p);
	}

}
/*
 * creates a number of particles with random poses
 */
int ParticleFilter::add_particles(int number) {
	//determines how accurate we want the particles to be. cm/accuracy
	double accuracy = 10;

	//create random samples
	for (int i = 0; i<number; i++){
		double x  = (rand()%(int)(this->x_dim*accuracy))/accuracy - this->x_dim/2;
		double y = (rand()%(int)(this->y_dim*accuracy))/accuracy - this->y_dim/2;
		double rot = (rand()%(int)(2*PI*accuracy))/accuracy-PI;
		double weight = 1;
		Particle p(x,y,rot,weight);
		this->particles.push_back(p);
	}
}
/*
 * gets the sum of the weigths, by iterating over the particle vector
 */
double ParticleFilter::sum_weights(){
	double sum = 0;
	for(int i = 0; i < this->particles.size(); i++ ){
		sum += this->particles[i].weight;
	}
	return(sum);
}
/*
 * prints out values for every particle in particle vector
 */
int ParticleFilter::print_particles() {
	cout<<"contents of particle list: x y rot weight no of particles: "<<this->particles.size()<<endl;
	for(int i = 0; i < this->particles.size(); i++){
		cout<<this->particles[i].x<<" "<<this->particles[i].y<<" "<<this->particles[i].rot<<" "<<this->particles[i].weight<<endl;
	}
}

/*
 * returns weighted avrg of all particles, final position estimate for current state
 */
int ParticleFilter::get_position_estimate(double* x_est,double* y_est,double* rot_est){
	double x = 0;
	double y = 0;
	double rot = 0;
	double sum = 0;

	//weighted average
	for(int i = 0; i < this->particles.size() ;  i++){
		double weight = particles[i].weight;
		sum += weight;
		x += particles[i].x * weight;
		y += particles[i].y * weight;
		rot += particles[i].rot * weight;
	}
	//save results
	*rot_est = rot / sum;
	*x_est = x / sum;
	*y_est = y / sum;
}
