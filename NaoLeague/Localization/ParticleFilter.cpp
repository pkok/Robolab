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



ParticleFilter::ParticleFilter(){
	//random seed:
	srand(time(NULL));
	this->x_dim = 740;
	this->y_dim = 540;
}
ParticleFilter::ParticleFilter(int width, int height,int number_of_particles){
	//random seed:
	srand(time(0));

	this->x_dim = 740;
	this->y_dim = 540;

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
	//get add fixed amount, to each bin we step into, sample from that particle
	//need to create new list?
	/*
	assert(this->particle_count>0);
	double m_inv =  1/this->particle_count;
	double r = rand() % ( m_inv * 1000000 ) / 1000000;
	double c = this->particle_head;
	*/
	// read particle 0..M
	// draw with probability alpha w[i]t
	// add x[i]_t to Xt

	vector<Particle> resampled_particles;

	double bin_border = this->particles[0].weight;
	double weight_sum = 0;

	double step_size = sum_weights()/(double) this->particles.size();

	double offset = rand() % (int)(step_size*1000)/(double)1000;

	int j = 1;
	for(int i = 0; i < this->particles.size(); i++){
		weight_sum =  offset + (i-1) * step_size;
		while(weight_sum >= bin_border){
			//cout<<j<<endl;
			j++;
			bin_border += this->particles[j].weight;
		}
		Particle p(this->particles[j].x,this->particles[j].y,this->particles[j].rot,1);
		resampled_particles.push_back(p);
	}
	this->particles = resampled_particles;
}
int ParticleFilter::dynamic(OdometryInformation odo_inf){

	//get odometry information //TODO: check if correct

	for(int i = 0; i < this->particles.size(); i++)
	{
		sample_motion_model_simple(odo_inf,&this->particles[i]);

	}
	//for 0..M
		// pose = sample_sample_motion_model
		// weight = measurement_model

	//CHECK FOR PARTICLES OUTSIDE THE FIELD
}

/*
 * computes the probability for x on a zero mean gaussian with variance variance
 */
double ParticleFilter::compute_prob_normal_dist(double a, double variance)
{
	double prob = 0;

	prob = 1/sqrt( 2*PI ) * exp( -a * a / ( 2 * variance ) );

	return(prob);
}


/*
 * sample from a normal distribution
 */

double ParticleFilter::sample_normal_distribution(double variance){
	double sqrt_var = sqrt(variance);
	int sqrt_var_2 = sqrt_var * 2;
	double sum = 0;

	for(int i = 0; i< 12;i++){
		//sample from -b/+b
		double rand_d = rand() % sqrt_var_2 - sqrt_var;

		sum += rand_d;
	}

	return(((double) sum) / 2);
}
/*
 * simple method to incorporate new odometry data
 */
int ParticleFilter::sample_motion_model_simple(OdometryInformation odometry_information,Particle* last_pose)
{

	double error_range = 0.01;
	double error_bearing = 0.2;
	double error_x = odometry_information.x * this->random_gaussian()* error_range;
	double error_y = odometry_information.y * this->random_gaussian()* error_range;
	double error_rot = odometry_information.rot * this->random_gaussian() * error_bearing;

	last_pose->x = last_pose->x+odometry_information.x + error_x;
	last_pose->y = last_pose->y+odometry_information.y + error_y;
	last_pose->rot = last_pose->rot + odometry_information.rot + error_rot;

	//TODO: reject particles outside the field
}

/*
 * receives all visual features seen on the map and computes likelihood of a landmark measurment (with known correspondence)
 */
double ParticleFilter::measurement_model(VisualFeature* feature,int no_observations ,Particle* current_pose,int map)
{
	//TODO if this does not work, change !! (assuming, same features will be observed with same probability) 1/n where n number of features of that type in the map
	//assume the nearest landmark is the one observed


	//find the the most likeli landmark that can be assigned to the feature and assume its the rigth one:

	//calculate position on map(assuming we are in the current pose):
	int x_feat = current_pose->x + sin(feature->bearing)*feature->range;
	int y_feat = current_pose->y + cos(feature->bearing)*feature->range;







	//iterate through all features of that type, calculate distance to feature observed in map
	//choose feature with smallest distance and then proceed
	double min_dist = 1000; //10 meter
	double feature_index = 0;

	switch (feature->type){
	case 0: //L crossing observed 8
	{
		for(int i = 0;i < 8; i++){
			//calculate distance
			double delta_x = x_feat - this->feature_map.l_cross[i].x ;
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
	default:
		cout<<"something went terribly wrong in measurement model!" <<endl;
		break;
	}

	cout<<"measurement model reporting min distance found: "<<min_dist<<" for feature "<<feature_index<<endl;

	//prob of right bearing * prob of right range * prob of being that landmark
	return(0);
}
/*
 * creates a number of particles with fixed poses:
 */
int ParticleFilter::add_particles(int number,double x, double y, double rot, double weight){
	//determines how accurate we want the particles to be. cm/accuracy

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
	for (int i = 1; i<number; i++){
		double x  = (rand()%(int)(this->x_dim*accuracy))/accuracy - this->x_dim/2;
		double y = (rand()%(int)(this->y_dim*accuracy))/accuracy - this->y_dim/2;
		double rot = (rand()%(int)(2*PI*accuracy))/accuracy-PI;//rand() % 630;
		double weight = 1;
		Particle p(x,y,rot,weight);
		this->particles.push_back(p);
	}
}

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
	cout<<"contents of particle list: x y rot weight"<<endl;
	for(int i = 0; i < this->particles.size(); i++){
		cout<<this->particles[i].x<<" "<<this->particles[i].y<<" "<<this->particles[i].rot<<" "<<this->particles[i].weight<<endl;
	}

}
