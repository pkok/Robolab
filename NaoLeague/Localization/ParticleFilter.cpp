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

#define PI 3.14159

using namespace std;

/*
double ParticleFilter::random_uniform ()
{
  return ((double) rand())/RAND_MAX;
}

double ParticleFilter::random_gaussian ()
{
  double u = random_uniform(), v = random_uniform();
  return sqrt(-2*log(u))*cos(2*M_PI*v);
}
*/


ParticleFilter::ParticleFilter(){
	this->width = 720;
	this->height = 540;
	this->particle_head = 0;
	this->particle_current = 0;
	this->alpha1 = 0.1;
	this->alpha2 = 0.01;
	this->alpha3 = 0.01;
	this->alpha4 = 0.1;
	create_particles(150);
}
ParticleFilter::ParticleFilter(int width, int height,int number_of_particles){
	this->width = width;
	this->height = height;
	this->particle_head = 0;
	this->particle_current = 0;
	this->alpha1 = 0.1;
	this->alpha2 = 0.01;
	this->alpha3 = 0.01;
	this->alpha4 = 0.1;
	create_particles(number_of_particles);
}
ParticleFilter::~ParticleFilter(){

}
/*
 * Debug function
 */
void ParticleFilter::test() {
	std::cout << "hello from particleFilter!" << std::endl;
}
int ParticleFilter::resample() {

	////low variance resampling:

	//draw random number
	//get add fixed amount, to each bin we step into, sample from that particle
	//need to create new list?



	// read particle 0..M
	// draw with probability alpha w[i]t
	// add x[i]_t to Xt
}
int ParticleFilter::dynamic(OdometryInformation odo_inf){
	this->particle_current = this->particle_head;

	//get odometry information //TODO: check if correct

	while(this->particle_current != 0)
	{
		sample_motion_model_simple(odo_inf,this->particle_current);
		this->particle_current = this->particle_current->next;
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
int ParticleFilter::sample_motion_model_simple(OdometryInformation odometry_information,Particle* last_pose)
{
	//cout<<"sampling from motion model simple"<<endl;


	//noise 0.01 meter per 1 meter and 0.2 rad per 2 pi rad? (taken form old python codebase)
	double odometry_noise_x = odometry_information.x + odometry_information.x * (double)sample_normal_distribution(10)/1000;
	//cout<<"x "<< odometry_noise_x<<endl;
	double odometry_noise_y = odometry_information.y + odometry_information.y * (double)sample_normal_distribution(10)/1000;
	//cout<<"y "<< odometry_noise_y <<endl;
	double odometry_noise_rot = odometry_information.rot + odometry_information.rot * (double)sample_normal_distribution(20)/100;
	//cout<<"rot "<<odometry_noise_rot<<" "<<last_pose->rot<<endl;
	//cout<<odometry_noise_x<< " "<< odometry_noise_y<<" " << odometry_noise_rot<< endl;
	last_pose->x = last_pose->x +  odometry_noise_x * cos(last_pose->rot) + odometry_noise_y * sin(odometry_information.rot);
	last_pose->y = last_pose->y + odometry_noise_y * sin(last_pose->rot) -  odometry_noise_y * cos(odometry_information.rot);
	last_pose->rot = last_pose->rot +  odometry_noise_rot;
	//cout<<last_pose->x<< " "<< last_pose->y<<" "<<last_pose->rot<<endl;

}
//motion model with odometry (xt-1,xt)T and last pose (not working at the moment!)
int ParticleFilter::sample_motion_model(OdometryInformation odometry_information,Particle* last_pose)
{
	//Fox et al probabilistic robotics 136
	//last_pose.x = odometry_information.x + random_gaussian();
	//last_pose.y = odometry_information.y + random_gaussian();
	//last_pose.rot = odometry_information.rot + random_gaussian();


	//assuming last pose is identical to old odometry information.
	double delta_x = odometry_information.x - last_pose->x;
	double delta_y = odometry_information.y - last_pose->y;

	double delta_rot1 = atan2(delta_x,delta_y)-last_pose->rot;
	double delta_trans = sqrt( delta_x * delta_x + delta_y * delta_y);
	double delta_rot2 = odometry_information.rot - last_pose->x - delta_rot1;

	double delta_rot1_err = delta_rot1 - sample_normal_distribution(this->alpha1 * delta_rot1 * delta_rot1 + alpha2 * delta_trans * delta_trans);
	double delta_trans_err = delta_trans - sample_normal_distribution(this->alpha3 * delta_trans * delta_trans + this->alpha4 * delta_rot1 * delta_rot1 + this->alpha4 * delta_rot2 * delta_rot2);
	double delta_rot2_err = delta_rot2 - sample_normal_distribution(this->alpha1 * delta_rot2 * delta_rot2 + this->alpha2 * delta_trans * delta_trans);


	cout<<delta_rot1<< " "<<delta_rot2 <<" "<<delta_trans<<endl;
	cout<<"odo:"<<odometry_information.x<<" "<<odometry_information.y<<endl;

	//TODO: might have to return pointer instead of object!
	last_pose->x = last_pose->x + delta_trans_err * cos(last_pose->rot + delta_rot1_err);
	last_pose->y = last_pose->y + delta_trans_err * sin(last_pose->rot + delta_rot1_err);
	last_pose->rot = last_pose->rot + delta_rot1_err + delta_rot2_err;
}

/*
 * receives all visual features seen on the map and computes likelihood of a landmark measurment (with known correspondence)
 */
double ParticleFilter::measurement_model(VisualFeature* feature,int no_observations ,Particle* current_pose,int map)
{
	//TODO if this does not work, change !! (assuming, same features will be observed with same probability) 1/n where n number of features of that type in the map
	//assume the nearest landmark is the one observed


	//iterate through all features of that type, calculate distance to feature observed in map
	//choose feature with smallest distance and then proceed
	double min_dist = 1000; //10 meter
	double feature_index = 0;

	switch (feature->type){
	case 0: //L crossing observed 8
	{
		for(int i = 0;i < 8; i++){
			//calculate distance
			double delta_x = feature->x - this->feature_map.l_cross[i].x ;
			double delta_y = feature->y - this->feature_map.l_cross[i].y ;
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
			double delta_x = feature->x - this->feature_map.t_cross[i].x ;
			double delta_y = feature->y - this->feature_map.t_cross[i].y ;
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
			double delta_x = feature->x - this->feature_map.t_cross[i].x ;
			double delta_y = feature->y - this->feature_map.t_cross[i].y ;
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



	//prob of right bearing * prob of right range * prob of being that landmark
	return(0);
}
/*
 * creates a number of particles with random poses
 */
int ParticleFilter::create_particles(int number) {

	int x = 250;//rand()%this->width;
	int y = 250;//rand()%this->height;
	double rot = 0;//rand()%630;
	double weight = 1;
	Particle* head_node = new Particle(x,y,rot,weight);
	this->particle_head = head_node;
	this->particle_current = head_node;
	Particle* current = this->particle_head;


	//create random samples
	for (int i = 1; i<number; i++){
		int x  = 250;//rand() % width;
		int y = 250;//rand() % height;
		double rot = 0;//rand() % 630;
		double weight = 1;
		Particle* new_node = new Particle(x,y,rot,weight);
		current->next = new_node;
		current = current->next;
	}
}

int ParticleFilter::count_particles() {
	Particle* ptr = this->particle_head;
	int count = 0;
	while(ptr != 0){
		ptr = ptr->next;
		count++;
	}
	return (count);
}


int ParticleFilter::print_particles() {
	Particle* ptr = this->particle_head;
	int ctr = 0;
	while(ptr!=0){
		cout<<"particle "<<ctr<<" x="<<ptr->x<<" y="<<ptr->y<<" rot="<<ptr->rot<<endl;
		ptr = ptr->next;
		ctr++;
	}
}
