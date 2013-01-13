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

using namespace std;


double ParticleFilter::random_uniform ()
{
  return ((double) rand())/RAND_MAX;
}

double ParticleFilter::random_gaussian ()
{
  double u = random_uniform(), v = random_uniform();
  return sqrt(-2*log(u))*cos(2*M_PI*v);
}


ParticleFilter::ParticleFilter(){
	this->width = 720;
	this->height = 540;
	this->particle_head = 0;
	this->particle_current = 0;
	create_particles(150);
}
ParticleFilter::ParticleFilter(int width, int height,int number_of_particles){
	this->width = width;
	this->height = height;
	this->particle_head = 0;
	this->particle_current = 0;
	create_particles(number_of_particles);
}
ParticleFilter::~ParticleFilter(){

}

void ParticleFilter::test() {
	std::cout << "hello from particleFilter!" << std::endl;
}
int ParticleFilter::resample() {

	// read particle 0..M
	// draw with probability alpha w[i]t
	// add x[i]_t to Xt
}
int ParticleFilter::dynamic(){

	//for 0..M
		// pose = sample_sample_motion_model
		// weight = measurement_model

	//CHECK FOR PARTICLES OUTSIDE THE FIELD
}
Particle ParticleFilter::sample_motion_model(OdometryInformation odometry_information,Particle last_pose)
{
	last_pose.x = odometry_information.x + random_gaussian();
	last_pose.y = odometry_information.y + random_gaussian();
	last_pose.rot = odometry_information.rot + random_gaussian();
}
int measurement_model(VisualInformation sensory_information,Particle current_pose,int map)
{
	return(0);
}
/*
 * creates a number of particles with random poses
 */
int ParticleFilter::create_particles(int number) {

	int x = rand()%this->width;
	int y = rand()%this->height;
	int rot = rand()%630;
	int weight = 100/number;
	Particle* head_node = new Particle(x,y,rot,weight);
	this->particle_head = head_node;
	this->particle_current = head_node;
	Particle* current = this->particle_head;


	//create random samples
	for (int i = 1; i<number; i++){
		int x  = rand() % width;
		int y = rand() % height;
		int rot = rand() % 630;
		int weight = 100/number;
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
