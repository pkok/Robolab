/*
 * ParticleFilter.cpp
 *
 *  Created on: Jan 12, 2013
 *      Author: owner
 */

#include "ParticleFilter.h"
#include <iostream>
#include <cstdlib>

using namespace std;

ParticleFilter::ParticleFilter(){
	this->width = 720;
	this->height = 540;
	this->particle_head = 0;
	this->particle_current = 0;
}
ParticleFilter::ParticleFilter(int width, int height){
	this->width = width;
	this->height = height;
	this->particle_head = 0;
	this->particle_current = 0;
}
ParticleFilter::~ParticleFilter(){

}

void ParticleFilter::test() {
	std::cout << "hello from particleFilter!" << std::endl;
}
int ParticleFilter::resample() {

}
int ParticleFilter::dynamic() {

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
