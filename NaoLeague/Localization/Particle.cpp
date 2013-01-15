/*
 * Particle.cpp
 *
 *  Created on: Jan 13, 2013
 *      Author: owner
 */
#include <assert.h>
#include "Particle.h"

Particle::Particle() {
	// TODO Auto-generated constructor stub
	this->x = 0;
	this->y = 0;
	this->rot = 0;
	this->next = 0; //null pointer

}
Particle::Particle(double x,double y, double rot,double weight)
{
	assert(weight <= 1);
	this->weight = weight;
	this->x = x;
	this->y = y;
	this->rot = rot;
	this->next = 0;
}
Particle::~Particle() {
	// TODO Auto-generated destructor stub
}

