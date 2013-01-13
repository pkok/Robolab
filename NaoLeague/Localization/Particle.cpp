/*
 * Particle.cpp
 *
 *  Created on: Jan 13, 2013
 *      Author: owner
 */

#include "Particle.h"

Particle::Particle() {
	// TODO Auto-generated constructor stub
	this->x = 0;
	this->y = 0;
	this->rot = 0;
	this->next = 0; //null pointer

}
Particle::Particle(int x,int y, int rot,int weight)
{
	this->weight = weight;
	this->x = x;
	this->y = y;
	this->rot = rot;
	this->next = 0;
}
Particle::~Particle() {
	// TODO Auto-generated destructor stub
}

