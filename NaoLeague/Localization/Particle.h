/*
 * Particle.h
 *
 *  Created on: Jan 13, 2013
 *      Author: owner
 */

#ifndef PARTICLE_H_
#define PARTICLE_H_

class Particle {

public:
	////vars
	//x: field width, y field height, rot ranges from 0..2*pie*100, weight 0..100;
	double x,y;
	double rot,weight;
	//for linked list of particles
	Particle* next;

	//functions
	Particle();
	Particle(double x,double y, double rot,double weight);
	virtual ~Particle();
};

#endif /* PARTICLE_H_ */
