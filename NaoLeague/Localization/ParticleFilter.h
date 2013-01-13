/*
 * ParticleFilter.h
 *
 *  Created on: Jan 12, 2013
 *      Author: owner
 */
#include "Particle.h"


class ParticleFilter {

public:
	//vars
	int width, height;
	Particle* particle_head;
	Particle* particle_current;

	//constructors
	ParticleFilter();
	ParticleFilter(int width, int height);
	virtual ~ParticleFilter();



	void test();
	int resample();
	int dynamic();
	//particle interaction
	int create_particles(int number);
	int count_particles();
	int print_particles();
};
