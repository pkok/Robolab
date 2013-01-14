/*
 * ParticleFilter.h
 *
 *  Created on: Jan 12, 2013
 *      Author: owner
 */
#include "Particle.h"

struct VisualInformation {
	  int range;
	  int bearing;
	};
	struct OdometryInformation {
		int x;
		int y;
		int rot;
	};




class ParticleFilter {

public:

	//vars
	int width, height;
	Particle* particle_head;
	Particle* particle_current;

	//constructors
	ParticleFilter();
	ParticleFilter(int width, int height,int number_of_particles);
	virtual ~ParticleFilter();



	void test();


	//incorporate new odometry information (move and rotate particle)
	Particle sample_motion_model(OdometryInformation odometry_information,Particle last_pose);

	//weight particles accroding to new sensory information (vision)
	int measurement_model(VisualInformation sensory_information,Particle current_pose,int map);

	//wander through all particles and update odometry and weight according to visual measurement
	int dynamic();
	//reassign particle poses according to weight of particles
	int resample();
	//particle interaction
	int create_particles(int number);
	int count_particles();
	int print_particles();


	//gaussian noise
	double random_uniform ();
	double random_gaussian ();
};
