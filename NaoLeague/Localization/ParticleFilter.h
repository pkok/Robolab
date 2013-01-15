/*
 * ParticleFilter.h
 *
 *  Created on: Jan 12, 2013
 *      Author: owner
 */
#include "Particle.h"
#include "FeatureMap.h"

struct VisualFeature {
	  double x;
	  double y;
	  int type; //0 : L-crossing, 1 : T-crossing 2 : X-crossing
	};
	struct OdometryInformation {
		int x;
		int y;
		double rot;
	};




class ParticleFilter {

public:

	//vars
	int width, height;
	int particle_count;
	Particle* particle_head;
	Particle* particle_current;

	//motion model variables
	//rotation ?uncertainty parameters
	double alpha1,alpha2;
	//translation ?
	double alpha3,alpha4;

	FeatureMap feature_map;

	//constructors
	ParticleFilter();
	ParticleFilter(int width, int height,int number_of_particles);
	virtual ~ParticleFilter();



	void test();


	//incorporate new odometry information (move and rotate particle)
	int sample_motion_model(OdometryInformation odometry_information,Particle* last_pose);
	int sample_motion_model_simple(OdometryInformation odometry_information,Particle* last_pose);
	//weight particles accroding to new sensory information (vision)
	double measurement_model(VisualFeature* feature,int no_observations ,Particle* current_pose,int map);

	//wander through all particles and update odometry and weight according to visual measurement
	int dynamic(OdometryInformation odo_inf);
	//reassign particle poses according to weight of particles
	int resample();
	//particle interaction
	int create_particles(int number);
	int delete_particle_list(Particle* particle_ptr);
	int count_particles();
	int print_particles();

/*
	//gaussian noise
	double random_uniform ();
	double random_gaussian ();
*/
	double sample_normal_distribution(double variance);
	double compute_prob_normal_dist(double a, double variance);
};

