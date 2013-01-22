/*
 * ParticleFilter.h
 *
 *  Created on: Jan 12, 2013
 *      Author: owner
 */
#ifndef PARTICLEFILTER_H_
#define PARTICLEFILTER_H_
#include "Particle.h"
#include "FeatureMap.h"
#include <vector>

using namespace std;

struct VisualFeature {
	  double range; //according to camera position
	  double bearing; //according to global coordinate system
	  int type; //0 : L-crossing, 1 : T-crossing 2 : X-crossing
	};
	struct OdometryInformation {
		double x;
		double y;
		double rot;
	};




class ParticleFilter {

public:

	bool augmented;
	//vars
	int x_dim;
	int y_dim;

	double variance_range;
	double variance_bearing;

	//noise added in resampling
	double resample_variance_pos;
	double resample_variance_rot;

	//factor to increase probability estimate of visual features
	double measurement_factor;


	//motion model update
	double error_range;
	double error_bearing;

	//additional parameters for augmented Monte carlo method:
	double alpha_slow;
	double alpha_fast;

	double w_slow;
	double w_fast;


	vector<Particle> particles;

	FeatureMap feature_map;

	//constructors
	ParticleFilter();
	ParticleFilter(int width, int height,int number_of_particles);
	virtual ~ParticleFilter();
	//set parameters
	void set_params();

	void test();


	//incorporate new odometry information (move and rotate particle)
	int sample_motion_model_simple(OdometryInformation odometry_information,Particle* last_pose);
	//weight particles accroding to new sensory information (vision)
	double measurement_model(vector<VisualFeature> features,Particle* current_pose);
	//finds most likeli landmark for given feature observation
	int find_landmark(VisualFeature feature, Particle* current_pose,double* dist);

	//wander through all particles and update odometry and weight according to visual measurement
	int dynamic(OdometryInformation odo_inf,vector<VisualFeature> vis_feats);
	//reassign particle poses according to weight of particles
	int resample();
	//particle interaction


	int add_particles(int number,double x, double y, double rot, double weight);
	int add_particles(int number);
	int delete_particles();
	int print_particles();
	double sum_weights();
	//gets the final weighted avrg of all particles for estimate of psoition
	int get_position_estimate(double* x_est,double* y_est,double* rot_est);

	//gaussian noise
	//sampling
	double random_uniform ();
	double random_gaussian ();
	//probability calculation
	double prob_gaussian(double val,double var_sq);

};
#endif

