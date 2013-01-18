/*
 * ParticleFilter.h
 *
 *  Created on: Jan 12, 2013
 *      Author: owner
 */
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

	//vars
	int x_dim;
	int y_dim;

	vector<Particle> particles;

	FeatureMap feature_map;

	//constructors
	ParticleFilter();
	ParticleFilter(int width, int height,int number_of_particles);
	virtual ~ParticleFilter();

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


	//gaussian noise
	//sampling
	double random_uniform ();
	double random_gaussian ();
	//probability calculation
	double prob_gaussian(double val,double var_sq);

};

