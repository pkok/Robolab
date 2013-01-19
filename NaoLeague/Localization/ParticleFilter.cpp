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
#include <assert.h>
#include <time.h>
#include <vector>

#define PI 3.14159

using namespace std;


double ParticleFilter::random_uniform ()
{
  return ((double) rand())/RAND_MAX;
}
/*
 * box muller transform
 */
double ParticleFilter::random_gaussian ()
{
  double u = random_uniform(), v = random_uniform();
  return sqrt(-2*log(u))*cos(2*M_PI*v);
}
/*
 * computes the probability of a point in a normal distribution
 */
double ParticleFilter::prob_gaussian(double val,double var_sq){
	return(1/sqrt(2*PI*var_sq)*exp(- ( val * val )/ (2 * var_sq) ));
}

ParticleFilter::ParticleFilter(){
	//random seed:
	srand(time(NULL));
	this->x_dim = 740;
	this->y_dim = 540;
}
ParticleFilter::ParticleFilter(int width, int height,int number_of_particles){
	//random seed:
	srand(time(0));

	this->x_dim = 740;
	this->y_dim = 540;

}
ParticleFilter::~ParticleFilter(){

}
/*
 * Debug function
 */
void ParticleFilter::test() {
	std::cout << "hello from particleFilter!" << std::endl;
}
/*
 * deletes the linked list of particles and all its descendants
 */
int ParticleFilter::delete_particles(){

}
/*
 * resamples all particles with low variance resampling algorithm. (creates new particle list)
 */
int ParticleFilter::resample() {
	//cout<<"start resampling"<<endl;
	////low variance resampling:

	//draw random number
	//get add fixed amount, to each bin we step into, sample from that particle
	//need to create new list?
	/*
	assert(this->particle_count>0);
	double m_inv =  1/this->particle_count;
	double r = rand() % ( m_inv * 1000000 ) / 1000000;
	double c = this->particle_head;
	*/
	// read particle 0..M
	// draw with probability alpha w[i]t
	// add x[i]_t to Xt

	vector<Particle> resampled_particles;

	double c = this->particles[0].weight;
	double u = 0;

	assert(this->particles.size() >= 0);

	double step_size = sum_weights()/(double) this->particles.size();

	assert(step_size >= 0);
	double offset = ((double)rand() / (double)RAND_MAX) * step_size;
	assert(offset > 0);

	int j = 0;

	//cout<< " while loop"<<endl;
	for(int i = 1; i <= this->particles.size(); i++){
		u =  offset + (i-1) * step_size;
		while(u >= c){
			//cout<<j<<endl;
			j++;
			c += this->particles[j].weight;
		}
		Particle p(this->particles[j].x,this->particles[j].y,this->particles[j].rot,1);
		resampled_particles.push_back(p);

		//cout<< "inside while loop"<<endl;
	}
	this->particles = resampled_particles;

	//cout<<"end resampling"<<endl;
}
int ParticleFilter::dynamic(OdometryInformation odo_inf,vector<VisualFeature> vis_feats){

	//get odometry information //TODO: check if correct

	for(int i = 0; i < this->particles.size(); i++)
	{
		sample_motion_model_simple(odo_inf,&this->particles[i]);
		this->particles[i].weight = this->measurement_model(vis_feats,&this->particles[i]);


		/*
		if(this->particles[i].weight < 0){
			cout<<" the impossible has happened!!"<<endl;
		}
		if(this->particles[i].x == 100 && this->particles[i].y == 0){
			cout<<"still one particle at 100,0"<<endl;
		}
		if(this->particles[i].weight > 0.00001){
			cout<<" yey!"<<this->particles[i].x<<" "<<this->particles[i].y<<" "<<this->particles[i].rot<<" "<<this->particles[i].weight<<endl;
		}*//*
		//TODO: dirty hack, should probably not be there
		if(this->particles[i].weight < 0.00001) //small value
		{
			this->particles[i].weight = 0.00001;
		}*/
		//if(this->particles[i].weight>0){
		//cout<<"found good estimate at : "<<this->particles[i].x<<" "<<this->particles[i].y<<" "<<this->particles[i].rot<<" weight is:"<<this->particles[i].weight<<endl;
		//}
		//if(this->particles[i].weight> 0){
		//	cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!look at me !!!!!!!!!!!!!!!!!!! "<<this->particles[i].weight<<endl;
		//}
	}
	//for 0..M
		// pose = sample_sample_motion_model
		// weight = measurement_model

	//CHECK FOR PARTICLES OUTSIDE THE FIELD
}

/*
 * simple method to incorporate new odometry data
 */
int ParticleFilter::sample_motion_model_simple(OdometryInformation odometry_information,Particle* last_pose)
{

	double error_range = 0.01;
	double error_bearing = 0.2;
	double error_x =  this->random_gaussian() * error_range * odometry_information.x;
	double error_y = this->random_gaussian() * error_range * odometry_information.y  ;
	double error_rot =  this->random_gaussian() * odometry_information.rot  * error_bearing;

	last_pose->x = last_pose->x + odometry_information.x + error_x;
	last_pose->y = last_pose->y + odometry_information.y + error_y;
	last_pose->rot = last_pose->rot + odometry_information.rot + error_rot;

	//TODO: reject particles outside the field
}
int ParticleFilter::find_landmark(VisualFeature feature, Particle* current_pose,double* dist){
	//iterate through all features of that type, calculate distance to feature observed in map

	//TODO: Include mahalanobis distance (takes into account that measurement error in different dimensions of the data is different) i.e. range and bearing.

	//calculate position on map(assuming we are in the current pose):
	int x_feat = current_pose->x + cos(feature.bearing+current_pose->rot)*feature.range;
	int y_feat = current_pose->y + sin(feature.bearing+current_pose->rot)*feature.range;
		//choose feature with smallest distance and then proceed
		double min_dist = 1000; //10 meter
		int feature_index = 0;

		switch (feature.type){
		case 0: //L crossing observed 8
		{
			for(int i = 0;i < 8; i++){
				//calculate distance
				double delta_x =  x_feat - this->feature_map.l_cross[i].x ;
				double delta_y = y_feat - this->feature_map.l_cross[i].y ;
				double dist =  sqrt(delta_x * delta_x + delta_y * delta_y);
				////cout<<dist<<" "<<i<<" "<<this->feature_map.l_cross[i].x<<" "<<this->feature_map.l_cross[i].y<<" "<<current_pose->x<<endl;
				if(dist < min_dist){

					min_dist = dist;
					feature_index = i;

				}
			}
			break;
		}
		case 1: //T crossing observed 7
		{

			for(int i = 0; i<7; i++){
				//calculate distance
				double delta_x = x_feat - this->feature_map.t_cross[i].x ;
				double delta_y = y_feat - this->feature_map.t_cross[i].y ;
				double dist =  sqrt(delta_x * delta_x + delta_y * delta_y);
				if(dist < min_dist){
					min_dist = dist;
					feature_index = i;
				}
			}
			break;
		}
		case 2: //X crossing observed 5
		{
			for(int i = 0;i<5;i++){
				//dist calculation
				double delta_x = x_feat - this->feature_map.x_cross[i].x ;
				double delta_y = y_feat - this->feature_map.x_cross[i].y ;
				double dist =  sqrt(delta_x * delta_x + delta_y * delta_y);
				if(dist < min_dist){
					min_dist = dist;
					feature_index = i;
				}
			}
			break;
		}
		case 3: //Goal post crossing crossing observed 4
		{
			for(int i = 0;i<4;i++){
				//dist calculation
				double delta_x = x_feat - this->feature_map.g_cross[i].x ;
				double delta_y = y_feat - this->feature_map.g_cross[i].y ;
				double dist =  sqrt(delta_x * delta_x + delta_y * delta_y);
				if(dist < min_dist){
					min_dist = dist;
					feature_index = i;
				}
			}
			break;
		}
		default:
			cout<<"something went terribly wrong in measurement model!" <<endl;
			break;
		}
		////cout<<"found landmark "<<feature_index<<" with dist:"<<*dist<<" specs:"<<this->feature_map.l_cross[feature_index].x<<" "<<this->feature_map.l_cross[feature_index].y <<endl;
		*dist = min_dist;
		return(feature_index);
}
/*
 * receives all visual features seen on the map and computes likelihood of a landmark measurment (with known correspondence)
 */
double ParticleFilter::measurement_model(vector<VisualFeature> features,Particle* current_pose)
{
	//TODO if this does not work, change !!
	//assume the nearest landmark is the one observed (ML approach)

	//TODO: think about interpreting several landmarks aggregated to 1, (e.g. if there is one goal post, calculate the expected position of the other)
	// lfet right corners, can be described as aggregation of L and T crossings
	//cout<<"start measurement"<<endl;
	//changeable parameters:
	double range_param = 3;
	double bearing_param = 2;
	double meas_param = 1;

	vector<double> dists;
	vector<vector<int> > landmarks; //TODO:maybe find some better representation
	for(int i = 0; i < features.size();i++){
		double dist = 0;
		////cout<<"finding ml landmark for "<<current_pose->x<<" "<<current_pose->y<<endl;
		int landmark_index = find_landmark(features[i],current_pose, &dist);
		vector<int> feat;
		feat.push_back(features[i].type);
		feat.push_back(landmark_index);
		feat.push_back(dist);
		landmarks.push_back(feat);
		dists.push_back(dist);
		//cout<<"measurement model reporting min distance found feature no"<<landmark_index<<" in distance "<<dist<<endl;
	}


	//thrun et al "Alorithm landmark_mode_known_correspondence(fi,ci,x,m) pp 179:
	//for every assigned landmark (no unassigned landmarks possible at the moment)
	vector<double> q;

	for(int i = 0; i < dists.size(); i++){
		double ra = -1;
		double phi = -1;
		double res = -1;

		switch(landmarks[i][0]){
		case 0: //L crossing
		{
			double delta_x = this->feature_map.l_cross[landmarks[i][1]].x - current_pose->x;
			double delta_y = this->feature_map.l_cross[landmarks[i][1]].y - current_pose->y;

			ra = sqrt( delta_x * delta_x + delta_y * delta_y );//landmarks[i][2];
			//added abs
			phi = atan2(abs(this->feature_map.l_cross[landmarks[i][1]].y) - abs(current_pose->y ),abs(this->feature_map.l_cross[landmarks[i][1]].x )  -  abs(current_pose->x ));


			////cout<< delta_x << " "<<delta_y<<" "<<ra<<" "<<phi<<" "<<"robot:"<<current_pose->rot<<" feat:"<<features[i].range<<" "<<features[i].bearing;
			////cout <<" "<<prob_gaussian(abs(features[i].range) - abs(ra),1) <<" "<< prob_gaussian(abs(features[i].bearing)  - abs(phi),1) <<" "<< prob_gaussian(0,1) <<" "<<endl;

			res = prob_gaussian(abs(features[i].range) - abs(ra),1) * prob_gaussian(abs(features[i].bearing)  - abs(phi),1) * prob_gaussian(0,1); //prob(si-sj,sigma_s) is last probability, how likeli ist that feature that landmark?
			////cout<<res<<endl;
			break;
		}
		case 1: //T crossings
		{
			double delta_x = this->feature_map.t_cross[landmarks[i][1]].x - current_pose->x;
			double delta_y = this->feature_map.t_cross[landmarks[i][1]].y - current_pose->y;

			ra = sqrt( delta_x * delta_x + delta_y * delta_y );//landmarks[i][2];
			//added abs
			phi = atan2(abs(this->feature_map.t_cross[landmarks[i][1]].y) - abs(current_pose->y ),abs(this->feature_map.t_cross[landmarks[i][1]].x )  -  abs(current_pose->x ));


			////cout<< delta_x << " "<<delta_y<<" "<<ra<<" "<<phi<<" "<<"robot:"<<current_pose->rot<<" feat:"<<features[i].range<<" "<<features[i].bearing;
			////cout <<" "<<prob_gaussian(abs(features[i].range) - abs(ra),1) <<" "<< prob_gaussian(abs(features[i].bearing)  - abs(phi),1) <<" "<< prob_gaussian(0,1) <<" "<<endl;

			res = prob_gaussian(abs(features[i].range) - abs(ra),1) * prob_gaussian(abs(features[i].bearing)  - abs(phi),1) * prob_gaussian(0,1); //prob(si-sj,sigma_s) is last probability, how likeli ist that feature that landmark?
			////cout<<res<<endl;
			break;

		}
		case 2: //X crossings
		{


			double delta_x = this->feature_map.x_cross[landmarks[i][1]].x - current_pose->x;
			double delta_y = this->feature_map.x_cross[landmarks[i][1]].y - current_pose->y;

			ra = sqrt( delta_x * delta_x + delta_y * delta_y );//landmarks[i][2];
			//added abs
			phi = atan2(abs(this->feature_map.x_cross[landmarks[i][1]].y) - abs(current_pose->y ),abs(this->feature_map.x_cross[landmarks[i][1]].x )  -  abs(current_pose->x ));


			////cout<< delta_x << " "<<delta_y<<" "<<ra<<" "<<phi<<" "<<"robot:"<<current_pose->rot<<" feat:"<<features[i].range<<" "<<features[i].bearing;
			////cout <<" "<<prob_gaussian(abs(features[i].range) - abs(ra),1) <<" "<< prob_gaussian(abs(features[i].bearing)  - abs(phi),1) <<" "<< prob_gaussian(0,1) <<" "<<endl;

			res = prob_gaussian(abs(features[i].range) - abs(ra),1) * prob_gaussian(abs(features[i].bearing)  - abs(phi),1) * prob_gaussian(0,1); //prob(si-sj,sigma_s) is last probability, how likeli ist that feature that landmark?
			////cout<<res<<endl;
			break;


		}
		case 3: //Goal Post crossings
		{
			double delta_x = this->feature_map.g_cross[landmarks[i][1]].x - current_pose->x;
			double delta_y = this->feature_map.g_cross[landmarks[i][1]].y - current_pose->y;

			ra = sqrt( delta_x * delta_x + delta_y * delta_y );//landmarks[i][2];
			//added abs
			phi = atan2(abs(this->feature_map.g_cross[landmarks[i][1]].y) - abs(current_pose->y ),abs(this->feature_map.g_cross[landmarks[i][1]].x )  -  abs(current_pose->x ));


			////cout<< delta_x << " "<<delta_y<<" "<<ra<<" "<<phi<<" "<<"robot:"<<current_pose->rot<<" feat:"<<features[i].range<<" "<<features[i].bearing;
			////cout <<" "<<prob_gaussian(abs(features[i].range) - abs(ra),1) <<" "<< prob_gaussian(abs(features[i].bearing)  - abs(phi),1) <<" "<< prob_gaussian(0,1) <<" "<<endl;

			res = prob_gaussian(abs(features[i].range) - abs(ra),1) * prob_gaussian(abs(features[i].bearing)  - abs(phi),1) * prob_gaussian(0,1); //prob(si-sj,sigma_s) is last probability, how likeli ist that feature that landmark?
			////cout<<res<<endl;
			break;
		}
		default:
			cout<<"something went wrong in measurement model!"<<endl;
			break;
		}
		//cout<<ra<<" "<<phi<<" "<<res<<";";
		q.push_back(res);
	}
	//cout<<endl;

	//return product of all probabilities?
	double prod = 1;
	for (int i = 0; i < q.size(); i++){

		prod = prod *  q[i]* range_param * bearing_param * meas_param;
		//cout<<" "<<q[i];
	}
	//cout<<endl;
	//cout<<" final weight estimate :"<<prod<<endl;
	//cout<<"completed measurement"<<endl;
	return(prod);
}
/*
 * creates a number of particles with fixed poses:
 */
int ParticleFilter::add_particles(int number,double x, double y, double rot, double weight){
	//determines how accurate we want the particles to be. cm/accuracy

	for (int i = 0; i< number; i++){
		Particle p;
		p.x = x;
		p.y = y;
		p.rot = rot;
		p.weight = 1;
		this->particles.push_back(p);
	}

}
/*
 * creates a number of particles with random poses
 */
int ParticleFilter::add_particles(int number) {
	//determines how accurate we want the particles to be. cm/accuracy
	double accuracy = 10;

	//create random samples
	for (int i = 0; i<number; i++){
		double x  = (rand()%(int)(this->x_dim*accuracy))/accuracy - this->x_dim/2;
		double y = (rand()%(int)(this->y_dim*accuracy))/accuracy - this->y_dim/2;
		double rot = (rand()%(int)(2*PI*accuracy))/accuracy-PI;//rand() % 630;
		double weight = 1;
		Particle p(x,y,rot,weight);
		this->particles.push_back(p);
	}
}

double ParticleFilter::sum_weights(){
	double sum = 0;
	for(int i = 0; i < this->particles.size(); i++ ){
		sum += this->particles[i].weight;
	}
	return(sum);
}
/*
 * prints out values for every particle in particle vector
 */
int ParticleFilter::print_particles() {
	cout<<"contents of particle list: x y rot weight no of particles: "<<this->particles.size()<<endl;
	for(int i = 0; i < this->particles.size(); i++){
		cout<<this->particles[i].x<<" "<<this->particles[i].y<<" "<<this->particles[i].rot<<" "<<this->particles[i].weight<<endl;
	}
}

/*
 * returns weighted avrg of all particles, final position estimate for current state
 */
int ParticleFilter::get_position_estimate(double* x_est,double* y_est,double* rot_est){
	double x = 0;
	double y = 0;
	double rot = 0;
	double sum = 0;
	for(int i = 0; i < this->particles.size() ;  i++){
		double weight = particles[i].weight;
		sum += weight;
		x += particles[i].x * weight;
		y += particles[i].y * weight;
		rot += particles[i].rot * weight;
		//cout<<rot<<endl;
	}
	*rot_est = rot / sum;
	*x_est = x / sum;
	*y_est = y / sum;
}
