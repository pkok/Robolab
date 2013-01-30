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
	this->set_params();
}
void ParticleFilter::set_params(){

	//type of MCL:
	this->augmented = true;

	//error parameters
	this->error_range = 0.1;//0.1;
	this->error_bearing = 0.2;//0.2;

	this->resample_variance_pos = 10; //20
	this->resample_variance_rot = 0.1;//1;

	this->measurement_factor = 1;

	this->variance_range = 10000;//200;//2000
	this->variance_bearing = 200;//200;


	//augmented pf parameters
	////0<=alpha_slow<<alpha_fast;
	//value taken from python code
	this->alpha_slow = 0.05;//0.05;
	this->alpha_fast = 0.5;//0.3;

	this->w_slow = 0;
	this->w_fast = 0;

	this->x_dim = 740;
	this->y_dim = 540;
}




ParticleFilter::ParticleFilter(double error_range,double error_bearing,double resample_variance_pos,double resample_variance_rot,double variance_range,double variance_bearing){
	//random seed:
	srand(time(0));
	this->error_range = error_range;//0.1;
	this->error_bearing = error_bearing;//0.2;

	this->resample_variance_pos = resample_variance_pos; //20
	this->resample_variance_rot = resample_variance_rot;

	this->measurement_factor = measurement_factor;

	this->variance_range = variance_range;//200;//2000
	this->variance_bearing = variance_bearing;//200;
	this->augmented = true;
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
 * samples from given landmark with range and bearing. must be of the same type!
 */
Particle ParticleFilter::sample_landmark_model(LandMark lm, VisualFeature vf){
	//TODO: make more efficient:
	double x = -1000000;
	double y = -1000000;
	Particle p;
	while( !( (-this->x_dim/(double)2 < x ) && (  this->x_dim/(double)2 > x) && (-this->y_dim/(double)2 < y ) && (this->y_dim/(double)2 > y ) ) ){
		double circle_pos = random_uniform() * 2* PI;
		double range = vf.range + random_gaussian() ;
		double bearing = vf.bearing + random_gaussian();
		x = lm.pos.x + range * cos(circle_pos);
		y = lm.pos.y + range * sin(circle_pos);
		double rot = circle_pos - PI - bearing;
		p.x = x;
		p.y = y;
		p.rot = rot;
		p.weight = 1;
	}
	return(p);
}
/*
 * resamples all particles with low variance resampling algorithm. (creates new particle list)
 */
int ParticleFilter::resample(VisualFeature vf) {

	////low variance resampling:

	//draw random number
	// add fixed amount, to each bin we step into, sample from that particle


	vector<Particle> resampled_particles;
	//first bin border
	double c = this->particles[0].weight;
	double u = 0;

	assert(this->particles.size() >= 0);


	//to make this foolproof, even if we dont use a probabilistic totally sound model
	double step_size = sum_weights()/(double) this->particles.size();

	//TODO: might want to remove this
	assert(step_size >= 0);

	//fixed offset to reach the next bin
	double offset = ((double)rand() / (double)RAND_MAX) * step_size;

	//TODO: might want to remove that
	assert(offset > 0);

	//DEBUG:
	int normal_add = 0;
	int other_add = 0;



	int j = 0;
	for(int i = 1; i <= this->particles.size(); i++){
		u =  offset + (i-1) * step_size;
		while(u >= c){
			j++;
			c += this->particles[j].weight;
		}
		//add particle and add uncertainty
		if(!this->augmented){
			Particle p(this->particles[j].x + random_gaussian() * this->resample_variance_pos, this->particles[j].y + random_gaussian() * this->resample_variance_pos,this->particles[j].rot + random_gaussian() * this->resample_variance_rot,1);
			resampled_particles.push_back(p);
		}
		if(this->augmented){
			//with max{0.0,1.0 - w_fast/w_slow add random sample
			double rand_val = random_uniform();
			if(rand_val <=  (1.0 - this->w_fast/this->w_slow) ){

				Particle p;

				//if goalpost
				if(vf.type == goal_post){
					//decide which goal post:
					int goal = rand() % 4;
					vector<LandMark> lm;
					lm = this->feature_map.get_features(goal_post);
					p = this->sample_landmark_model(lm[goal],vf);
				}
				//if x crossing
				if(vf.type == x_crossing){
					int x = rand() % 3;
					vector<LandMark> lm;
					lm = this->feature_map.get_features(x_crossing);
					p = this->sample_landmark_model(lm[x],vf);
				}
				//if t crossing
				if(vf.type == t_crossing){
					int t = rand() % 6;
					vector<LandMark> lm;
					lm = this->feature_map.get_features(t_crossing);
					p = this->sample_landmark_model(lm[t],vf);
				}
				//if l crossing
				if(vf.type == l_crossing){
					int l = rand() % 8;
					vector<LandMark> lm;
					lm = this->feature_map.get_features(l_crossing);
					p = this->sample_landmark_model(lm[l],vf);
				}

				other_add ++;
				resampled_particles.push_back(p);
			}else{
				normal_add ++;
				Particle p(this->particles[j].x + random_gaussian() * this->resample_variance_pos, this->particles[j].y + random_gaussian() * this->resample_variance_pos,this->particles[j].rot + random_gaussian() * this->resample_variance_rot,1);
				resampled_particles.push_back(p);
			}

		}
	}

	//save new particles
	this->particles = resampled_particles;

}
/*
 * resamples all particles with low variance resampling algorithm. (creates new particle list)
 * if no visual feature is observed
 */
int ParticleFilter::resample() {

	////low variance resampling:

	//draw random number
	// add fixed amount, to each bin we step into, sample from that particle


	vector<Particle> resampled_particles;

	//first bin border
	double c = this->particles[0].weight;
	double u = 0;

	assert(this->particles.size() >= 0);


	//to make this foolproof, even if we dont use a probabilistic totally sound model
	double step_size = sum_weights()/(double) this->particles.size();

	//TODO: might want to remove this
	assert(step_size >= 0);

	//fixed offset to reach the next bin
	double offset = ((double)rand() / (double)RAND_MAX) * step_size;

	//TODO: might want to remove that
	assert(offset > 0);


	int j = 0;
	for(int i = 1; i <= this->particles.size(); i++){
		u =  offset + (i-1) * step_size;
		while(u >= c){
			j++;
			c += this->particles[j].weight;
		}
		//add particle and add uncertainty
		if(!this->augmented){
			Particle p(this->particles[j].x + random_gaussian() * this->resample_variance_pos, this->particles[j].y + random_gaussian() * this->resample_variance_pos,this->particles[j].rot + random_gaussian() * this->resample_variance_rot,1);
			resampled_particles.push_back(p);
		}
		if(this->augmented){
			//with max{0.0,1.0 - w_fast/w_slow add random sample
			double rand = random_uniform();
			if(rand <=  (1.0 - this->w_fast/this->w_slow) ){
				double x = random_uniform() * this->x_dim - this->x_dim/(double)2;
				double y = random_uniform() * this->y_dim - this->y_dim/(double)2;
				double rot = random_uniform() * 2* PI - PI;

				Particle p(x,y,rot,1);
				resampled_particles.push_back(p);
			}else{
				Particle p(this->particles[j].x + random_gaussian() * this->resample_variance_pos, this->particles[j].y + random_gaussian() * this->resample_variance_pos,this->particles[j].rot + random_gaussian() * this->resample_variance_rot,1);
				resampled_particles.push_back(p);
			}

		}
	}

	//save new particles
	this->particles = resampled_particles;

}
int ParticleFilter::dynamic(OdometryInformation odo_inf,vector<VisualFeature> vis_feats){
	//variable for augmented MCL
	double w_avrg = 0;
	//get odometry information
	for(int i = 0; i < this->particles.size(); i++)
	{
		sample_motion_model_simple(odo_inf,&this->particles[i]);
		this->particles[i].weight = this->measurement_model(vis_feats,&this->particles[i]);

		//augmented MCL
		w_avrg += 1/this->particles.size() + this->particles[i].weight;
	}
	//augmented MCL:
	this->w_slow = this->w_slow + this->alpha_slow * (w_avrg - this->w_slow);
	this->w_fast = this->w_fast + this->alpha_fast * (w_avrg - this->w_fast );

}

/*
 * simple method to incorporate new odometry data
 */
int ParticleFilter::sample_motion_model_simple(OdometryInformation odometry_information,Particle* last_pose)
{
	//rotate odometry_vector, according to last pose (transform to local coordinate system):
	double x  = odometry_information.x * cos(last_pose->rot) - odometry_information.y * sin(last_pose->rot);
	double y  = odometry_information.x * sin(last_pose->rot) + odometry_information.y * cos(last_pose->rot);

	//add noise to vector
	double error_x =  this->random_gaussian() * this->error_range * x;
	double error_y = this->random_gaussian() * this->error_range * y  ;
	double error_rot =  this->random_gaussian() * odometry_information.rot  * this->error_bearing;

	last_pose->x =last_pose->x +  x + error_x;
	last_pose->y = last_pose->y + y + error_y;
	last_pose->rot = last_pose->rot + odometry_information.rot + error_rot;


	//if set outside field, put it in at random again.
	if(last_pose->x < -370 || last_pose->y > 370){

		last_pose->x = random_uniform() * this->x_dim - (double)this->x_dim/(double)2;
		last_pose->y = random_uniform() * this->y_dim - (double)this->y_dim/(double)2;
	}
	if(last_pose->y < -270 || last_pose->y > 270){
		last_pose->x = random_uniform() * this->x_dim - (double)this->x_dim/(double)2;
		last_pose->y = random_uniform() * this->y_dim - (double)this->y_dim/(double)2;
	}

	//TODO: reject particles outside the field !!
}
LandMark ParticleFilter::find_landmark(VisualFeature feature,
		Particle* current_pose, double* dist) {
	//iterate through all features of that type, calculate distance to feature observed in map

	//TODO: Include mahalanobis distance (takes into account that measurement error in different dimensions of the data is different) i.e. range and bearing.
	//TODO: If does not work well enough, ensure, that for two observation not the same landmark is chosen !
	//TODO: Include likelihood of observing other landmarks, that contribute enough to the likelihood

	vector<LandMark> lms = this->feature_map.get_features(feature.type);


	LandMark lm_on_map;

	//calculate position on map(assuming we are in the current pose):
	double x_feat = current_pose->x
			+ cos(feature.bearing + current_pose->rot) * feature.range;
	double y_feat = current_pose->y
			+ sin(feature.bearing + current_pose->rot) * feature.range;

	//choose feature with smallest distance and then proceed
	double min_dist = 100000; //10 meter
	for (int i = 0; i < lms.size(); i++) {
		//calculate distance
		double delta_x = x_feat - lms[i].pos.x;
		double delta_y = y_feat - lms[i].pos.y;
		double dist = sqrt(delta_x * delta_x + delta_y * delta_y);
		if (dist < min_dist) {
			//cout<<"found min dist for "<<i<<endl;
			min_dist = dist;
			lm_on_map = lms[i];
			if(lm_on_map.type >3){
				cout<<"PENISPENIS"<<lm_on_map.type<<" | "<<lms[i].type<<endl;
			}
		}
	}
	if(min_dist ==100000){
		cout<<"DID NOT FIND MIN DISTANCE !!"<<endl;
	}
	if(lm_on_map.type >3){
		cout<<"PENISPENISPENIS"<<lm_on_map.type<<"|"<<endl;
	}
	*dist = min_dist;
	return (lm_on_map);
}


/*
 * receives all visual features seen on the map and computes likelihood of a landmark measurment (with known correspondence)
 */
double ParticleFilter::measurement_model(vector<VisualFeature> features,
		Particle* current_pose) {
	//TODO if this does not work, change !!
	//assume the nearest landmark is the one observed (ML approach)

	//TODO: think about interpreting several landmarks aggregated to 1, (e.g. if there is one goal post, calculate the expected position of the other)
	// lfet right corners, can be described as aggregation of L and T crossings

	//distances to observed landmarks to landmarks on the map
	vector<double> dists;

	//index, landmark type, landmark index
	vector < vector<int> > landmarks;

	//pointer to landmark
	vector<LandMark> lms;

	//for each feature found, try to map it with features on the map
	for (int i = 0; i < features.size(); i++) {
		double dist = 0;
		LandMark landmark = find_landmark(features[i], current_pose,
				&dist);
		assert(landmark.type < 4);
		lms.push_back(landmark);

		dists.push_back(dist);

	}


	//thrun et al "Alorithm landmark_mode_known_correspondence(fi,ci,x,m) pp 179:
	//for every assigned landmark (no unassigned landmarks possible at the moment)
	vector<double> q;

	for(int i = 0; i < dists.size(); i++){
		double ra = -1;
		double phi = -1;
		double res = -1;
		////switch(landmarks[i][0]){
		//cout<<"LMPONTER:"<<lms[i].type<<endl;
		switch(lms[i].type){
		case l_crossing: //L crossing
		{
			double delta_x = lms[i].pos.x - current_pose->x;
			double delta_y = lms[i].pos.y - current_pose->y;

			//range
			ra = sqrt( delta_x * delta_x + delta_y * delta_y );//landmarks[i][2];
			//bearing
			phi = atan2(abs(lms[i].pos.y) - abs(current_pose->y ),abs(lms[i].pos.x )  -  abs(current_pose->x ));

			//how good is our estimate?
			res = this->measurement_factor * prob_gaussian(abs(features[i].range) - abs(ra),this->variance_range) * prob_gaussian(abs(features[i].bearing)  - abs(phi),this->variance_bearing) * prob_gaussian(0,1); //prob(si-sj,sigma_s) is last probability, how likeli ist that feature that landmark?

			break;
		}
		case t_crossing: //T crossings
		{
			double delta_x = lms[i].pos.x - current_pose->x;
			double delta_y = lms[i].pos.y - current_pose->y;

			//range
			ra = sqrt( delta_x * delta_x + delta_y * delta_y );//landmarks[i][2];
			//bearing
			phi = atan2(abs(lms[i].pos.y) - abs(current_pose->y ),abs(lms[i].pos.x )  -  abs(current_pose->x ));

			//how good is our estimate?
			res = this->measurement_factor * prob_gaussian(abs(features[i].range) - abs(ra),this->variance_range) * prob_gaussian(abs(features[i].bearing)  - abs(phi),this->variance_bearing) * prob_gaussian(0,1); //prob(si-sj,sigma_s) is last probability, how likeli ist that feature that landmark?
			break;
		}
		case x_crossing: //X crossings
		{
			double delta_x = lms[i].pos.x - current_pose->x;
			double delta_y = lms[i].pos.y - current_pose->y;
			//range
			ra = sqrt( delta_x * delta_x + delta_y * delta_y );//landmarks[i][2];
			//bearing
			phi = atan2(abs(lms[i].pos.y) - abs(current_pose->y ),abs(lms[i].pos.x )  -  abs(current_pose->x ));
			//how good is our estimate?
			res = this->measurement_factor * prob_gaussian(abs(features[i].range) - abs(ra),this->variance_range) * prob_gaussian(abs(features[i].bearing)  - abs(phi),this->variance_bearing) * prob_gaussian(0,1); //prob(si-sj,sigma_s) is last probability, how likeli ist that feature that landmark?

			break;
		}
		case goal_post: //Goal Post crossings
		{
			double delta_x = lms[i].pos.x - current_pose->x;
			double delta_y = lms[i].pos.y - current_pose->y;
			//range
			ra = sqrt( delta_x * delta_x + delta_y * delta_y );//landmarks[i][2];
			//bearing
			phi = atan2(abs(lms[i].pos.y) - abs(current_pose->y ),abs(lms[i].pos.x )  -  abs(current_pose->x ));
			//how good is our estimate?
			res = this->measurement_factor * prob_gaussian(abs(features[i].range) - abs(ra),this->variance_range) * prob_gaussian(abs(features[i].bearing)  - abs(phi),this->variance_bearing) * prob_gaussian(0,1); //prob(si-sj,sigma_s) is last probability, how likeli ist that feature that landmark?
			break;
		}
		default:
			cout<<"something went wrong in measurement model!"<<endl;
			break;
		}
		//we dont need smaller numbers
		if(res < 0.0000000001){ //if really small, just set to small value TODO: is there something wrong ?
			res = 0.0000000001;
		}
		//save result
		q.push_back(res);
	}

	//combining results for measurement via product
	double prod = 1;
	for (int i = 0; i < q.size(); i++){
		//if(q[i] == 0){q[i] = 0.0001;}
		prod = prod * q[i];
	}
	return(prod);
}
/*
 * adds single particle with specified parameters.
 */
int ParticleFilter::add_particle(double x, double y, double rot, double weight){
	Particle p;
	p.x = x;
	p.y = y;
	p.rot = rot;
	p.weight = weight;
	this->particles.push_back(p);
}
/*
 * creates a number of particles with fixed poses:
 */
int ParticleFilter::add_particles(int number,double x, double y, double rot, double weight){

	//no comment.
	for (int i = 0; i< number; i++){
		Particle p;
		p.x = x;
		p.y = y;
		p.rot = rot;
		p.weight = weight;
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
		double rot = (rand()%(int)(2*PI*accuracy))/accuracy-PI;
		double weight = 1;
		Particle p(x,y,rot,weight);
		this->particles.push_back(p);
	}
}
/*
 * gets the sum of the weigths, by iterating over the particle vector
 */
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

	//weighted average
	for(int i = 0; i < this->particles.size() ;  i++){
		double weight = particles[i].weight;
		sum += weight;
		x += particles[i].x * weight;
		y += particles[i].y * weight;
		rot += particles[i].rot * weight;
	}
	//save results
	*rot_est = rot / sum;
	*x_est = x / sum;
	*y_est = y / sum;
}

int ParticleFilter::kmeans(LocationVisualizer* lv,double* x1,double* y1, double* x2, double* y2){
	//chek for two clusters

	//take random particles:
	int rand_part_1 = floor(random_uniform() * this->particles.size());
	int rand_part_2 = floor(random_uniform() * this->particles.size());

	vector<Particle> cluster1;
	vector<Particle> cluster2;

	double mean1_x = 1;//this->particles[rand_part_1].x;
	double mean1_y = 0;//this->particles[rand_part_1].y;

	double mean2_x = -1;//this->particles[rand_part_2].x;
	double mean2_y = 0;//this->particles[rand_part_2].y;

	double mean1_x_old = 0;
	double mean1_y_old = 0;
	double mean2_x_old = 0;
	double mean2_y_old = 0;



	double dist_clusters = 1000;



	double dist_means1 = 100;
	double dist_means2 = 100;
	double delta_x = 100;
	double delta_y = 100;
	lv->draw_particle(20,mean1_x,mean1_y,0,0,255);
	lv->draw_particle(20,mean2_x,mean2_y,0,0,255);
	lv->draw_particle(this->particles);

	lv->create_window();
	lv->clear_buffer();

	int iter_count = 0;
	while (dist_means1 > 1 || dist_means2 > 1) {


		iter_count++;
		if (iter_count > 1000) {
			break;
		}
		if (iter_count % 10 == 0 ||iter_count == 1) {
			cout << dist_means1 << "|" << dist_means2 << endl;
			cout << iter_count << " : " << mean1_x_old << "," << mean1_y_old
					<< " " << mean1_x << "," << mean1_y << "|" << mean2_x_old
					<< "," << mean2_y_old << " " << mean2_x << "," << mean2_y
					<< endl;
			cout<<"no of particles 1 : "<<cluster1.size()<<"| 2 : "<<cluster2.size()<<endl;
			lv->draw_particle(20,mean1_x,mean1_y,0,0,255);
			lv->draw_particle(20,mean2_x,mean2_y,0,0,255);
			lv->draw_particle(this->particles);

			lv->create_window();
			lv->clear_buffer();
		}
		//assignemnt: assign each observation to the cluster whose mean is closest to it
		//calculate distance to mean 1
		//calculate distance to mean 2
		//assign to point which is closest

		mean1_x_old = mean1_x;
		mean1_y_old = mean1_y;
		mean2_x_old = mean2_x;
		mean2_y_old = mean2_y;
		vector<Particle> empty_vec1;
		vector<Particle> empty_vec2;

		cluster1 = empty_vec1;
		cluster2 = empty_vec2;

		for (int i = 0; i < this->particles.size(); i++) {
			double delta_x = mean1_x - this->particles[i].x;
			double delta_y = mean1_y - this->particles[i].y;

			double dist_1 = sqrt(delta_x * delta_x + delta_y * delta_y);

			delta_x = mean2_x - this->particles[i].x;
			delta_y = mean2_y - this->particles[i].y;
			double dist_2 = sqrt(delta_x * delta_x + delta_y * delta_y);

			//decide to which cluster to assign
			if (dist_2 >= dist_1) {
				//assign to cluster 1
				cluster1.push_back(this->particles[i]);
			} else {
				//assign to cluster 2
				cluster2.push_back(this->particles[i]);
			}
		}

		//update step: calculate the new means to be the centroid of the observations in the new cluster
		//iterate through every particle in cluster, take avrg x and y
		mean1_x = 0;
		mean1_y = 0;
		for (int i = 0; i < cluster1.size(); i++) {
			mean1_x += cluster1[i].x;
			mean1_y += cluster1[i].y;
		}
		mean1_x = mean1_x / (double) cluster1.size();
		mean1_y = mean1_y / (double) cluster1.size();

		mean2_x = 0;
		mean2_y = 0;
		for (int i = 0; i < cluster2.size(); i++) {
			mean2_x += cluster2[i].x ;
			mean2_y += cluster2[i].y ;
		}
		mean2_x = mean2_x / (double) cluster2.size();
		mean2_y = mean2_y / (double) cluster2.size();

		//distance of both clusters
		double delta_x_clusters = mean1_x - mean2_x;
		double delta_y_clusters = mean1_y - mean2_y;
		dist_clusters = sqrt(
				delta_x_clusters * delta_x_clusters
						+ delta_y_clusters * delta_y_clusters);


		//change of means in euclidean distance cluster 1
		delta_x = mean1_x_old - mean1_x;
		delta_y = mean1_y_old - mean1_y;

		dist_means1 = sqrt(delta_x * delta_x + delta_y * delta_y);

		//change of means in euclidean distance cluster 2
		delta_x = mean2_x_old - mean2_x;
		delta_y = mean2_y_old - mean2_y;
		dist_means2 = sqrt(delta_x * delta_x + delta_y * delta_y);
		/*if(mean2_x_old == mean2_x){cout << iter_count << " : " << mean1_x_old << "," << mean1_y_old
				<< " " << mean1_x << "," << mean1_y << "|" << mean2_x_old
				<< "," << mean2_y_old << " " << mean2_x << "," << mean2_y
				<< endl;}*/
		cout<<dist_means2<<endl;
	}
	*x1 = mean1_x;
	*y1 = mean1_y;
	*x2 = mean2_x;
	*y2 = mean2_y;
	lv->draw_particle(20,mean1_x,mean1_y,0,0,255);
	lv->draw_particle(20,mean2_x,mean2_y,0,0,255);
	lv->draw_particle(this->particles);

	lv->create_window();
	lv->clear_buffer();
}
