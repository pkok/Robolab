/*
 * InputGenerator.cpp
 *
 *  Created on: Jan 19, 2013
 *      Author: owner
 */



#include "InputGenerator.h"

#include <cmath>
#include <assert.h>

using namespace std;


double InputGenerator::random_uniform ()
{
  return ((double) rand())/RAND_MAX;
}
/*
 * box muller transform returns sample from gaussian distribution
 */
double InputGenerator::random_gaussian ()
{
  double u = random_uniform(), v = random_uniform();
  return sqrt(-2*log(u))*cos(2*M_PI*v);
}


InputGenerator::InputGenerator() {
	// TODO Auto-generated constructor stub

}

InputGenerator::~InputGenerator() {
	// TODO Auto-generated destructor stub
}
/*
 * generates new odometry model
 */
int InputGenerator::generate_odometry(double x,double y, double rot, OdometryInformation* odo_inf){

	odo_inf->rot = rot + random_gaussian()/5;
	odo_inf->x = x + random_gaussian();
	odo_inf->y = y + random_gaussian(); //random value
	//cout<<"IG odo_inf"<<odo_inf->x<<" "<<odo_inf->y<<" "<<odo_inf->rot<<endl;
}
/*
 * generates a set of new features
 */
int InputGenerator::generate_features(double x,double y, double rot,FeatureMap fm, vector<VisualFeature>* vis_feat){
	//get range and bearing of all landmarks according to our current position:

	vector<VisualFeature> vis_feat_all;

	calculate_range_bearing(fm,x,y,rot,&vis_feat_all);

	//pick random head rotation (from 239°/2 .. -239°/2 )

	//double head_rot =  random_uniform() * HEAD_ROT_MAX - HEAD_ROT_MAX_2;
	double head_rot = rot;
	//camera angle horizontal from ( 61°/2 .. - 61°/2 )
	double camera_right = head_rot - CAMERA_ROT_MAX_2;
	double camera_left = head_rot + CAMERA_ROT_MAX_2;

	//discard features that are not vissible
	for(int i = 0; i< vis_feat_all.size(); i++){
		//add random noise
		vis_feat_all[i].bearing +=random_gaussian() * 0.1;
		vis_feat_all[i].range += random_gaussian() * 2;
		assert(vis_feat_all[i].range < 10000);

		if((  vis_feat_all[i].bearing > camera_right && vis_feat_all[i].bearing < camera_left  )){
			vis_feat->push_back(vis_feat_all[i]);
		}
	}


	return(0);
}



// convert feature from map to robot space

int InputGenerator::calculate_range_bearing(FeatureMap fm,double x, double y, double rot, vector<VisualFeature>* poss_feat){

	//iterate l_cross
	vector<LandMark> lm_l;
	lm_l = fm.get_features(l_crossing);
	for(int i = 0; i<lm_l.size() ; i++){

		double delta_x =  lm_l[i].pos.x -x ;
		double delta_y = lm_l[i].pos.y - y ;

		//position of unity vector with rotation:
		double x_unity =cos(rot);
		double y_unity = sin(rot);


		double r = sqrt((x - lm_l[i].pos.x) * (x - lm_l[i].pos.x) + (y - lm_l[i].pos.y) * (y - lm_l[i].pos.y));

		//check if "behind" robot:
		//project vector:
		double proj = (delta_x * x_unity + delta_y * y_unity) / (r * r);
		double proj_x =  delta_x * proj;
		double proj_y = delta_y * proj;

		double bear = acos((x_unity * delta_x + y_unity * delta_y) / (1 * r));

		VisualFeature f;
		f.bearing = bear;
		f.range =r;
		f.type = l_crossing ;
		poss_feat->push_back(f);
	}
	//iterate t_cross
	vector<LandMark> lm_t;
	lm_t = fm.get_features(t_crossing);
	for(int i = 0; i<lm_t.size() ; i++){
		double delta_x = lm_t[i].pos.x - x ;
		double delta_y = lm_t[i].pos.y - y;

		//position of unity vector with rotation:
		double x_unity = cos(rot);
		double y_unity = sin(rot);


		double r = sqrt((x - lm_t[i].pos.x) * (x - lm_t[i].pos.x) + (y - lm_t[i].pos.y) * (y - lm_t[i].pos.y));



		//check if "behind" robot:
		//project vector:
		double proj = (delta_x * x_unity + delta_y * y_unity) ;
		double proj_x =  delta_x * proj;
		double proj_y = delta_y * proj;

		double bear = acos((x_unity * delta_x + y_unity * delta_y) / (1 * r));
		VisualFeature f;
		f.bearing = bear;
		f.range =r;
		f.type = t_crossing ;
		poss_feat->push_back(f);
	}
	//iterate x_cross
	vector<LandMark> lm_x;
	lm_x = fm.get_features(x_crossing);
	for(int i = 0; i<lm_x.size() ; i++){
		double delta_x =  lm_x[i].pos.x -x ;
		double delta_y = lm_x[i].pos.y  -y;

		//position of unity vector with rotation:
		double x_unity = cos(rot);
		double y_unity = sin(rot);
		double r = sqrt((x - lm_x[i].pos.x) * (x - lm_x[i].pos.x) + (y - lm_x[i].pos.y) * (y - lm_x[i].pos.y));

		//check if "behind" robot:
		//project vector:
		double proj = (delta_x * x_unity + delta_y * y_unity) ;
		double proj_x =  delta_x * proj;
		double proj_y = delta_y * proj;

		double bear = acos((x_unity * delta_x + y_unity * delta_y) / (1 * r));

		VisualFeature f;
		f.bearing = bear;
		f.range =r;
		f.type = x_crossing ;
		poss_feat->push_back(f);
	}
	//iterate g_cross
	vector<LandMark> lm_g;
	lm_g = fm.get_features(goal_post);
	for(int i = 0; i<4 ; i++){
		double delta_x =  lm_g[i].pos.x - x ;
		double delta_y =   lm_g[i].pos.y - y;

		//position of unity vector with rotation:
		double x_unity = cos(rot);
		double y_unity = sin(rot);


		double r = sqrt((x - lm_g[i].pos.x) * (x - lm_g[i].pos.x) + (y - lm_g[i].pos.y) * (y - lm_g[i].pos.y));



		//check if "behind" robot:
		//project vector:
		double proj = (delta_x * x_unity + delta_y * y_unity) ;
		double proj_x =  delta_x * proj;
		double proj_y = delta_y * proj;




		double bear = acos(proj/(1*r));//(x_unity * delta_x + y_unity * delta_y) / (1 * r);
		//cout<<"goal origin:"<<i<<" "<<delta_x<<"|"<<delta_y<<endl;
		//cout<<"goal:"<<i<<" "<<proj_x<<" "<<x_unity<<"|"<<proj_y<<" "<<y_unity<<endl;

		VisualFeature f;
		f.bearing = bear;
		f.range =r;
		f.type = goal_post ;
		poss_feat->push_back(f);
	}

}
