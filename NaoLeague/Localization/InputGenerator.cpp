/*
 * InputGenerator.cpp
 *
 *  Created on: Jan 19, 2013
 *      Author: owner
 */



#include "InputGenerator.h"

#include <cmath>

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
		if((  vis_feat_all[i].bearing > camera_right && vis_feat_all[i].bearing < camera_left  )){
			vis_feat->push_back(vis_feat_all[i]);
		}
	}


	return(0);
}



// convert feature from map to robot space

int InputGenerator::calculate_range_bearing(FeatureMap fm,double x, double y, double rot, vector<VisualFeature>* poss_feat){

	//iterate l_cross
	for(int i = 0; i<8 ; i++){

		double delta_x =  fm.l_cross[i].x -x ;
		double delta_y = fm.l_cross[i].y - y ;

		//position of unity vector with rotation:
		double x_unity =cos(rot);
		double y_unity = sin(rot);


		double r = sqrt((x - fm.l_cross[i].x) * (x - fm.l_cross[i].x) + (y - fm.l_cross[i].y) * (y - fm.l_cross[i].y));

		//check if "behind" robot:
		//project vector:
		double proj = (delta_x * x_unity + delta_y * y_unity) / (r * r);
		double proj_x =  delta_x * proj;
		double proj_y = delta_y * proj;

		double bear = acos((x_unity * delta_x + y_unity * delta_y) / (1 * r));

		VisualFeature f;
		f.bearing = bear;
		f.range =r;
		f.type = 0 ;
		poss_feat->push_back(f);
	}
	//iterate t_cross
	for(int i = 0; i<6 ; i++){
		double delta_x = fm.t_cross[i].x - x ;
		double delta_y = fm.t_cross[i].y - y;

		//position of unity vector with rotation:
		double x_unity = cos(rot);
		double y_unity = sin(rot);


		double r = sqrt((x - fm.t_cross[i].x) * (x - fm.t_cross[i].x) + (y - fm.t_cross[i].y) * (y - fm.t_cross[i].y));



		//check if "behind" robot:
		//project vector:
		double proj = (delta_x * x_unity + delta_y * y_unity) ;
		double proj_x =  delta_x * proj;
		double proj_y = delta_y * proj;

		double bear = acos((x_unity * delta_x + y_unity * delta_y) / (1 * r));
		VisualFeature f;
		f.bearing = bear;
		f.range =r;
		f.type = 1 ;
		poss_feat->push_back(f);
	}
	//iterate x_cross
	for(int i = 0; i<5 ; i++){
		double delta_x =  fm.x_cross[i].x -x ;
		double delta_y = fm.x_cross[i].y  -y;

		//position of unity vector with rotation:
		double x_unity = cos(rot);
		double y_unity = sin(rot);
		double r = sqrt((x - fm.x_cross[i].x) * (x - fm.x_cross[i].x) + (y - fm.x_cross[i].y) * (y - fm.x_cross[i].y));

		//check if "behind" robot:
		//project vector:
		double proj = (delta_x * x_unity + delta_y * y_unity) ;
		double proj_x =  delta_x * proj;
		double proj_y = delta_y * proj;

		double bear = acos((x_unity * delta_x + y_unity * delta_y) / (1 * r));

		VisualFeature f;
		f.bearing = bear;
		f.range =r;
		f.type = 2 ;
		poss_feat->push_back(f);
	}
	//iterate g_cross
	for(int i = 0; i<4 ; i++){
		double delta_x =  fm.g_cross[i].x - x ;
		double delta_y =   fm.g_cross[i].y - y;

		//position of unity vector with rotation:
		double x_unity = cos(rot);
		double y_unity = sin(rot);


		double r = sqrt((x - fm.g_cross[i].x) * (x - fm.g_cross[i].x) + (y - fm.g_cross[i].y) * (y - fm.g_cross[i].y));



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
		f.type = 3 ;
		poss_feat->push_back(f);
	}

}
