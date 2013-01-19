/*
 * InputGenerator.h
 *
 *  Created on: Jan 19, 2013
 *      Author: owner
 */
#pragma once

#include <vector>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include "ParticleFilter.h"

#define PI 					3.14159

#define HEAD_ROT_MAX 		4.17133691
#define HEAD_ROT_MAX_2 		2.08566845

#define CAMERA_ROT_MAX 		1.06465084
#define CAMERA_ROT_MAX_2 	0.53232542

using namespace std;


class InputGenerator {
public:


	double random_uniform ();
	double random_gaussian ();


	int generate_odometry(double x,double y, double rot,OdometryInformation* odo_inf);
	int generate_features(double x,double y, double rot,FeatureMap fm, vector<VisualFeature>* vis_feat);
	int calculate_range_bearing(FeatureMap fm,double x, double y, double rot, vector<VisualFeature>* poss_feat);

	InputGenerator();
	virtual ~InputGenerator();
};

