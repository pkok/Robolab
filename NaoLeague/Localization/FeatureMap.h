/*
 * FeatureMap.h
 *
 *  Created on: Jan 14, 2013
 *      Author: owner
 */
#ifndef FEATUREMAP_H_
#define FEATUREMAP_H_

#include <vector>

using namespace std;

//position of each feature
struct Pos
{
	int x;
	int y;
	Pos(){
		this->x = 0;
		this->y = 0;
	}
	Pos(int x,int y){
		this->x = x;
		this->y = y;
	}
};

enum FeatureType {l_crossing,t_crossing,x_crossing, goal_post};

struct LandMark{
	Pos pos;
	FeatureType type;
	LandMark(double x, double y, FeatureType ft);
};
//map of feature positions
struct FeatureMap{

	vector<LandMark> land_marks;

	//0,0 upper left corner,
/*
	Pos l_cross[8];

	Pos t_cross[6];

	Pos x_cross[5];

	Pos g_cross[4]; //goal posts
*/
	//returns landmarks of only specified FeatureType
	vector<LandMark*> get_features(FeatureType ft);

	//constructor
	FeatureMap();

};

#endif


