/*
 * FeatureMap.cpp
 *
 *  Created on: Jan 14, 2013
 *      Author: owner
 */

#include <vector>
#include <assert.h>
#include <iostream>
#include "FeatureMap.h"

using namespace std;

LandMark::LandMark(double x, double y, FeatureType ft){
	Pos pos(x,y);
	this->pos = pos;
	this->type = ft;
}
LandMark::LandMark(){

}


FeatureMap::FeatureMap(){
		//add landmark to vector: 8

	this->land_marks.push_back(LandMark(300, 200, l_crossing));
	this->land_marks.push_back(LandMark(300, -200, l_crossing));
	this->land_marks.push_back(LandMark(240, 110, l_crossing));
	this->land_marks.push_back(LandMark(240, -110, l_crossing));
	this->land_marks.push_back(LandMark(-240, 110, l_crossing));
	this->land_marks.push_back(LandMark(-240, -110, l_crossing));
	this->land_marks.push_back(LandMark(-300, 200, l_crossing));
	this->land_marks.push_back(LandMark(-300, -200, l_crossing));

	//6
	this->land_marks.push_back(LandMark(300,110, t_crossing));
	this->land_marks.push_back(LandMark(300,-110, t_crossing));
	this->land_marks.push_back(LandMark(0,200, t_crossing));
	this->land_marks.push_back(LandMark(0,-200, t_crossing));
	this->land_marks.push_back(LandMark(-300,110, t_crossing));
	this->land_marks.push_back(LandMark(-300,-110, t_crossing));

	//5
	//this->land_marks.push_back(LandMark(120, 0, x_crossing));
	this->land_marks.push_back(LandMark(0, 60, x_crossing));
	this->land_marks.push_back(LandMark(0, 0, x_crossing));
	this->land_marks.push_back(LandMark(0, -60, x_crossing));
	//this->land_marks.push_back(LandMark(-120,0, x_crossing));

	//4
	this->land_marks.push_back(LandMark(300,70, goal_post));
	this->land_marks.push_back(LandMark(300,-70, goal_post));
	this->land_marks.push_back(LandMark(-300,70, goal_post));
	this->land_marks.push_back(LandMark(-300,-70, goal_post));


	/* OLD FEATURE MAP
	this->land_marks.push_back(LandMark(300, 200, l_crossing));
	this->land_marks.push_back(LandMark(300, -200, l_crossing));
	this->land_marks.push_back(LandMark(240, 110, l_crossing));
	this->land_marks.push_back(LandMark(240, -110, l_crossing));
	this->land_marks.push_back(LandMark(-240, 110, l_crossing));
	this->land_marks.push_back(LandMark(-240, -110, l_crossing));
	this->land_marks.push_back(LandMark(-300, 200, l_crossing));
	this->land_marks.push_back(LandMark(-300, -200, l_crossing));

	//6
	this->land_marks.push_back(LandMark(300,110, t_crossing));
	this->land_marks.push_back(LandMark(300,-110, t_crossing));
	this->land_marks.push_back(LandMark(0,200, t_crossing));
	this->land_marks.push_back(LandMark(0,-200, t_crossing));
	this->land_marks.push_back(LandMark(-300,110, t_crossing));
	this->land_marks.push_back(LandMark(-300,-110, t_crossing));

	//5
	this->land_marks.push_back(LandMark(120, 0, x_crossing));
	this->land_marks.push_back(LandMark(0, 60, x_crossing));
	this->land_marks.push_back(LandMark(0, 0, x_crossing));
	this->land_marks.push_back(LandMark(0, -60, x_crossing));
	this->land_marks.push_back(LandMark(-120,0, x_crossing));

	//4
	this->land_marks.push_back(LandMark(300,70, goal_post));
	this->land_marks.push_back(LandMark(300,-70, goal_post));
	this->land_marks.push_back(LandMark(-300,70, goal_post));
	this->land_marks.push_back(LandMark(-300,-70, goal_post));*/

/*
		//chance to observe this object 1/5
		this->x_cross[0]= Pos(120,0);
		this->x_cross[1] = Pos(0,60);
		this->x_cross[2] = Pos(0,0);
		this->x_cross[3] = Pos(0,-60);
		this->x_cross[4] = Pos(-120,0);

		this->l_cross[0] = Pos(300,200);
		this->l_cross[1] = Pos(300,-200);
		this->l_cross[2] = Pos(240,110);
		this->l_cross[3] = Pos(240,-110);
		this->l_cross[4] = Pos(-240,110);
		this->l_cross[5] = Pos(-240,-110);
		this->l_cross[6] = Pos(-300,200);
		this->l_cross[7] = Pos(-300,-200);

		this->t_cross[0] = Pos(300,110);
		this->t_cross[1] = Pos(300,-110);
		this->t_cross[2] = Pos(0,200);
		this->t_cross[3] = Pos(0,-200);
		this->t_cross[4] = Pos(-300,110);
		this->t_cross[5] = Pos(-300,-110);


		this->g_cross[0] = Pos(300,70);
		this->g_cross[1] = Pos(300,-70);
		this->g_cross[2] = Pos(-300,70);
		this->g_cross[3] = Pos(-300,-70);*/
	}

/*
 * searches vector of landmarks for suitable type, returns a pointer, to every landmark found in landmark vector.
 */
vector<LandMark> FeatureMap::get_features(FeatureType ft){
	//TODO:can be made faster:
	vector<LandMark> ret;
	for (int i = 0; i < this->land_marks.size();i++){
		if(land_marks[i].type == ft){
			LandMark lm = land_marks[i];
			ret.push_back(lm);
			if(lm.type > 4){
				cout<<"fuck you get_features!"<<endl;
			}
		assert(lm.type < 4);
		}
	}

	return(ret);
}

/*
Pos::Pos(int x, int y){

}*/
/*
FeatureMap::FeatureMap(){
		int penis = 0;

}*/
