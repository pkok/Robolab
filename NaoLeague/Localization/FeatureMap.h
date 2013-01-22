/*
 * FeatureMap.h
 *
 *  Created on: Jan 14, 2013
 *      Author: owner
 */
#ifndef FEATUREMAP_H_
#define FEATUREMAP_H_

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


//map of feature positions
struct FeatureMap{



	//0,0 upper left corner,

	Pos l_cross[8];

	Pos t_cross[6];

	Pos x_cross[5];

	Pos g_cross[4]; //goal posts
	FeatureMap(){
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
		this->g_cross[3] = Pos(-300,-70);
	}

};

#endif


