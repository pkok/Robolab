/*
 * FeatureMap.h
 *
 *  Created on: Jan 14, 2013
 *      Author: owner
 */


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

	Pos t_cross[7];

	Pos x_cross[5];

	FeatureMap(){
		//chance to observe this object 1/5
		this->x_cross[0]= Pos(270,310);
		this->x_cross[1] = Pos(250,270);
		this->x_cross[2] = Pos(270,370);
		this->x_cross[3] = Pos(490,270);
		this->x_cross[4] = Pos(270,430);

		this->l_cross[0] = Pos(70,70);
		this->l_cross[1] = Pos(670,70);
		this->l_cross[2] = Pos(130,160);
		this->l_cross[3] = Pos(610,160);
		this->l_cross[4] = Pos(130,380);
		this->l_cross[5] = Pos(610,380);
		this->l_cross[6] = Pos(70,470);
		this->l_cross[7] = Pos(670,470);

		this->t_cross[0] = Pos(370,70);
		this->t_cross[1] = Pos(70,160);
		this->t_cross[2] = Pos(670,160);
		this->t_cross[3] = Pos(70,380);
		this->t_cross[4] = Pos(610,380);
		this->t_cross[5] = Pos(70,470);
		this->t_cross[6] = Pos(670,470);
	}

};


