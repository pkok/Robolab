/*
 * LocationVisualizer.h
 *
 *  Created on: Jan 15, 2013
 *      Author: owner
 */

#ifndef LOCATIONVISUALIZER_H_
#define LOCATIONVISUALIZER_H_

#include <cv.h>
using namespace cv;

class LocationVisualizer{
public:
	//field image with only lines
	Mat field_img;
	//field image with particles
	Mat current_frame;

	//fieldsize
	int h;
	int w;

	//rectangles
	int outer_line[8];
	int goal_line_1[8];
	int goal_line_2[8];

	//lines
	int middle_line[4];

	//points
	int penalty1[3];
	int penalty2[3];

	int goalpost11[3];
	int goalpost12[3];
	int goalpost21[3];
	int goalpost22[3];

	int middle_circle[3];



	void draw_field();
	void draw_field_lines();
	void create_window();


	void draw_line( Mat img, Point start, Point end );
	void draw_rectangle(Mat img, Point one,Point two,Point three,Point four);
	void clear_buffer();
	void draw_particle(Point location, double rotation,double certainty);
	void read_file(int* h,int* w,int outer_line[],int goal_line1[],int goal_line2[], int middle_line[], int penalty1[],int penalty2[],int goalpost11[],int goalpost12[],int goalpost21[],int goalpost22[],int middle_circle[]);
	void convert_to_image_coordinates(int* x,int* y);
	void convert_all();

	LocationVisualizer();
	~LocationVisualizer();
};

#endif /* LOCATIONVISUALIZER_H_ */
