#ifndef GOAL_DETECTION_H
#define GOAL_DETECTION_H

#include <iostream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>

#define R_POST 0
#define L_POST 0
#define O_POST 0

using namespace cv;

struct goalposts{
	int type;
	Point root_position;
	Point top_position;
	double width;
};

void goalPostDetection(Mat yellow, vector<Point> goalRoots, int* hor_hist);

#endif