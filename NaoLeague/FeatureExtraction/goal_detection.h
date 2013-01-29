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

#define HIST_THRESHOLD 0.3
#define ROOT_GAIN 1.5
#define CONTROL_MAX 0.2
#define CROP_THRESHOLD 5
#define SAMPLING_VER 5
#define SAMPLING_HOR 5

using namespace cv;

struct goalposts{
	int type;
	Point root_position;
	double root_confidence;
	Point top_position;
	double width;
};

struct posts_lines{
	int candidate;
	Vec4i line;
};

void goalPostDetection(Mat yellow, vector<Point> goalRoots, double* hor_hist, int* ver_hist);

#endif