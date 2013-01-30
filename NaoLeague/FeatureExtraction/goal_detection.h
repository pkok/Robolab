#ifndef GOAL_DETECTION_H
#define GOAL_DETECTION_H

#include <iostream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>

#define R_POST 0
#define L_POST 1
#define V_POST 2
#define O_POST 3

#define HIST_THRESHOLD 0.3
#define ROOT_GAIN 1.5
#define CONTROL_MAX 0.2
#define CROP_THRESHOLD 5
#define SAMPLING_VER 5
#define SAMPLING_HOR 5
#define ROOT_OFFSET_Y 10

using namespace cv;

struct goalposts{
	int type;
	Vec4i line;
	Point root_position;
	Point top_position;
	double width;
	double root_confidence;
};

struct posts_lines{
	int candidate;
	Vec4i line;
};

void goalPostDetection(Mat image, vector<Point> goalRoots, double* hor_hist, int* ver_hist, vector<goalposts> &goalPosts);

#endif