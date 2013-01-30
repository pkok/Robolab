#ifndef LINE_FEATURE_DETECTION_H
#define LINE_FEATURE_DETECTION_H

#include <math.h>
#include <cv.h>
#include <highgui.h>

//line feature types
#define L_CROSS 0
#define T_CROSS 1
#define X_CROSS 3
#define UNKNOWN 4

#define L_THRESHOLD 0.15
#define T_THRESHOLD 0.15
#define X_THRESHOLD 0.15

#define DIFF_THRESHOLD_L 0.3
#define DIFF_THRESHOLD_T 0.3
#define DIFF_THRESHOLD_X 0.3

#define TRANS_2T_X_THRESHOLD 0.7
#define DISTANCE_2T_X 8

using namespace cv;

struct field_feature{
	double orientation[2];
	double confidence;
};

struct field_intersection{
	Point position;
	field_feature t;
	field_feature l;
	field_feature x;
	double min_pl_length;
};

struct field_point{
	int type;
	Point position;
	double orientation[2];
	double confidence;
};

void line_features(Mat image, vector<Vec4i> lines, vector<field_intersection> &result_intersections);

void line_most_prob_features(Mat image, vector<Vec4i> lines, vector<field_point> &result_intersections);

#endif
