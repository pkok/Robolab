#ifndef LINE_FEATURE_DETECTION_H
#define LINE_FEATURE_DETECTION_H

#include <math.h>
#include <cv.h>
#include <highgui.h>

//line feature types
#define L_CROSS 0
#define T_CROSS 1
#define X_CROSS 3
#define ELLIPSE 4

#define L_THRESHOLD 0.5
#define T_THRESHOLD 0.5
#define X_THRESHOLD 0.5

#define DIFF_THRESHOLD 0.1

#define LOW_THRESHOLD 0.1

using namespace cv;

struct field_point{
	int type;
	Point position;
	double orientation[2];
	double confidence;
};

void line_features(Mat image, vector<Vec4i> lines);

#endif
