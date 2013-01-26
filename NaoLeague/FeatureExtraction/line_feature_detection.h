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

using namespace cv;

struct Orient{
	double angle1;
	double angle2;
};

struct field_point{
	int type;
	Point position;
	Orient orientation;
	double confidence;
};

void line_features(Mat image, vector<Vec4i> lines);

#endif
