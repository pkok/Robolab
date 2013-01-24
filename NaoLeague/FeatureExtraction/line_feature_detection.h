#ifndef LINE_FEATURE_DETECTION_H
#define LINE_FEATURE_DETECTION_H

#include <math.h>
#include <cv.h>
#include <highgui.h>

using namespace cv;

void line_features(Mat image, vector<Vec4i> lines);

#endif
