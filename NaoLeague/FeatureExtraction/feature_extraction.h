#ifndef FEATURE_EXTRACTION_H
#define FEATURE_EXTRACTION_H

#include <cv.h>
#include <highgui.h>
#include "img_processing.h"
#include "line_detection.h"
#include "line_feature_detection.h"
#include "ellipse_detector.h"
#include "goal_detection.h"

using namespace cv;

void extract_features(Mat img_rgb, vector<field_point> &result_intersections, vector<goalposts> &goalPosts);

#endif