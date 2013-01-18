#ifndef HOUGH_H
#define HOUGH_H

#include <iostream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>

using namespace cv;

void probabilistic_hough_trans(Mat src, vector<Vec4i> &lines);

#endif
