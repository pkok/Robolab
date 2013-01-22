#ifndef LINES_H
#define LINES_H

#include <iostream>
#include <math.h>
#include <cv.h>
#include <highgui.h>

using namespace cv;

void line_clustering(Mat image);

double similarity_measure(Vec4i line1, Vec4i line2);

double similarity_measure(Vec4i line1, Vec4i line2);

#endif