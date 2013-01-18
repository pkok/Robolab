#ifndef BACKGROUND_H
#define BACKGROUND_H

#include <iostream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>

using namespace cv;
using namespace std;

bool hsv_range(Vec3b pixel, int h_min, int h_max, int s_min, int s_max, int v_min, int v_max);

void ass_val_pixel(Vec3b &pixel, int h, int s, int v);

void ass_val_pixel2pixel(Vec3b &src, Vec3b &dst);

void remove_background(Mat image, Mat &lines, Mat &posts, Mat &ball);

#endif
