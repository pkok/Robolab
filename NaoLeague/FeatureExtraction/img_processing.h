#ifndef BACKGROUND_H
#define BACKGROUND_H

#include <iostream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>

#define BACK_THRESHOLD 1

#define YEL_HUE_MIN  20
#define YEL_HUE_MAX  38
#define YEL_SAT_MIN  100
#define YEL_SAT_MAX  255
#define YEL_VAL_MIN  100
#define YEL_VAL_MAX  255

#define GR_HUE_MIN  38
#define GR_HUE_MAX  75
#define GR_SAT_MIN  50
#define GR_SAT_MAX  255
#define GR_VAL_MIN  50
#define GR_VAL_MAX  255

#define WH_HUE_MIN  0
#define WH_HUE_MAX  255
#define WH_SAT_MIN  0
#define WH_SAT_MAX  60
#define WH_VAL_MIN  200
#define WH_VAL_MAX  255

using namespace cv;
using namespace std;

bool hsv_range(Vec3b pixel, int h_min, int h_max, int s_min, int s_max, int v_min, int v_max);

void ass_val_pixel(Vec3b &pixel, int h, int s, int v);

void ass_val_pixel2pixel(Vec3b &src, Vec3b &dst);

void remove_background(Mat image, Mat &lines, Mat &posts, Mat &ball);

double compute_white_ratio(Mat image, Point point1, Point point2);

#endif
