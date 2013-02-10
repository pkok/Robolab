#ifndef DIS_ANG_TRANSLATION_H
#define DIS_ANG_TRANSLATION_H

#include <iostream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>

#define HOR_CAMERA_FIELD 61.0
#define VER_CAMERA_FIELD 47.0
#define CAMERA_HEIGHT 0.6

using namespace cv;

/*
For every image taken from nao's camera, we have these 4 angles
to use...
I leave it for now until it works
[8]: left angle (radian).
[9]: topAngle (radian).
[10]: rightAngle (radian).
[11]: bottomAngle (radian).
Camera height is also given.
*/

struct dis_bear{
	double distance;
	double bearing;
};

dis_bear pixel2dis_bear(Point2f pixel);

Point2f normalizePixelPosition(Mat image, Point pixel);

#endif