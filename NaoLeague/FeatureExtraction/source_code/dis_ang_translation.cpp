#include <iostream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>
#include "dis_ang_translation.h"
#include "geometry_utils.h"

using namespace cv;
using namespace std;

Point2f normalizePixelPosition(Mat image, Point pixel)
{
	Point2f norm_pix;
	norm_pix.x = (double)pixel.x / image.rows;
	norm_pix.y = (double)pixel.y / image.cols;
	return norm_pix;
}

dis_bear pixel2dis_bear(Point2f pixel)
{
	dis_bear result;

	double hor_angle_camera;
	double ver_angle_camera;
	// these are the angles in respect to the camera
	hor_angle_camera = (pixel.y - 0.5) * HOR_CAMERA_FIELD;
	ver_angle_camera = - (pixel.x - 0.5) * VER_CAMERA_FIELD;

	// yaw and pitch angles, these values will be taken 
	// from nao normally.
	double head_yaw_angle = -20.0;
	double head_pitch_angle = 0.0;

	// global angles in respect to the body
	double global_hor_angle = hor_angle_camera + head_pitch_angle;
	double global_ver_angle = ver_angle_camera + head_yaw_angle;

	double angleOffset = abs(global_hor_angle - head_pitch_angle);

	// Assuming that points are only in the 2D field space
	double elevation = global_ver_angle;

	double y = CAMERA_HEIGHT / tan(radians(-elevation));
	double x = y * tan(radians(-global_hor_angle));
	result.distance = sqrt(x*x + y*y);
	result.bearing = global_hor_angle;
	return result;
}
