#include <iostream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>
#include "img_processing.h"
#include "line_detection.h"
#include "line_feature_detection.h"
#include "ellipse_detector.h"
#include "goal_detection.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	Mat img_rgb, img_hsv;
	Mat img_lines_binary, img_posts_binary, img_ball_binary;

	if( argc != 2 || !(img_rgb=imread(argv[1], 1)).data)
		return -1;
	imshow("original", img_rgb);

	cvtColor(img_rgb,img_hsv,CV_BGR2HSV);

	vector<Point> goalRoots;
	double hor_hist[img_hsv.cols];
	int ver_hist[img_hsv.rows];
	remove_background(img_hsv, img_lines_binary, img_posts_binary, img_ball_binary, goalRoots, hor_hist, ver_hist);

	vector<Vec4i> lines;

	line_extraction(img_lines_binary, lines, 5, 5);
#if 0
	detect_ellipse(img_lines_binary, lines);
#endif

#if 1
	// the next two lines find intersections and return
	// a triple X,T,L with the confidence, orientation, position
	// for each one...
	vector<field_intersection> result_intersections;
	line_features(img_lines_binary, lines, result_intersections);
#else
	// the next two lines find intersections and return
	// only the most probable cross type for each intersectiion
	// with it confidence, orientation, position
	vector<field_point> result_intersections;
	line_most_prob_features(img_lines_binary, lines, result_intersections);
#endif

	goalPostDetection(img_posts_binary, goalRoots, hor_hist, ver_hist);
	waitKey(0);
	return 0;
}
