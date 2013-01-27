#include <iostream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>
#include "img_processing.h"
#include "line_detection.h"
#include "line_feature_detection.h"
#include "ellipse_detector.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	clock_t startTime = clock();
	Mat img_rgb, img_hsv;
	Mat img_lines_binary, img_posts_binary, img_ball_binary;

	if( argc != 2 || !(img_rgb=imread(argv[1], 1)).data)
		return -1;
	imshow("original", img_rgb);

	cvtColor(img_rgb,img_hsv,CV_BGR2HSV);

	remove_background(img_hsv, img_lines_binary, img_posts_binary, img_ball_binary);

	vector<Vec4i> lines;

	line_extraction(img_lines_binary, lines);

	detect_ellipse(img_lines_binary, lines);

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

	std::cout << double( clock() - startTime )*1000 / (double)CLOCKS_PER_SEC<< " ms." << std::endl;
	waitKey(0);
	return 0;
}
