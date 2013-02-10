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
#include "dis_ang_translation.h"

using namespace cv;
using namespace std;

void extract_features(Mat img_rgb, vector<field_point> &result_intersections,
		vector<goalposts> &goalPosts) {
	Mat img_hsv;
	Mat img_lines_binary, img_posts_binary, img_ball_binary;

	cvtColor(img_rgb, img_hsv, CV_BGR2HSV);

	vector<Point> goalRoots;
	double hor_hist[img_hsv.cols];
	int ver_hist[img_hsv.rows];
	remove_background(img_hsv, img_lines_binary, img_posts_binary,
			img_ball_binary, goalRoots, hor_hist, ver_hist);

	vector<Vec4i> lines;
	line_extraction(img_lines_binary, lines, 5, 5);

	vector<Vec4i> ellipse_prob_lines;
	detect_ellipse(img_lines_binary, lines, ellipse_prob_lines);

	line_most_prob_features(img_lines_binary, lines, ellipse_prob_lines,
			result_intersections);

	goalPostDetection(img_posts_binary, goalRoots, hor_hist, ver_hist,
			goalPosts);

	for (int i = 0; i < goalPosts.size(); ++i)
	{
		if(goalPosts[i].type == 0 || goalPosts[i].type == 1){
			line( img_posts_binary, Point(goalPosts[i].root_position.y,goalPosts[i].root_position.x) ,
	     	Point(goalPosts[i].top_position.y,goalPosts[i].top_position.x), Scalar(0,0,255), goalPosts[i].width, 8 );
		}
	}
	imshow("va",img_posts_binary);

	return;
}
