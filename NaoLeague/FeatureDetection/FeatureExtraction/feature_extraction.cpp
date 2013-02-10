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

void extract_features(Mat img_rgb, vector<field_point> &result_intersections, vector<goalposts> &goalPosts)
{
	Mat img_hsv;
	Mat img_lines_binary, img_posts_binary, img_ball_binary;

	cvtColor(img_rgb,img_hsv,CV_BGR2HSV);

	vector<Point> goalRoots;
	double hor_hist[img_hsv.cols];
	int ver_hist[img_hsv.rows];
	remove_background(img_hsv, img_lines_binary, img_posts_binary, img_ball_binary, goalRoots, hor_hist, ver_hist);

	vector<Vec4i> lines;
	line_extraction(img_lines_binary, lines, 5, 5);

	vector<Vec4i> ellipse_prob_lines;
	detect_ellipse(img_lines_binary, lines, ellipse_prob_lines);

	line_most_prob_features(img_lines_binary, lines, ellipse_prob_lines, result_intersections);

	goalPostDetection(img_posts_binary, goalRoots, hor_hist, ver_hist, goalPosts);

	for (int i = 0; i < goalPosts.size(); ++i)
	{
		circle(img_rgb, Point(goalPosts[i].root_position.y, goalPosts[i].root_position.x), 2, Scalar(0,0,255), 2, 8, 0);
		cout << goalPosts[i].type << endl;
		Point2f norm_pix = normalizePixelPosition(img_rgb, goalPosts[i].root_position);
		cout << norm_pix << endl;
		dis_bear test;
		test = pixel2dis_bear(norm_pix);
		cout << test.distance << endl;
		cout << test.bearing << endl;
	}
	imshow("vabrea", img_rgb);
	return;
}
