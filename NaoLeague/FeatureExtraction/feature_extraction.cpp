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

void extract_features(Mat img_rgb, vector<field_point> &result_intersections, vector<goalposts> &goalPosts)
{
	Mat img_hsv;
	Mat img_lines_binary, img_posts_binary, img_ball_binary;

	cvtColor(img_rgb,img_hsv,CV_BGR2HSV);

	vector<Point> goalRoots;
	double hor_hist[img_hsv.cols];
	int ver_hist[img_hsv.rows];
	remove_background(img_rgb,img_hsv, img_lines_binary, img_posts_binary, img_ball_binary, goalRoots, hor_hist, ver_hist);

	imwrite("bin_lines.png", img_lines_binary);
	imwrite("bin_posts.png", img_posts_binary);
	vector<Vec4i> lines;

	line_extraction(img_lines_binary, lines, 5, 5);

	// vector<Vec4i> ellipse_prob_lines;
	// detect_ellipse(img_lines_binary, lines, ellipse_prob_lines);

	// line_most_prob_features(img_lines_binary, lines, ellipse_prob_lines, result_intersections);

	Mat ee = Mat::zeros(img_hsv.rows, img_hsv.cols, CV_8UC3);
	for (int i = 0; i < ee.rows; ++i)
	{
		for (int j = 0; j < ee.cols; ++j)
		{
			ee.at<Vec3b>(i,j)[0] = 0;
			ee.at<Vec3b>(i,j)[1] = 140;
			ee.at<Vec3b>(i,j)[2] = 0;
		}
	}
	for (int i = 0; i < lines.size(); ++i)
	{
		line( ee, Point(lines[i][1], lines[i][0]),Point(lines[i][3],lines[i][2]), Scalar(255,255,255), 1, 8 );
	}

	// goalPostDetection(img_posts_binary, goalRoots, hor_hist, ver_hist, goalPosts);
	// for (int i = 0; i < goalPosts.size(); ++i)
	// {
	// 	if(goalPosts[i].type == R_POST)
	// 	{
	// 		line( ee, Point(goalPosts[i].line[1], goalPosts[i].line[0]),
	// 		      Point(goalPosts[i].line[3],goalPosts[i].line[2]), Scalar(0, 255, 255), goalPosts[i].width, 8 );
	// 		cout << "r " << goalPosts[i].width << " " << goalPosts[i].root_confidence << endl;
	// 	}
	// 	if(goalPosts[i].type == L_POST)
	// 	{
	// 		line( ee, Point(goalPosts[i].line[1], goalPosts[i].line[0]),
	// 		      Point(goalPosts[i].line[3],goalPosts[i].line[2]), Scalar(0, 255, 255), goalPosts[i].width, 8 );
	// 		cout << "l " << goalPosts[i].width << " " << goalPosts[i].root_confidence << endl;
	// 	}
	// 	if(goalPosts[i].type == O_POST)
	// 	{
	// 		line( ee, Point(goalPosts[i].line[1], goalPosts[i].line[0]),
	// 		      Point(goalPosts[i].line[3],goalPosts[i].line[2]), Scalar(0, 255, 255), goalPosts[i].width, 8 );
	// 		cout << "o " << goalPosts[i].width << " " << goalPosts[i].root_confidence << endl;
	// 	}
	// 	if(goalPosts[i].type == V_POST)
	// 	{
	// 		line( ee, Point(goalPosts[i].line[1], goalPosts[i].line[0]),
	// 		      Point(goalPosts[i].line[3],goalPosts[i].line[2]), Scalar(0, 255, 255), goalPosts[i].width, 8 );
	// 		cout << "v " << goalPosts[i].width << " " << goalPosts[i].root_confidence << endl;
	// 	}
	// }
	imshow("gergreb", ee);
	imwrite("reconstruction.png",ee);
	
	
	return;
}
