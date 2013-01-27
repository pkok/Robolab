#include <iostream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>
#include "geometry_utils.h"

using namespace std;
using namespace cv;

void goalPostDetection(Mat yellow, vector<Point> goalRoots, int* hor_hist)
{

	Mat temp = Mat::zeros(yellow.rows, yellow.cols, CV_8UC3);
	Mat temp1 = Mat::zeros(yellow.rows, yellow.cols, CV_8UC3);
	yellow.copyTo(temp);
	imshow("yellow",temp);
	for( size_t i = 0; i < goalRoots.size(); i++ )
	{
		circle(temp, Point(goalRoots[i].y, goalRoots[i].x), 2, Scalar(0,0,255), 1, 8, 0);
		hor_hist[goalRoots[i].y] *= 1.6; 

	}
	imshow("possible roots",temp);

	for( int i = 0; i < temp.cols; i++ )
	{
		circle(temp1, Point(i, temp.rows - 1 - hor_hist[i]), 2, Scalar(0,0,255), 1, 8, 0);

	}
	imshow("possible",temp1);


	// Mat dst, color_dst;
	// Canny( yellow, dst, 50, 200, 3 );
	// cvtColor( dst, color_dst, CV_GRAY2BGR );

	// vector<Vec4i> lines;
	// HoughLinesP( dst, lines, 1, CV_PI/300, 20, 20, 20 );

	// vector<Vec4i> lines_ver;
	// vector<Vec4i> lines_hor;
	// for( int i = 0; i < lines.size(); i++ )
	// {
	// 	std::cout << line_angle(Vec4i(lines[i][0],lines[i][1],lines[i][2],lines[i][3])) << std::endl;
	// 	if(Vec4i(line_angle(lines[i][0],lines[i][1],lines[i][2],lines[i][3])) > 70 &&
	// 	        line_angle(Vec4i(lines[i][0],lines[i][1],lines[i][2],lines[i][3])) < 110)
	// 	{
	// 		lines_ver.push_back(Vec4i(lines[i][0],lines[i][1],lines[i][2],lines[i][3]));
	// 	}
	// 	else
	// 	{
	// 		lines_hor.push_back(Vec4i(lines[i][0],lines[i][1],lines[i][2],lines[i][3]));
	// 	}
	// }

	// // to determine the number of goals we see we are going
	// // to split up the lines according to if they have yellow pixels
	// // among them.

	// for( size_t i = 0; i < lines_ver.size(); i++ )
	// {
	// 	line( color_dst, Point(lines_ver[i][0], lines_ver[i][1]),
	// 	      Point(lines_ver[i][2], lines_ver[i][3]), Scalar(0,0,255), 2, 8 );
	// }
	// for( size_t i = 0; i < lines_hor.size(); i++ )
	// {
	// 	line( color_dst, Point(lines_hor[i][0], lines_hor[i][1]),
	// 	      Point(lines_hor[i][2], lines_hor[i][3]), Scalar(255,0,0), 2, 8 );
	// }


	// imshow("posts2",color_dst);
	return;
}