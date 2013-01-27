#include <iostream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>
#include "geometry_utils.h"

using namespace std;
using namespace cv;


void detect_ellipse(Mat image, vector<Vec4i> lines)
{
	Mat black = Mat::zeros(image.rows, image.cols, CV_8UC3);
	for(int i = 0; i < lines.size(); i ++)
	{
		line( black, Point(lines[i][1], lines[i][0]),
		      Point(lines[i][3],lines[i][2]), Scalar(0,0,255), 1, 8 );
		double angle = points_angle_360(Point(lines[i][1], lines[i][0]),
		      Point(lines[i][3],lines[i][2]));
		cout << angle << endl;
		
	}
	imshow("s", black);

	std::vector<std::vector<Vec4i>> ellipse_lines;
	for(int i = 0; i < lines.size(); i ++)
	{
		std::vector<Vec4i> temp;
		double angle_current = points_angle_360(Point(lines[i][1], lines[i][0]),
		      Point(lines[i][3],lines[i][2]));
		temp.push_back(lines[i]);
		for

		
	}

	return;
}