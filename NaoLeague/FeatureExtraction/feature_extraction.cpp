#include <iostream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>
#include "background.h"

using namespace cv;
using namespace std;

vector<Vec4i> probabilistic_hough_trans(Mat src)
{

	Mat dst, color_dst;
	Canny( src, dst, 50, 200, 3 );
	cvtColor( dst, color_dst, CV_GRAY2BGR );
	imshow("grey", color_dst);

	vector<Vec4i> lines;
	HoughLinesP( dst, lines, 1, CV_PI/300, 20, 10, 10 );
	for( size_t i = 0; i < lines.size(); i++ )
	{
		line( color_dst, Point(lines[i][0], lines[i][1]),
		      Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
	}
	imshow("lines", color_dst);
	return lines;
}

int main(int argc, char** argv)
{
	clock_t startTime = clock();
	Mat img_rgb, img_hsv;
	Mat img_lines, img_posts, img_ball;

	if( argc != 2 || !(img_rgb=imread(argv[1], 1)).data)
		return -1;
	imshow("original", img_rgb);

	cvtColor(img_rgb,img_hsv,CV_BGR2HSV);

	remove_background(img_hsv, img_lines, img_posts, img_ball);

	probabilistic_hough_trans(img_lines);

	std::cout << double( clock() - startTime )*1000 / (double)CLOCKS_PER_SEC<< " ms." << std::endl;
	waitKey(0);
	return 0;
}
