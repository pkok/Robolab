#include <iostream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>
#include "background.h"
#include "hough.h"
#include "lines.h"

using namespace cv;
using namespace std;

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

	vector<Vec4i> lines, clustered_lines;
	//probabilistic_hough_trans(img_lines, lines);
	
	line_clustering(img_lines);

	std::cout << double( clock() - startTime )*1000 / (double)CLOCKS_PER_SEC<< " ms." << std::endl;
	waitKey(0);
	return 0;
}