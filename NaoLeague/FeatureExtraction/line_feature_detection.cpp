#include <math.h>
#include <cv.h>
#include <highgui.h>
#include <time.h>
#include "geometry_utils.h"

using namespace cv;
using namespace std;

double unifRand()
{
	return rand() / double(RAND_MAX);
}
double unifRand(double a, double b)
{
	return (b-a)*unifRand() + a;
}

void seed()
{
	srand(time(0));
}

void line_features(Mat image, vector<Vec4i> lines)
{
	Mat black = Mat::zeros(image.rows, image.cols, CV_8UC3);
	seed();
	for(int i = 3; i < 5; i++)
	{
		int r = floor(unifRand(0.0, 255.0));
		int g = floor(unifRand(0.0, 255.0));
		int b = floor(unifRand(0.0, 255.0));

		line( black, Point(lines[i][0], lines[i][1]),
		      Point(lines[i][2],lines[i][3]), Scalar(g,r,b), 1, 8 );
	}

	Point* inters = intersection(lines[3],lines[4]);
	if(inters != NULL){
		circle(black, Point(inters->x, inters->y), 2, Scalar(0,255,0), 1, 8, 0);
	}
	
	imshow("s", black);
	return;
}
