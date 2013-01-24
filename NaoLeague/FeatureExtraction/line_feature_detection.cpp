#include <math.h>
#include <cv.h>
#include <highgui.h>
#include <time.h>
#include "geometry_utils.h"
#include "img_processing.h"

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
	for(int i = 0; i < lines.size(); i++)
	{
		int r = floor(unifRand(0.0, 255.0));
		int g = floor(unifRand(0.0, 255.0));
		int b = floor(unifRand(0.0, 255.0));

		line( black, Point(lines[i][0], lines[i][1]),
		      Point(lines[i][2],lines[i][3]), Scalar(g,r,b), 1, 8 );
	}

	for(int i = 0; i < lines.size(); i++)
	{
		for(int j = 0; j < lines.size(); j++)
		{
			if(i != j)
			{
				Point* inters = intersection(lines[i],lines[j]);
				if(abs(line_angle(lines[i]) - line_angle(lines[j])) >= 5)
				{
					if(inters != NULL)
					{
						// Point close_point_line1;
						// Point close_point_line2;
						// double min_dis_p1 = DBL_MAX;
						// double min_dis_p2 = DBL_MAX;
						// for(int ii = 0; ii < 2; ii ++)
						// {
						// 	double temp_dis_point1 = points_distance(Point(lines[i][2*ii+1], lines[i][2*ii]), Point(inters->x, inters->y));
						// 	double temp_dis_point2 = points_distance(Point(lines[j][2*ii+1], lines[j][2*ii]), Point(inters->x, inters->y));
						// 	if(temp_dis_point1 < min_dis_p1){
						// 		min_dis_p1 = temp_dis_point1;
						// 		close_point_line1 = Point(lines[i][2*ii+1], lines[i][2*ii]);
						// 	}

						// }
						// if(compute_white_ratio(image, close_point_line1, Point(inters->x, inters->y)) > 0.5){
							circle(black, Point(inters->x, inters->y), 2, Scalar(0,255,0), 1, 8, 0);
						// }
					}
				}
			}
		}
	}
	
	imshow("s", black);
	return;
}
