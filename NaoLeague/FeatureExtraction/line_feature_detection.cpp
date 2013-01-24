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

void closest_intersection_point(Point* inters, Vec4i line){
	
	Point close;
	if(intersection_in_line(Point(inters->y, inters->x), lines[i]))
	{
		return Point(inters->y, inters->x);
	}
	else
	{
		double temp;
		double min_distance = DBL_MAX;
		for(int p=0; p<2;p++)
		{
			double temp = points_distance(Point(line[2*p+1], line[2*p]), Point(inters->y, inters->x));
			if(temp < min_distance)
			{
				min_distance = temp;
			 	close = Point(line[2*p+1], line[2*p]);
			}
		}
	}
	return close;
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
				Point* inters = intersection(lines[i], lines[j], image);
				if(abs(line_angle(lines[i]) - line_angle(lines[j])) >= 5)
				{
					if(inters != NULL)
					{
						
						
						Point close_i = closest_intersection_point(inters, lines[i]);
						Point close_j = closest_intersection_point(inters, lines[j]);
						

						double white1 = compute_white_ratio(image, close_i, Point(inters->y, inters->x));
						double white2 = compute_white_ratio(image, close_j, Point(inters->y, inters->x));



						if(white1 > 0.8 && white2 > 0.8){
							cout << "point" << endl;
							circle(black, Point(inters->x, inters->y), 2, Scalar(0,0,255), 1, 8, 0);	
						}
						
						
					}
				}
			}
		}
	}
	
	imshow("s", black);
	return;
}
