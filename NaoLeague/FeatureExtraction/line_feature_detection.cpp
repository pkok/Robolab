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

Point closest_intersection_point(Point* inters, Vec4i line){
	
	Point close;
	if(intersection_in_line(Point(inters->x, inters->y), line))
	{
		return Point(inters->x, inters->y);
	}
	else
	{
		double temp;
		double min_distance = DBL_MAX;
		for(int p=0; p<2;p++)
		{
			double temp = points_distance(Point(line[2*p], line[2*p+1]), Point(inters->x, inters->y));
			if(temp < min_distance)
			{
				min_distance = temp;
			 	close = Point(line[2*p], line[2*p+1]);
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
		line( black, Point(lines[i][1], lines[i][0]),
		      Point(lines[i][3],lines[i][2]), Scalar(g,r,b), 1, 8 );
	}

	for(int i = 0; i < lines.size(); i++)
	{
		for(int j = i + 1; j < lines.size(); j++)
		{
			if(i != j)
			{

				double angle_i = points_angle(Point(lines[i][1], lines[i][0]), Point(lines[i][3], lines[i][2]));
				double angle_j = points_angle(Point(lines[j][1], lines[j][0]), Point(lines[j][3], lines[j][2]));
				Point* inters = intersection(lines[i], lines[j], image);

				if(abs(angle_i - angle_j) >= 10)
				{
					
					if(inters != NULL)
					{
						
						Point intersection = Point(inters->x, inters->y);

						Point close_i = closest_intersection_point(inters, lines[i]);
						Point close_j = closest_intersection_point(inters, lines[j]);
						

						double white_i = compute_white_ratio(image, close_i, intersection);
						double white_j = compute_white_ratio(image, close_j, intersection);

						//how close is the intersection point to the line start or end...
						Point middle_point_i = line_middle_point(lines[i]);
						double middle_point_distance_i = points_distance(middle_point_i, close_i);

						double l_measure_one_i = ( middle_point_distance_i - 
							points_distance(intersection, close_i)) / middle_point_distance_i;

						if(l_measure_one_i < 0.0) l_measure_one_i = 0.0;

						Point middle_point_j = line_middle_point(lines[j]);
						double middle_point_distance_j = points_distance(middle_point_j, close_j);

						double l_measure_one_j = ( middle_point_distance_j - 
							points_distance(intersection, close_j)) / middle_point_distance_j;

						if(l_measure_one_j < 0.0) l_measure_one_j = 0.0;

						l_measure_one_i *= white_i;
						l_measure_one_j *= white_j;

						
						if(l_measure_one_i > 0.7 && l_measure_one_j > 0.7){
							cout << l_measure_one_i << endl;
							cout << l_measure_one_j << endl;
							circle(black, Point(intersection.y, intersection.x), 2, Scalar(0,0,255), 2, 8, 0);		
						}
						
						
					}
				}
			}
		}
	}
	
	imshow("s", black);
	return;
}
