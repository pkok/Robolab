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

Point closest_end_point(Point* inters, Vec4i line){
	
	Point close;
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
	return close;
}

Point closest_point(Point* inters, Vec4i line){
	
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

double l_measure(Point* inters, Mat image, Vec4i line_i, Vec4i line_j){

	double measure;
	Point intersection = Point(inters->x, inters->y);
	Point close_i = closest_end_point(inters, line_i);
	Point close_j = closest_end_point(inters, line_j);

	double white_i = compute_white_ratio(image, close_i, intersection);
	double white_j = compute_white_ratio(image, close_j, intersection);

	//how close is the intersection point to the line start or end...
	Point middle_point_i = line_middle_point(line_i);
	double middle_point_distance_i = points_distance(middle_point_i, close_i);

	double l_measure_one_i = ( middle_point_distance_i - 
	points_distance(intersection, close_i)) / middle_point_distance_i;

	if(l_measure_one_i < 0.0) l_measure_one_i = 0.0;

	Point middle_point_j = line_middle_point(line_j);
	double middle_point_distance_j = points_distance(middle_point_j, close_j);

	double l_measure_one_j = ( middle_point_distance_j - 
		points_distance(intersection, close_j)) / middle_point_distance_j;

	if(l_measure_one_j < 0.0) l_measure_one_j = 0.0;

	double l_measure_i = l_measure_one_i * white_i;
	double l_measure_j = l_measure_one_j * white_j;

	measure = l_measure_j * l_measure_i;

	return measure;
}

double x_measure(Point* inters, Mat image, Vec4i line_i, Vec4i line_j){

	double measure;
	Vec4i line_base;
	Vec4i line_t;
	Point intersection = Point(inters->x, inters->y);

	Point close_i = closest_point(inters, line_i);
	Point close_j = closest_point(inters, line_j);

	if(intersection_in_line(intersection, line_i) && intersection_in_line(intersection, line_j))
	{
		//how close is the intersection point to the line start or end...
		Point middle_point_i = line_middle_point(line_i);
		double middle_point_distance_i = points_distance(middle_point_i, close_i);

		double x_measure_i = (middle_point_distance_i - points_distance(intersection, close_i)) / middle_point_distance_i;

		if(x_measure_i < 0.0) x_measure_i = 0.0;

		x_measure_i = pow(x_measure_i, 5);

		Point middle_point_j = line_middle_point(line_j);
		double middle_point_distance_j = points_distance(middle_point_j, close_j);

		double x_measure_j = (middle_point_distance_j - points_distance(intersection, close_j)) / middle_point_distance_j;

		if(x_measure_j < 0.0) x_measure_j = 0.0;

		x_measure_j = pow(x_measure_j, 5);

		return max(1.0 - (x_measure_j + x_measure_i), 0.0);

	}
	else
	{
		return 0.0;
	}

}

double t_measure(Point* inters, Mat image, Vec4i line_i, Vec4i line_j){

	double measure;
	Vec4i line_base;
	Vec4i line_t;
	Point intersection = Point(inters->x, inters->y);

	Point close_i = closest_end_point(inters, line_i);
	Point close_j = closest_end_point(inters, line_j);

	if(intersection_in_line(intersection, line_i) && intersection_in_line(intersection, line_j))
	{
		//how close is the intersection point to the line start or end...
		Point middle_point_i = line_middle_point(line_i);
		double middle_point_distance_i = points_distance(middle_point_i, close_i);

		double t_measure_i = (middle_point_distance_i - points_distance(intersection, close_i)) / middle_point_distance_i;

		if(t_measure_i < 0.0) t_measure_i = 0.0;

		t_measure_i = pow(t_measure_i, 5);

		Point middle_point_j = line_middle_point(line_j);
		double middle_point_distance_j = points_distance(middle_point_j, close_j);

		double t_measure_j = (middle_point_distance_j - points_distance(intersection, close_j)) / middle_point_distance_j;

		if(t_measure_j < 0.0) t_measure_j = 0.0;

		t_measure_j = pow(t_measure_j, 5);

		return abs(t_measure_j - t_measure_i);
	}
	else if(intersection_in_line(intersection, line_i) || intersection_in_line(intersection, line_j))
	{

		// distinquish among these two lines which is the 
		// the base of the possible T shape and which is 
		// the other...
		// line_base is the T's |
		Point close_point_base;
		Point close_point_t;
		double dis_i = points_distance(intersection, close_i);
		double dis_j = points_distance(intersection, close_j);
		double dis_line_base;

		if(intersection_in_line(intersection, line_i)){
			line_base = line_j;
			close_point_base = close_j;
			close_point_t = intersection;
			line_t = line_i;
			dis_line_base = dis_j;
		}
		else
		{
			line_base = line_i;
			close_point_base = close_i;
			close_point_t = closest_end_point(inters, line_j);
			line_t = line_j;
			dis_line_base = dis_i;
		}

		double white_base = compute_white_ratio(image, close_point_base, close_point_t);
		double white_t = 1;

		Point middle_point_base = line_middle_point(line_base);
		double middle_point_distance_base = points_distance(middle_point_base, close_point_base);

		double base_line_measure = ( middle_point_distance_base - 
		points_distance(intersection, close_point_base)) / middle_point_distance_base;

		if(base_line_measure < 0.0) base_line_measure = 0.0;

		base_line_measure = white_base * pow(base_line_measure, 5);

		Point middle_point_t = line_middle_point(line_t);
		double middle_point_distance_t = points_distance(middle_point_t, close_point_t);

		double t_measure = (middle_point_distance_t - points_distance(intersection, close_point_t)) / middle_point_distance_t;

		if(t_measure < 0.0) t_measure = 0.0;

		t_measure = pow(t_measure, 5);

		return abs(t_measure - base_line_measure);
	}
	else
	{
		return 0.0;
	}
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

	int intersection_num = 0;
	for(int i = 0; i < lines.size(); i++)
	{
		for(int j = i+1; j < lines.size(); j++)
		{
			if(i != j)
			{

				double angle_i = points_angle(Point(lines[i][1], lines[i][0]), Point(lines[i][3], lines[i][2]));
				double angle_j = points_angle(Point(lines[j][1], lines[j][0]), Point(lines[j][3], lines[j][2]));
				Point* inters = intersection(lines[i], lines[j], image);
		
				if(inters != NULL)
				{
					intersection_num ++;
					Mat temp;
					black.copyTo(temp);
					
					double l_m = l_measure(inters, image, lines[i], lines[j]);
					double t_m = t_measure(inters, image, lines[i], lines[j]);
					double x_m = x_measure(inters, image, lines[i], lines[j]);

					cout << "intersection " << intersection_num << " L: " << l_m << " T: " << t_m << " X: " << x_m <<endl;
					stringstream ss;
					ss << intersection_num;

					circle(temp, Point(inters->y, inters->x), 2, Scalar(0,0,255), 2, 8, 0);	
					imshow("intersection"+ss.str(),temp);	
				}
				
			}
		}
	}
	imshow("s", black);
	return;
}
