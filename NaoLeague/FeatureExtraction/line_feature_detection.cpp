#include <math.h>
#include <cv.h>
#include <highgui.h>
#include <time.h>
#include "geometry_utils.h"
#include "img_processing.h"


using namespace cv;
using namespace std;

struct field_feature{
	Point position;
	double orientation[2];
	double confidence;
};

struct field_intersection{
	field_feature t;
	field_feature l;
	field_feature x;
};

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

Point closest_end_point(Point* inters, Vec4i line)
{

	Point close;
	double temp;
	double min_distance = DBL_MAX;
	for(int p=0; p<2; p++)
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

Point closest_point(Point* inters, Vec4i line)
{
	Point close;
	if(intersection_in_line(Point(inters->x, inters->y), line))
	{
		return Point(inters->x, inters->y);
	}
	else
	{
		double temp;
		double min_distance = DBL_MAX;
		for(int p=0; p<2; p++)
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

void l_measure(Point* inters, Mat image, Vec4i line_i, Vec4i line_j, double &l_confidence, double* l_orientation)
{
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
	l_measure_one_i = pow(l_measure_one_i,3);
	Point middle_point_j = line_middle_point(line_j);
	double middle_point_distance_j = points_distance(middle_point_j, close_j);
	double l_measure_one_j = ( middle_point_distance_j -
	                           points_distance(intersection, close_j)) / middle_point_distance_j;
	if(l_measure_one_j < 0.0) l_measure_one_j = 0.0;
	l_measure_one_j = pow(l_measure_one_j,3);
	double l_measure_i = l_measure_one_i * white_i;
	double l_measure_j = l_measure_one_j * white_j;
	l_confidence = l_measure_j * l_measure_i;
	l_orientation[0] = points_angle_360(close_i, middle_point_i);
	l_orientation[1] = points_angle_360(close_j, middle_point_j);
}

void x_measure(Point* inters, Mat image, Vec4i line_i, Vec4i line_j, double t_measure, double &x_confidence, double* x_orientation)
{
	Point intersection = Point(inters->x, inters->y);
	if(intersection_in_line(intersection, line_i) && intersection_in_line(intersection, line_j))
	{
		x_confidence = 1 - t_measure;
		x_orientation[0] = points_angle(Point(line_i[1], line_i[0]), Point(line_i[3], line_i[2]));
		x_orientation[1] = points_angle(Point(line_j[1], line_j[0]), Point(line_j[3], line_j[2]));
	}
	else
	{
		x_confidence = 0.0;
		x_orientation[0] = 0.0;
		x_orientation[1] = 0.0;
	}
}

void t_measure(Point* inters, Mat image, Vec4i line_i, Vec4i line_j, double &t_confidence, double* t_orientation)
{
	double measure;
	Point intersection = Point(inters->x, inters->y);
	if(intersection_in_line(intersection, line_i) && intersection_in_line(intersection, line_j))
	{
		Point close_i = closest_end_point(inters, line_i);
		Point close_j = closest_end_point(inters, line_j);
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
		t_confidence = abs(t_measure_j - t_measure_i);
		if(t_measure_j > t_measure_i)
		{
			t_orientation[0] = points_angle_360(close_j, middle_point_j);
			t_orientation[1] = points_angle_360(close_i, middle_point_i);
		}
		else
		{
			t_orientation[0] = points_angle_360(close_i, middle_point_i);
			t_orientation[1] = points_angle_360(close_j, middle_point_j);
		}
		
	}
	else if(intersection_in_line(intersection, line_i) || intersection_in_line(intersection, line_j))
	{
		// cout << "Two part t" << endl;
		// distinquish among these two lines which is the
		// the base of the possible T shape and which is
		// the other...
		// line_base is the T's |
		Vec4i line_base;
		Vec4i line_t;
		Point close_base;
		Point close_t;
		if(intersection_in_line(intersection, line_i))
		{
			line_base = line_j;
			line_t = line_i;
		}
		else
		{
			line_base = line_i;
			line_t = line_j;
		}
		close_base = closest_end_point(inters, line_base);
		close_t = intersection;
		double white = compute_white_ratio(image, intersection, close_base);
		Point middle_point_base = line_middle_point(line_base);
		Point middle_point_t = line_middle_point(line_t);
		double point_distance_base = points_distance(middle_point_base, close_base);
		double base_line_measure = ( point_distance_base -
		                             points_distance(intersection, close_base)) / point_distance_base;
		if(base_line_measure < 0.0) base_line_measure = 0.0;
		base_line_measure = pow(base_line_measure, 1);
		double point_distance_t = points_distance(intersection, closest_end_point(inters, line_t));
		double t_line_measure = (points_distance(closest_end_point(inters, line_t), middle_point_t) - point_distance_t) / points_distance(closest_end_point(inters, line_t), middle_point_t);
		if(t_line_measure < 0.0) t_line_measure = 0.0;
		t_line_measure = 1.0 - pow(t_line_measure,5);
		t_confidence = white * t_line_measure * base_line_measure;
		t_orientation[0] = points_angle_360(close_base, middle_point_base);
		t_orientation[1] = points_angle_360(close_t, middle_point_t);
	}
	else
	{
		t_confidence = 0.0;
		t_orientation[0] = 0.0;
		t_orientation[1] = 0.0;
	}
}

void store_intersection(field_intersection current, vector<field_intersection> &intersections)
{
	if(intersections.size() == 0)
	{
		intersections.push_back(current);
	}
	else
	{

	}
}


void line_features(Mat image, vector<Vec4i> lines)
{
	//visualization stuff
	Mat black = Mat::zeros(image.rows, image.cols, CV_8UC3);
	seed();
	for(int i = 0; i < lines.size(); i ++)
	{
		int r = floor(unifRand(0.0, 255.0));
		int g = floor(unifRand(0.0, 255.0));
		int b = floor(unifRand(0.0, 255.0));
		line( black, Point(lines[i][1], lines[i][0]),
		      Point(lines[i][3],lines[i][2]), Scalar(g,r,b), 1, 8 );
		
	}
	//..end of visualization stuff
	double l_confidence, t_confidence, x_confidence, angle_i, angle_j;
	double l_orientation[2], t_orientation[2], x_orientation[2];
	Point* inters;
	int intersection_num = 0;
	vector<field_intersection> intersections;
	for(int i = 0; i < lines.size(); i++)
	{
		for(int j = i+1; j < lines.size(); j++)
		{
			if(i != j)
			{
				angle_i = points_angle(Point(lines[i][1], lines[i][0]), Point(lines[i][3], lines[i][2]));
				angle_j = points_angle(Point(lines[j][1], lines[j][0]), Point(lines[j][3], lines[j][2]));
				inters = intersection(lines[i], lines[j], image);
				if(abs(angle_j - angle_i) > 10)
				{
					if(inters != NULL)
					{
						intersection_num ++;
						Mat temp;
						black.copyTo(temp);

						l_measure(inters, image, lines[i], lines[j], l_confidence, l_orientation);
						t_measure(inters, image, lines[i], lines[j], t_confidence, t_orientation);
						x_measure(inters, image, lines[i], lines[j], t_confidence, x_confidence, x_orientation);

						field_feature l, t, x;
						l.position = Point(inters->y, inters->x);
						l.confidence = l_confidence;

						t.position = Point(inters->y, inters->x);
						t.confidence = t_confidence;

						x.position = Point(inters->y, inters->x);
						x.confidence = x_confidence;

						for(int a = 0; a < 2; a ++){
							x.orientation[a] = x_orientation[a];
							l.orientation[a] = l_orientation[a];
							t.orientation[a] = t_orientation[a];
						}
						
						field_intersection current_intersection;
						current_intersection.x = x;
						current_intersection.t = t;
						current_intersection.l = l;

						store_intersection(current_intersection, intersections);


						if(t_confidence > 0.0 || l_confidence > 0.0 ||  x_confidence > 0.0)
						{
							cout << i << endl;
							cout << j << endl;
							double l1 = points_distance(Point(lines[i][1], lines[i][0]), Point(lines[i][3], lines[i][2]));
							double l2 = points_distance(Point(lines[j][1], lines[j][0]), Point(lines[j][3], lines[j][2]));

							cout << "inter " << inters->x << "," << inters->y << " "  << intersection_num << " L: " << l_confidence << " T: " << t_confidence << " X: " << x_confidence << " l:" << l1+l2 << endl;
							stringstream ss;
							ss << intersection_num;

							circle(temp, Point(inters->y, inters->x), 2, Scalar(0,0,255), 2, 8, 0);
							imshow("intersection"+ss.str(),temp);
						}
					}
				}
			}
		}
	}
	imshow("s", black);
	return;
}
