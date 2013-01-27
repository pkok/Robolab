#include <math.h>
#include <cv.h>
#include <highgui.h>
#include <time.h>
#include "geometry_utils.h"
#include "img_processing.h"
#include "line_feature_detection.h"


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

field_point decide_type(field_intersection intersection)
{
	field_point result;
	// we decide x when...
	if(intersection.x.confidence > X_THRESHOLD &&
		DIFF_THRESHOLD_X * intersection.x.confidence > intersection.t.confidence &&
		DIFF_THRESHOLD_X * intersection.x.confidence > intersection.l.confidence)
	{
		result.type = X_CROSS;
		result.confidence = intersection.x.confidence;
		result.position = intersection.position;
		for(int a = 0; a < 2; a ++)
		{
			result.orientation[a] = intersection.x.orientation[a];
			result.orientation[a] = intersection.x.orientation[a];
			result.orientation[a] = intersection.x.orientation[a];
		}
	}
	else if(intersection.l.confidence > L_THRESHOLD &&
		DIFF_THRESHOLD_L * intersection.l.confidence > intersection.t.confidence &&
		DIFF_THRESHOLD_L * intersection.l.confidence > intersection.x.confidence)
	{
		result.type = L_CROSS;
		result.confidence = intersection.l.confidence;
		result.position = intersection.position;
		for(int a = 0; a < 2; a ++)
		{
			result.orientation[a] = intersection.l.orientation[a];
		}
	}
	else if(intersection.t.confidence > T_THRESHOLD &&
		DIFF_THRESHOLD_T * intersection.t.confidence > intersection.l.confidence &&
		DIFF_THRESHOLD_T * intersection.t.confidence > intersection.x.confidence)
	{
		result.type = T_CROSS;
		result.confidence = intersection.t.confidence;
		result.position = intersection.position;
		for(int a = 0; a < 2; a ++)
		{
			result.orientation[a] = intersection.t.orientation[a];
		}
	}
	else
	{
		result.type = UNKNOWN;
	}
	return result;
}

void store_intersection(field_intersection current, vector<field_intersection> &intersections)
{
	if(intersections.size() == 0)
	{
		intersections.push_back(current);
	}
	else
	{
		for(int i = 0; i < intersections.size(); i++)
		{
			if(points_distance(intersections[i].position, current.position) < DISTANCE_2T_X)
			{
				// if both intersections are going to be Ts then we check if we can
				// form an X from them.
				if (decide_type(intersections[i]).type == T_CROSS &&
					decide_type(current).type == T_CROSS)
				{
					// check angles if the are in exactly or almost 
					// opposite...
					double base_angle_stored = intersections[i].t.orientation[0];
					double base_angle_current = current.t.orientation[0];
					// we are getting the minimum angle difference
					double angle_diff_base = 180 - abs(abs(base_angle_stored - base_angle_current) - 180);
					double confidence_change = angle_diff_base / 180;
					// we have to compare the other lines' orientation 
					// as well
					double t_angle_stored = intersections[i].t.orientation[1];
					double t_angle_current = current.t.orientation[1];
					double angle_diff_t = 180 - abs(abs(t_angle_stored - t_angle_current) - 180);
					confidence_change *= angle_diff_t / 180;

					// change the stored intersection into a X cross...
					if (confidence_change > TRANS_2T_X_THRESHOLD)
					{
						// update confidence for every type
						intersections[i].t.confidence = 0;
						intersections[i].l.confidence = 0;
						intersections[i].x.confidence = 1;
						// set new angles in the produced X cross
						double angle11 = (base_angle_stored > 180) ? base_angle_stored - 180 : base_angle_stored;
						double angle12 = (base_angle_current > 180) ? base_angle_current - 180 : base_angle_current;
						double angle21 = (t_angle_stored > 180) ? t_angle_stored - 180 : t_angle_stored;
						double angle22 = (t_angle_current > 180) ? t_angle_current - 180 : t_angle_current;
						intersections[i].x.orientation[0] = (angle11 + angle12) / 2;
						intersections[i].x.orientation[1] = (angle21 + angle22) / 2;
						//update the position taking the average of the two
						// Ts positions, which are located near..
						intersections[i].position = line_middle_point(Vec4i(intersections[i].position.x,
							intersections[i].position.y, current.position.x, current.position.y));
					}
				}
				// same point...same decision about it.
				else if(decide_type(intersections[i]).type == decide_type(current).type)
				{
					// we have to merge these intersections, adding more probability 
					// to the type
				}
				// different desicion about the same point weird?
				else
				{
					// check line lengths and confidence to decide or 
					// to reject the one or the other.
				}
			}
		}
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

						if(t_confidence > 0.0 || l_confidence > 0.0 ||  x_confidence > 0.0)
						{

							field_feature l, t, x;
							l.confidence = l_confidence;
							t.confidence = t_confidence;
							x.confidence = x_confidence;

							for(int a = 0; a < 2; a ++){
								x.orientation[a] = x_orientation[a];
								l.orientation[a] = l_orientation[a];
								t.orientation[a] = t_orientation[a];
							}
							
							field_intersection current_intersection;
							current_intersection.position = Point(inters->y, inters->x);
							current_intersection.x = x;
							current_intersection.t = t;
							current_intersection.l = l;

							store_intersection(current_intersection, intersections);
						
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
