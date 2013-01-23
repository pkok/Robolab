#include <math.h>
#include <cv.h>
#include <highgui.h>
#include "geometry_utils.h"

using namespace cv;

void line_error(vector<Point> line, Point start, Point best_candidate, double &error)
{
	error = 0;
	for(int i = 0; i < line.size(); i++)
	{
		error += pow(point_line_distance(line[i],
		                                 Vec4i(start.x, start.y, best_candidate.x, best_candidate.y)),3);
	}
	return;
}
void line_error(vector<Point> line1, vector<Point> line2, Point start, Point end, double &error)
{
	error = 0;
	for(int i = 0; i < line1.size(); i++)
	{
		error += pow(point_line_distance(line1[i],
		                                 Vec4i(start.x, start.y, end.x, end.y)),3);
	}
	for(int i = 0; i < line2.size(); i++)
	{
		error += pow(point_line_distance(line2[i],
		                                 Vec4i(start.x, start.y, end.x, end.y)),3);
	}
	return;
}
double points_distance(Point point1, Point point2)
{
	double dx = point2.x - point1.x;
	double dy = point2.y - point1.y;
	double distance = sqrt(dx*dx + dy*dy);
	return distance;
}
double point_line_distance(Point point, Vec4i line)
{
	double normalLength = hypot(line[2] - line[0], line[3] - line[1]);
	return abs((point.x - line[0]) * (line[3] - line[1]) - (point.y - line[1]) * (line[2] - line[0])) / normalLength;
}
bool equal_points(Point point1, Point point2)
{
	return (point1.x == point2.x && point1.y == point2.y);
}
double points_angle(Point point1, Point point2)
{
	double angle = atan2(point2.x-point1.x,point2.y-point1.y);
	angle = angle * (180 / CV_PI);
	if( angle < 0 )
		angle += 180;
	return angle;
}
