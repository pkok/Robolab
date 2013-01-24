#ifndef GEOMETRY_UTILS_H
#define GEOMETRY_UTILS_H

#include <math.h>
#include <cv.h>
#include <highgui.h>

using namespace cv;

void line_error(vector<Point> line, Point start, Point best_candidate, double &error);

void line_error(vector<Point> line1, vector<Point> line2, Point start, Point end, double &error);

double points_distance(Point point1, Point point2);

double point_line_distance(Point point, Vec4i line);

bool equal_points(Point point1, Point point2);

double points_angle(Point point1, Point point2);

Point* intersection(Vec4i line1, Vec4i line2);

double line_angle(Vec4i line);

#endif
