#include <iostream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>
#include "lines.h"
#include <queue>

using namespace cv;
using namespace std;

#define ANGLE_THR 10

struct point_dis
{
	Point pnt;
	double distance;
};


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

double compute_white_ratio(Mat image, Point point1, Point point2)
{

	int white_counter = 0;
	int all_counter = 0;
	int x0 = point1.x;
	int y0 = point1.y;
	int x1 = point2.x;
	int y1 = point2.y;
	int dx = abs(x1-x0);
	int dy = abs(y1-y0);

	int err;
	int sx = 0;
	int sy = 0;
	if (x0 < x1)
	{
		sx = 1;
	}
	else
	{
		sx = -1;
	}
	if (y0 < y1)
	{
		sy = 1;
	}
	else
	{
		sy = -1;
	}
	err = dx-dy;

	while(1)
	{
		all_counter ++;
		if((int)image.at<Vec3b>(x0,y0)[0] != 0)
		{
			white_counter ++;
		}
		if (x0 == x1 && y0 == y1)
		{
			break;
		}
		int e2 = 2*err;
		if (e2 > -dy)
		{
			err = err - dy;
			x0 = x0 + sx;
		}
		if(e2 <  dx)
		{
			err = err + dx;
			y0 = y0 + sy;
		}
	}
	return (double) white_counter/all_counter;
}

double points_angle(Point point1, Point point2)
{
	double angle = atan2(point2.x-point1.x,point2.y-point1.y);
	angle = angle * (180 / CV_PI);
	if( angle < 0 )
		angle += 180;
	return angle;
}

void mark_lines(Mat image, Mat &point_image, vector<Point> &points)
{

	for(int i = 0; i < image.rows; i += 10)
	{

		bool pass = false;
		int col = 0;
		for(int j = 0; j < image.cols; j++)
		{
			if(pass)
			{
				if(image.at<Vec3b>(i,j)[0] == 0)
				{
					pass = false;
					int pixel = floor((col + j)/2);
					point_image.at<Vec3b>(i,pixel)[0] = 255;
					point_image.at<Vec3b>(i,pixel)[1] = 255;
					point_image.at<Vec3b>(i,pixel)[2] = 255;
					points.push_back(Point(i ,pixel));
				}
			}
			else
			{
				if(image.at<Vec3b>(i,j)[0] > 0)
				{
					pass = true;
					col = j;
				}
			}
		}
	}


	for(int j = 0; j < image.cols; j += 10)
	{
		bool pass = false;
		int row = 0;
		for(int i = 0; i < image.rows; i++)
		{
			if(pass)
			{
				if(image.at<Vec3b>(i,j)[0] == 0)
				{
					pass = false;

					int pixel = floor((row + i)/2);
					point_image.at<Vec3b>(pixel,j)[0] = 255;
					point_image.at<Vec3b>(pixel,j)[1] = 255;
					point_image.at<Vec3b>(pixel,j)[2] = 255;
					points.push_back(Point(pixel,j));

				}
			}
			else
			{
				if(image.at<Vec3b>(i,j)[0] > 0)
				{
					pass = true;
					row = i;
				}
			}
		}
	}
}

void find_candidate_points(vector<Point> points, Point start, Point previous, vector<Point> line, vector<point_dis> &candidates)
{
	for(int i=0; i < points.size(); i++)
	{
		if(!equal_points(points[i],previous))
		{
			double temp_sim_value = points_distance(previous, points[i]);
			if(line.size() > 1)
			{
				temp_sim_value = point_line_distance(points[i], Vec4i(start.x, start.y, previous.x, previous.y));
			}

			if(candidates.size() == 3)
			{
				for(int j=0; j<candidates.size(); j++)
				{
					for(int l=0; l<j; l++)
					{
						if(candidates[j].distance < candidates[l].distance)
						{
							point_dis temp=candidates[j];
							candidates[j]=candidates[l];
							candidates[l]=temp;
						}
					}
				}
				if(temp_sim_value < candidates[candidates.size() - 1].distance)
				{
					candidates.erase(candidates.begin() + candidates.size() - 1);
					point_dis temp;
					temp.pnt = points[i];
					temp.distance = temp_sim_value;
					candidates.push_back(temp);
				}
			}
			else
			{
				point_dis temp;
				temp.pnt = points[i];
				temp.distance = temp_sim_value;
				candidates.push_back(temp);
			}
		}
	}

}


void line_clustering(Mat image)
{
	Mat black = Mat::zeros(image.rows, image.cols, CV_8UC3);
	vector<Point> points;
	vector< vector<Point> > lines;
	mark_lines(image, black, points);

	imshow("binary", image);
	int iteration = 0;

	while(points.size() != 0)
	{
		iteration++;
		vector<Point> line;
		bool end = false;
		Point start = points[0];
		Point previous = points[0];
		do
		{
			line.push_back(previous);
			double min_dis = DBL_MAX;

			vector<point_dis> candidates;
			find_candidate_points(points, start, previous, line, candidates);

			Point best_candidate;
			double best_score = DBL_MAX;
			for(int i = 0; i < candidates.size(); i ++)
			{
				Point next = candidates[i].pnt;
				double white = compute_white_ratio(image, previous, next);
				double distance = points_distance(previous, next);
				double angle = abs(points_angle(start, next) - points_angle(start, previous));
				double score;
				if(line.size() > 1)
				{
					score = (1.01 - white) * angle;
				}
				else
				{
					score = (1.01 - white) * distance;
				}
				if(white < 0.7)
				{
					score += 300;
				}
				if(score < best_score)
				{
					best_candidate = next;
					best_score = score;
				}
			}

			if(best_score > 300)
			{
				end = true;
				for(int i=0; i < points.size(); i++)
				{
					if(equal_points(previous, points[i]))
					{
						points.erase(points.begin() + i);
						break;
					}
				}
			}
			else
			{
				double sum_error = 0;
				if(line.size() >= 3)
				{
					for(int i = 0; i < line.size(); i++)
					{
						sum_error += pow(point_line_distance(line[i],
						                                     Vec4i(start.x, start.y, best_candidate.x, best_candidate.y)),3);
					}
				}
				if(sum_error < 200)
				{
					line.push_back(best_candidate);
					for(int i=0; i < points.size(); i++)
					{
						if(equal_points(previous, points[i]))
						{
							points.erase(points.begin() + i);
							break;
						}
					}

					previous = best_candidate;

				}
				else
				{
					if(line.size() <= 4 && line.size() != 0)
					{
						for(int i=0; i<line.size(); i++)
						{
							points.push_back(line[i]);
						}
						line.clear();
					}
					end = true;
				}
			}
		}
		while(!end);
		Mat t;
		black.copyTo(t);
		for(int i=0; i < line.size(); i++)
		{
			circle(t, Point(line[i].y, line[i].x), 2, Scalar(0,255,0), 1, 8, 0);
		}
		circle(t, Point(start.y, start.x), 2, Scalar(0,0,255), 1, 8, 0);
		stringstream ss;
		ss << iteration;
		imshow("line"+ss.str(),t);
		lines.push_back(line);
		line.clear();
	}


	// lines visualization...

	// for(int i = 0; i < lines.size(); i++)
	// {
	// 	cout << "sdfSBRB" << endl;
	// 	Point point1,point2;
	// 	double max_distance = 0;
	// 	for(int j1 = 0; j1 < lines[i].size(); j1++)
	// 	{
	// 		for(int j2 = 0; j2 < lines[i].size(); j2++)
	// 		{
	// 			if(j1 != j2)
	// 			{
	// 				double temp = points_distance(lines[i][j1], lines[i][j2]);
	// 				if(temp > max_distance)
	// 				{
	// 					point1 = lines[i][j1];
	// 					point2 = lines[i][j2];
	// 					max_distance = temp;
	// 				}
	// 			}
	// 		}
	// 	}
	// 	line( black, Point(point1.y, point1.x),
	// 	      Point(point2.y, point2.x), Scalar(0,0,255), 1, 8 );
	// }

	imshow("s", black);
}
