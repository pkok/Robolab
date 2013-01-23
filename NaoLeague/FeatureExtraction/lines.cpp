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
	for(int i = 0; i < image.rows; i += 5)
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
	for(int j = 0; j < image.cols; j += 5)
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
void find_candidate_points(vector<Point> points,Point start, Point previous, vector<Point> line, vector<point_dis> &candidates)
{
	for(int i=0; i < points.size(); i++)
	{
		if(!equal_points(points[i],previous))
		{
			double temp_sim_value = points_distance(previous, points[i]);
			if(line.size() >= 3)
			{
				temp_sim_value = temp_sim_value * 0.05 + point_line_distance(points[i], Vec4i(start.x, start.y, previous.x, previous.y));
			}
			if(candidates.size() == 5)
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

void find_best_candidate(Mat image, vector<point_dis> candidates, vector<Point> line,  Point start, Point previous, Point &best_candidate, double &best_score)
{

	best_score = DBL_MAX;
	double white;
	double distance;
	double score;
	Point temp;

	for(int i = 0; i < candidates.size(); i ++)
	{
		temp = candidates[i].pnt;
		score = 0;
		distance = points_distance(previous, temp);
		white = compute_white_ratio(image, previous, temp);
		score = (1.01 - white) * distance;
		if(score < best_score)
		{
			best_candidate = temp;
			best_score = score;
		}
	}
	return;
}

void line_error(vector<Point> line, Point start, Point best_candidate, double &error){
	error = 0;
	for(int i = 0; i < line.size(); i++)
	{
		error += pow(point_line_distance(line[i],
		Vec4i(start.x, start.y, best_candidate.x, best_candidate.y)),3);
	}
	return;
}

void double_line_error(vector<Point> line1, vector<Point> line2, Point start, Point end, double &error){
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

void delete_point(Point element, vector<Point> &points)
{
	for(int i=0; i < points.size(); i++)
	{
		if(equal_points(element, points[i]))
		{
			points.erase(points.begin() + i);
			break;
		}
	}
	return;
}

void store_line(vector< vector<Point> > &lines, vector<Point> line)
{
	if(line.size() == 1)
		return;
	if(lines.size() == 0){
		lines.push_back(line);
	}else{
		
		Point current[2];
		current[0] = line[0];
		double temp_distance_current;
		double max_distance_current = 0;
		for( int i = 0; i < line.size(); i++){
			temp_distance_current = points_distance(current[0], line[0]);
			if(temp_distance_current > max_distance_current){
				max_distance_current = temp_distance_current;
				current[1] = line[i];
			}
		}
		double current_line_angle = points_angle(current[0], current[1]);

		double best_match_error = DBL_MAX;
		double temp_match_error;
		int best_match_line;
		for(int i=0; i < lines.size(); i++){
			
			
			Point stored[2];
			stored[0] = line[0];
			double temp_distance_stored;
			double max_distance_stored = 0;
			for( int j = 0; j < lines[i].size(); j++){
				temp_distance_stored = points_distance(stored[0], lines[i][0]);
				if(temp_distance_current > max_distance_stored){
					max_distance_stored = temp_distance_stored;
					stored[1] = lines[i][j];
				}
			}

			double stored_line_angle = points_angle(stored[0], stored[1]);

			if(abs(current_line_angle - stored_line_angle) < 10){

				Point start_new;
				Point end_new;
				double max_distance_new = 0;
				double temp_distance_new;
				for(int jj=0; jj < 2; jj++ ){
					for(int j=0; j < 2; j++ ){
						temp_distance_new = points_distance(stored[jj], current[j]);
						if(temp_distance_new > max_distance_new){
							max_distance_new = temp_distance_new;
							start_new = stored[jj];
							end_new = current[j];
						}
					}
				}

				double_line_error(lines[i], line, start_new, end_new, temp_match_error);

			}else
			{
				temp_match_error = DBL_MAX;
			}

			if(temp_match_error < best_match_error){
				best_match_error = temp_match_error;
				best_match_line = i;
			}
		}
		cout << best_match_error << endl;
		if(best_match_error < 50){
			for(int i=0; i<line.size(); i++){
				lines[best_match_line].push_back(line[i]);
			}
		}else{
			lines.push_back(line);
		}
	}
	return;
}

void line_clustering(Mat image)
{
	Mat black = Mat::zeros(image.rows, image.cols, CV_8UC3);
	vector<Point> points;
	vector< vector<Point> > lines;
	mark_lines(image, black, points);
	imshow("binary", image);
	int iteration = 0;

	Mat t;
	black.copyTo(t);
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
			vector<point_dis> candidates;
			find_candidate_points(points, start,  previous, line, candidates);

			// find best candidate to connect
			Point best_candidate;
			double error;
			find_best_candidate(image, candidates, line, start, previous, best_candidate, error);
			candidates.clear();
			if(error > 3)
			{
				end = true;
			}
			else
			{
				if(line.size() >= 4)
				{
					double sum_error;
					line_error(line, start, best_candidate, sum_error);
					if(sum_error < 10)
					{
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
				else
				{
					line.push_back(best_candidate);
					previous = best_candidate;
				}
			}
			delete_point(previous, points);
		}
		while(!end);

		Mat t;
		black.copyTo(t);
		for(int i=0; i < line.size(); i++)
		{
			circle(t, Point(line[i].y, line[i].x), 2, Scalar(0,255,0), 1, 8, 0);
		}
		circle(t, Point(start.y, start.x), 2, Scalar(255,0,0), 1, 8, 0);
		stringstream ss;
		ss << iteration;
		imshow("line"+ss.str(),t);
		store_line(lines, line);
		line.clear();
		// break;

	}





	//lines visualization...
	for(int i = 0; i < lines.size(); i++)
	{
		cout << "line" << endl;
		Point point1,point2;
		double max_distance = 0;
		for(int j1 = 0; j1 < lines[i].size(); j1++)
		{
			for(int j2 = 0; j2 < lines[i].size(); j2++)
			{
				if(j1 != j2)
				{
					double temp = points_distance(lines[i][j1], lines[i][j2]);
					if(temp > max_distance)
					{
						point1 = lines[i][j1];
						point2 = lines[i][j2];
						max_distance = temp;
					}
				}
			}
		}
		line( black, Point(point1.y, point1.x),
		      Point(point2.y, point2.x), Scalar(0,0,255), 1, 8 );
	}
	imshow("s", black);
}
