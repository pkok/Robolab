#include <iostream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>
#include "geometry_utils.h"
#include "img_processing.h"
#include "line_detection.h"

using namespace cv;
using namespace std;

void mark_lines(Mat image, vector<Point> &points, int hor_step, int ver_step)
{
	if(hor_step > 0)
	{
		for(int i = 0; i < image.rows; i += hor_step)
		{
			bool pass = false;
			int col = 0;
			if((int)image.at<Vec3b>(i,0)[0] > 0) pass = true;
			for(int j = 0; j < image.cols; j++)
			{
				if(pass)
				{
					if(image.at<Vec3b>(i,j)[0] == 0 || i == (image.cols - 1))
					{
						pass = false;
						int pixel = floor((col + j)/2);
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
	}
	if(ver_step > 0)
	{
		for(int j = 0; j < image.cols; j += ver_step)
		{
			bool pass = false;
			int row = 0;
			if((int)image.at<Vec3b>(0,j)[0] > 0) pass = true;
			for(int i = 0; i < image.rows; i++)
			{
				if(pass)
				{
					if(image.at<Vec3b>(i,j)[0] == 0 || i == (image.rows - 1))
					{
						pass = false;
						int pixel = floor((row + i)/2);
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
}
void find_candidate_points(vector<Point> points, Point start, Point previous, vector<Point> line, vector<point_dis> &candidates)
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

void store_line(Mat image, vector< vector<Point> > &lines, vector<Point> line)
{
	if(line.size() == 1)
		return;
	if(lines.size() == 0)
	{
		lines.push_back(line);
	}
	else
	{
		Point current[2];
		double temp_distance_current;
		double max_distance_current = 0;
		for( int i = 0; i < line.size(); i++)
		{
			for( int j = 0; j < line.size(); j++)
			{
				temp_distance_current = points_distance(line[i], line[j]);
				if(temp_distance_current > max_distance_current)
				{
					max_distance_current = temp_distance_current;
					current[0] = line[i];
					current[1] = line[j];
				}
			}
		}
		double current_line_angle = points_angle(current[0], current[1]);
		double best_match_error = DBL_MAX;
		double temp_match_error;
		int best_match_line;
		for(int i=0; i < lines.size(); i++)
		{
			Point stored[2];
			double temp_distance_stored;
			double max_distance_stored = 0;
			for( int ii = 0; ii < lines[i].size(); ii++)
			{
				for( int jj = 0; jj < lines[i].size(); jj++)
				{
					temp_distance_stored = points_distance(lines[i][ii], lines[i][jj]);
					if(temp_distance_stored > max_distance_stored)
					{
						max_distance_stored = temp_distance_stored;
						stored[0] = lines[i][ii];
						stored[1] = lines[i][jj];
					}
				}
			}

			double stored_line_angle = points_angle(stored[0], stored[1]);
			if(abs(current_line_angle - stored_line_angle) < 10)
			{
				Point start_new;
				Point end_new;
				Point close_stored;
				Point close_new;
				double max_distance_new = 0;
				double temp_distance_new;
				double min_distance_new = DBL_MAX;
				for(int jj=0; jj < 2; jj++ )
				{
					for(int j=0; j < 2; j++ )
					{
						temp_distance_new = points_distance(stored[jj], current[j]);
						if(temp_distance_new > max_distance_new)
						{
							max_distance_new = temp_distance_new;
							start_new = stored[jj];
							end_new = current[j];
						}
						if(temp_distance_new < min_distance_new)
						{
							min_distance_new = temp_distance_new;
							close_stored = stored[jj];
							close_new = current[j];
						}
					}
				}
				line_error(lines[i], line, start_new, end_new, temp_match_error);
				temp_match_error /= lines[i].size() + line.size();
				if(compute_white_ratio(image, close_new, close_stored) < 0.4)
				{
					temp_match_error += 2000;
				}
			}
			else
			{
				temp_match_error = DBL_MAX;
			}

			if(temp_match_error < best_match_error)
			{
				best_match_error = temp_match_error;
				best_match_line = i;
			}
		}
		double threshold_error = lines[best_match_line].size() * 20;
		if(best_match_error < threshold_error)
		{
			for(int i=0; i<line.size(); i++)
			{
				lines[best_match_line].push_back(line[i]);
			}
		}
		else
		{
			lines.push_back(line);
		}
	}
	return;
}

void line_extraction(Mat image, vector<Vec4i> &produced_lines, int hor_step, int ver_step)
{
	vector<Point> points;
	vector< vector<Point> > lines;
	mark_lines(image, points, hor_step, ver_step);
	while(points.size() != 0)
	{
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
					if(sum_error < 5)
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
		store_line(image, lines, line);
		line.clear();
	}
	//lines construction and export...
	for(int i = 0; i < lines.size(); i++)
	{
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
		produced_lines.push_back(Vec4i(point1.x, point1.y, point2.x, point2.y));
	}
}
