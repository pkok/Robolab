#include <iostream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>
#include "geometry_utils.h"
#include "goal_detection.h"
#include "line_detection.h"
#include "hough_line_detection.h"

using namespace std;
using namespace cv;

int horizontal_post(Mat image, vector<Vec4i> lines_hor, vector<posts_lines> best_candidate_lines, Vec4i &result)
{
	if(best_candidate_lines.size() == 0)
	{
		double best_measure = DBL_MAX;
		int best_hor_match = 0;
		for (int i = 0; i < lines_hor.size(); ++i)
		{
			double hor_line_angle = line_angle(lines_hor[i]);
			double hor_angle_measure = 1 - abs(hor_line_angle - 90) / 90;
			double length_measure = 1 - line_length(lines_hor[i]) / image.cols;
			double temp_sum_measure = length_measure * hor_angle_measure;
			if(temp_sum_measure < best_measure)
			{
				best_measure = temp_sum_measure;
				best_hor_match = i;
			}
		}
		result = lines_hor[best_hor_match];
		return 1;
	}
	else
	{
		double best_measure = DBL_MAX;
		int best_hor_match = 0;
		for (int i = 0; i < lines_hor.size(); ++i)
		{
			double hor_line_angle = line_angle(lines_hor[i]);
			double hor_angle_measure = 1 - abs(hor_line_angle - 90) / 90;
			double crossing_measure = 0;
			Point middle_point = line_middle_point(lines_hor[i]);
			double length_measure = 1 - line_length(lines_hor[i]) / image.cols;
			double position_measure = 0;
			for (int j = 0; j < best_candidate_lines.size(); ++j)
			{
				Point* intersect = intersection_full(lines_hor[i], best_candidate_lines[j].line);
				if (intersect != NULL)
				{
					if(best_candidate_lines[j].line[0] < best_candidate_lines[j].line[2])
					{
						crossing_measure += points_distance(Point(best_candidate_lines[j].line[0], 
							best_candidate_lines[j].line[1]), Point(intersect->x, intersect->y));
						position_measure += abs(middle_point.x - best_candidate_lines[j].line[0]);
					}
					else
					{
						crossing_measure += points_distance(Point(best_candidate_lines[j].line[2],
							best_candidate_lines[j].line[3]), Point(intersect->x, intersect->y));
						position_measure += abs(middle_point.x - best_candidate_lines[j].line[2]);

					}
				}
				else
				{
					crossing_measure = DBL_MAX;
				}
			}
			double temp_sum_measure = length_measure * (hor_angle_measure + position_measure + pow(crossing_measure,2));
			if(temp_sum_measure < best_measure)
			{
				best_measure = temp_sum_measure;
				best_hor_match = i;
			}
		}
		if (best_measure < 100)
		{
			result = lines_hor[best_hor_match];
			return 2;
		}
		else
		{
			result = lines_hor[best_hor_match];
			return 0;
		}
	}
}

void vertical_posts(Mat image, int x_offset, vector<Vec4i> lines_ver, vector<int> candidate_cols, vector<posts_lines> &best_candidate_lines)
{
	double match_measure;
	for (int i = 0; i < candidate_cols.size(); ++i)
	{
		int best_match = 0;
		double match_measure = DBL_MAX;
		double ver_line_angle;
		double angle_measure;
		double position_measure;
		double temp_match_measure;
		double length_measure;
		Point middle_point;
		for(int j = 0; j < lines_ver.size(); j ++)
		{
			ver_line_angle = line_angle(lines_ver[j]);
			angle_measure = abs(ver_line_angle - 90);
			middle_point = line_middle_point(lines_ver[j]);
			position_measure = pow(abs(middle_point.y + x_offset - candidate_cols[i]),2);
			length_measure = (1 - line_length(lines_ver[j]) / image.rows);
			temp_match_measure = length_measure * (angle_measure + position_measure);
			if (temp_match_measure < match_measure)
			{
				match_measure = temp_match_measure;
				best_match = j;
			}
		}
		if(match_measure < 50)
		{
			posts_lines temp;
			temp.candidate = i;
			temp.line = lines_ver[best_match];
			best_candidate_lines.push_back(temp);
			lines_ver.erase(lines_ver.begin() + best_match);
		}
	}
}

void extend_line(Mat image, Vec4i &line)
{
	Point left, right;
	if(line[0] > line[2])
	{
		left = Point(line[0], line[1]);
		right = Point(line[2], line[3]);
	}
	else
	{
		left = Point(line[2], line[3]);
		right = Point(line[0], line[1]);
	}

	double angle_top_bottom = points_angle_360(left, right);
	bool end = false;
	Point newPoint;
	int len = 0;
	do
	{
		newPoint.x = floor(right.x + len * sin(CV_PI * angle_top_bottom/180));
		newPoint.y = floor(right.y + len * cos(CV_PI * angle_top_bottom/180));
		if(newPoint.x < 0 || newPoint.y < 0 || newPoint.x >= image.rows || newPoint.y >= image.cols ||
		        (int)image.at<Vec3b>(newPoint.x,newPoint.y)[0] == 0)
		{
			end = true;
		}
		else
		{
			line[0] = newPoint.x;
			line[1] = newPoint.y;
		}
		len++;
	}
	while(!end);

	end = false;
	len = 0;
	angle_top_bottom = points_angle_360(right, left);
	do
	{
		newPoint.x = floor(left.x + len * sin(CV_PI * angle_top_bottom/180));
		newPoint.y = floor(left.y + len * cos(CV_PI * angle_top_bottom/180));
		if(newPoint.x < 0 || newPoint.y < 0 || newPoint.x >= image.rows || newPoint.y >= image.cols ||
		        (int)image.at<Vec3b>(newPoint.x,newPoint.y)[0] == 0)
		{
			end = true;
		}
		else
		{
			line[2] = newPoint.x;
			line[3] = newPoint.y;
		}
		len++;
	}
	while(!end);
}

Rect crop_region_interest(Mat image, double* hor_hist, int* ver_hist, vector<int> local_maxima)
{
	int right_threshold = 0;
	bool right = false;
	bool left = false;
	int left_thershold = 0;
	int top_interest = 0;
	int bottom_interest = 0;
	//horizontal area of interest
	for (int i = 0; i < image.rows; i++)
	{
		if(ver_hist[i] > 0.0)
		{
			top_interest = i;
			break;
		}
	}
	for (int i = image.rows; i >= 0; i--)
	{
		if(ver_hist[i] > 0.0)
		{
			bottom_interest = i;
			break;
		}
	}

	// vertical area of interest...
	int counter = 0;
	int candidate = 0;
	for( int i=local_maxima[candidate]; i >= 0; i--)
	{
		if(hor_hist[i] == 0)
		{
			counter ++;
			if(counter == CROP_THRESHOLD)
			{
				left_thershold = i;
				left = true;
				break;
			}
		}
		else
		{
			counter = 0;
		}
	}
	counter = 0;
	candidate = local_maxima.size() - 1;
	for( int i=local_maxima[candidate]; i < image.cols; i++)
	{
		if(hor_hist[i] == 0)
		{
			counter ++;
			if(counter == CROP_THRESHOLD)
			{
				right_threshold = i;
				right = true;
				break;
			}
		}
		else
		{
			counter = 0;
		}
	}
	int x = (left) ? (left_thershold) : 0;
	int y = top_interest;
	int width = (right) ? (right_threshold - x) : (image.cols - x);
	int height = bottom_interest - top_interest;
	return Rect(x,y,width,height);
}


void goalPostDetection(Mat image, vector<Point> goalRoots, double* hor_hist, int* ver_hist)
{
	vector<goalposts> goalPosts;
	vector<int> candidate_cols;
	bool inTransition = false;
	double last_maximum = 0.0;
	int last_candidate = 0;

	imshow("original binary", image);
	for( int i = 0; i < goalRoots.size(); i++ )
	{
		hor_hist[goalRoots[i].y] *= ROOT_GAIN;
	}


	for( int i = 0; i < image.cols; i++ )
	{
		if(inTransition)
		{
			if( hor_hist[i] > last_maximum )
			{
				inTransition = true;
				last_maximum = hor_hist[i];
				last_candidate = i;
			}
			else
			{
				if(hor_hist[i] < CONTROL_MAX * last_maximum)
				{
					inTransition = false;
					candidate_cols.push_back(last_candidate);
					cout << "candidate y_pos :" << last_candidate << endl;
				}
			}
		}
		else
		{
			if( hor_hist[i] > HIST_THRESHOLD )
			{
				inTransition = true;
				last_maximum = hor_hist[i];
				last_candidate = i;
			}
		}

	}

	Rect roi = crop_region_interest(image, hor_hist, ver_hist, candidate_cols);
	Mat cropped;
	image.copyTo(cropped);
	cropped = cropped(roi);

	// resize(post_image, dst, dst.size(), 0, 0, 0);
	vector<Vec4i> lines_ver;
	line_extraction(cropped, lines_ver, SAMPLING_VER, 0);
	
	// from the produced lines find for each candidate to be a goalpost
	// the best line representing it...
	vector<posts_lines> best_candidate_lines;
	vertical_posts(image, roi.x, lines_ver, candidate_cols, best_candidate_lines);
	for (int i = 0; i < best_candidate_lines.size(); ++i)
	{
		extend_line(cropped, best_candidate_lines[i].line);
	}

	vector<Vec4i> lines_hor;
	Vec4i line_hor_pass;
	line_extraction(cropped, lines_hor, 0, SAMPLING_HOR);
	int find_horizontal = horizontal_post(image, lines_hor, best_candidate_lines, line_hor_pass);
	if(find_horizontal == 2)
	{
		extend_line(cropped, line_hor_pass);
	}

	for (int i = 0; i < best_candidate_lines.size(); ++i)
	{
		line( cropped, Point(best_candidate_lines[i].line[1], best_candidate_lines[i].line[0]),
		      Point(best_candidate_lines[i].line[3],best_candidate_lines[i].line[2]), Scalar(0,0,255), 2, 8 );
	}

	line( cropped, Point(line_hor_pass[1], line_hor_pass[0]),
		     Point(line_hor_pass[3],line_hor_pass[2]), Scalar(255,0,0), 2, 8 );
	
	imshow("post_cropped", cropped);
	return;
}
