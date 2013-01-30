#include <iostream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>
#include "img_processing.h"

using namespace cv;
using namespace std;

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

bool hsv_range(Vec3b pixel, int h_min, int h_max, int s_min, int s_max, int v_min, int v_max)
{
	bool isAtRange = true;
	if(pixel[0] < h_min || pixel[0] > h_max)
	{
		isAtRange = false;
	}
	if(pixel[1] < s_min || pixel[1] > s_max)
	{
		isAtRange = false;
	}
	if(pixel[2] < v_min || pixel[2] > v_max)
	{
		isAtRange = false;
	}
	return isAtRange;
}

void ass_val_pixel(Vec3b &pixel, int h, int s, int v)
{
	pixel[0] = h;
	pixel[1] = s;
	pixel[2] = v;
	return;
}

void ass_val_pixel2pixel(Vec3b &src, Vec3b &dst)
{
	src[0] = dst[0];
	src[1] = dst[1];
	src[2] = dst[2];
}

void remove_background(Mat image, Mat &lines, Mat &posts, Mat &ball, vector<Point> &goalRoot, double* hor_hist, int* ver_hist)
{

	lines = Mat::zeros(image.rows, image.cols, CV_8UC3);
	posts = Mat::zeros(image.rows, image.cols, CV_8UC3);
	ball = Mat::zeros(image.rows, image.cols, CV_8UC3);
	Mat field = Mat::zeros(image.rows, image.cols, CV_8UC3);
	Mat background_border = Mat::zeros(image.rows, image.cols, CV_8UC1);
	vector<Vec2i> background_border_points = vector<Vec2i>();

	// boolean variable which declares if the current row pixel is above field
	// height...
	bool background, continuous;
	int counter;
	for(int j = 0; j < image.cols; j++)
	{
		hor_hist[j] = 0;
		background = true;
		counter = 0;
		for(int i = 0; i < image.rows; i++)
		{
			// hue refers to yellow, binary white will be stored in the goalposts image
			// in order to find the posts later...
			if(hsv_range(image.at<Vec3b>(i,j), YEL_HUE_MIN, YEL_HUE_MAX, YEL_SAT_MIN, YEL_SAT_MAX, YEL_VAL_MIN, YEL_VAL_MAX))
			{
				hor_hist[j]++;
				ver_hist[i]++;
				ass_val_pixel(posts.at<Vec3b>(i,j), 255, 255, 255);
				ass_val_pixel2pixel(field.at<Vec3b>(i,j), image.at<Vec3b>(i,j));
				if(i < image.cols-1)
				{
					if(hsv_range(image.at<Vec3b>(i+1,j), GR_HUE_MIN, GR_HUE_MAX, GR_SAT_MIN, GR_SAT_MAX, GR_VAL_MIN, GR_VAL_MAX))
					{
						goalRoot.push_back(Point(i,j));
					}
				}
			}

			// check for the horizontal start of the field
			if(background)
			{
				if(hsv_range(image.at<Vec3b>(i,j), GR_HUE_MIN, GR_HUE_MAX, GR_SAT_MIN, GR_SAT_MAX, GR_VAL_MIN, GR_VAL_MAX))
				{
					counter ++;
					ass_val_pixel(field.at<Vec3b>(i,j), 0, 0, 0);
					if(counter > BACK_THRESHOLD)
					{
						ass_val_pixel(field.at<Vec3b>(i - BACK_THRESHOLD, j), 255, 255, 255);
						background_border.at<uchar>(i - BACK_THRESHOLD, j) = 255;
						background_border_points.push_back(Vec2i(i - BACK_THRESHOLD, j));
						for( int k = 0; k < BACK_THRESHOLD + 5; k++)
						{
							if((i-k) >= 0)
							{
								ass_val_pixel2pixel(field.at<Vec3b>(i-k,j), image.at<Vec3b>(i-k,j));
							}
						}
						background = false;
						//ass_val_pixel(background_border.at<Vec3b>(i,j), 255, 255, 255);
					}
				}
				else
				{
					continuous = false;
					counter = 0;
				}
			}
			else
			{
				if(hsv_range(image.at<Vec3b>(i,j), WH_HUE_MIN, WH_HUE_MAX, WH_SAT_MIN, WH_SAT_MAX, WH_VAL_MIN, WH_VAL_MAX))
				{
					ass_val_pixel(lines.at<Vec3b>(i,j), 255, 255, 255);
				}
				else
				{
					ass_val_pixel(lines.at<Vec3b>(i,j), 0, 0, 0);
				}
				ass_val_pixel2pixel(field.at<Vec3b>(i,j), image.at<Vec3b>(i,j));
			}
		}
		imwrite("back.png", field);
		hor_hist[j] /= image.rows;
	}
}
