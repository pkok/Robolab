#include <iostream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>
#include "background.h"

using namespace cv;
using namespace std;

#define BACK_THRESHOLD 6

#define YEL_HUE_MIN 20
#define YEL_HUE_MAX  38
#define YEL_SAT_MIN  100
#define YEL_SAT_MAX  255
#define YEL_VAL_MIN  100
#define YEL_VAL_MAX  255

#define GR_HUE_MIN  38
#define GR_HUE_MAX  75
#define GR_SAT_MIN  50
#define GR_SAT_MAX  255
#define GR_VAL_MIN  50
#define GR_VAL_MAX  255

#define WH_HUE_MIN  0
#define WH_HUE_MAX  255
#define WH_SAT_MIN  0
#define WH_SAT_MAX  60
#define WH_VAL_MIN  200
#define WH_VAL_MAX  255

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

void remove_background(Mat image, Mat &lines, Mat &posts, Mat &ball)
{

	lines = Mat::zeros(image.rows, image.cols, CV_8UC3);
	posts = Mat::zeros(image.rows, image.cols, CV_8UC3);
	ball = Mat::zeros(image.rows, image.cols, CV_8UC3);
	Mat field = Mat::zeros(image.rows, image.cols, CV_8UC3);
	
	// boolean variable which declares if the current row pixel is above field
	// height...
	bool background, continuous;
	int counter;
	for(int j = 0; j < image.cols; j++)
	{
		background = true;
		counter = 0;
		for(int i = 0; i < image.rows; i++)
		{
			// hue refers to yellow, binary white will be stored in the goalposts image
			// in order to find the posts later...
			if(hsv_range(image.at<Vec3b>(i,j), YEL_HUE_MIN, YEL_HUE_MAX, YEL_SAT_MIN, YEL_SAT_MAX, YEL_VAL_MIN, YEL_VAL_MAX))
			{
				ass_val_pixel(posts.at<Vec3b>(i,j), 255, 255, 255);
				ass_val_pixel2pixel(field.at<Vec3b>(i,j), image.at<Vec3b>(i,j));
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
						for( int k = 0; k < BACK_THRESHOLD + 5; k++)
						{
							if((i-k) >= 0)
								ass_val_pixel2pixel(field.at<Vec3b>(i-k,j), image.at<Vec3b>(i-k,j));
						}
						background = false;
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
	}
	//imshow("field", field);
	//imshow("posts", posts);
	//imshow("lines", lines);
}