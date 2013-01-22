#include <iostream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>
#include "background.h"
#include "lines.h"

using namespace cv;
using namespace std;

#define BACK_THRESHOLD 1

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
	Mat background_border = Mat::zeros(image.rows, image.cols, CV_8UC1);
  vector<Vec2i> background_border_points = vector<Vec2i>();
	
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
	}

  int minimum_datapoints_used = (int) ((double) background_border_points.size() * 0.2 * 0.3);
  int iterations = 20;
  double threshold = 0.5;
  int close_data_values = minimum_datapoints_used * 4;
  Vec4f fitted_line = Vec4f();
  Vec4f fitted_line2 = Vec4f();
  Vec4f fitted_line3 = Vec4f();
  vector<Vec2i> left_border = vector<Vec2i>();
  vector<Vec2i> middle_border = vector<Vec2i>();
  vector<Vec2i> right_border = vector<Vec2i>();
  vector<Vec2i> used_data = vector<Vec2i>();
  vector<Vec2i> used_data2 = vector<Vec2i>();
  vector<Vec2i> used_data3 = vector<Vec2i>();
  double estimation_error = INFINITY;

  int i = 0;
  for (; i < background_border_points.size() * 0.33; ++i) {
    left_border.push_back(background_border_points.at(i));
  }
  for (; i < background_border_points.size() * 0.66; ++i) {
    middle_border.push_back(background_border_points.at(i));
  }
  for (; i < background_border_points.size(); ++i) {
    right_border.push_back(background_border_points.at(i));
  }

  RANSAC_line(left_border, minimum_datapoints_used, iterations, threshold, close_data_values, fitted_line, used_data, estimation_error);
  cout << "dataset size: " << left_border.size() << "|" << middle_border.size() << "|" << right_border.size() << endl;
  cout << "minimum_datapoints_used: " << minimum_datapoints_used << endl;
  cout << "close_data_values: " << close_data_values << endl;
  cout << "used_data size: " << used_data.size() << endl;
  cout << "estimation error (avg. LSE): " << estimation_error << endl;

  RANSAC_line(middle_border, minimum_datapoints_used, iterations, threshold, close_data_values, fitted_line2, used_data2, estimation_error);
  RANSAC_line(right_border, minimum_datapoints_used, iterations, threshold, close_data_values, fitted_line3, used_data3, estimation_error);

  float slope = fitted_line[1] / fitted_line[0];
  circle(field, 
      Point(fitted_line[3], fitted_line[2]), 
      4, 
      Scalar(255, 255, 0));
  line(field,
      Point(fitted_line[3] - 50*fitted_line[1], (fitted_line[2]) - 50*fitted_line[0]),
      Point(fitted_line[3] + 50*fitted_line[1], (fitted_line[2]) + 50*fitted_line[0]),
      Scalar(255, 255, 0));
  circle(background_border, 
      Point(fitted_line[3], fitted_line[2]), 
      4, 
      255);
  line(background_border,
      Point(fitted_line[3] - 50*fitted_line[1], (fitted_line[2]) - 50*fitted_line[0]),
      Point(fitted_line[3] + 50*fitted_line[1], (fitted_line[2]) + 50*fitted_line[0]),
      255);

  slope = fitted_line2[1] / fitted_line2[0];
  circle(field, 
      Point(fitted_line2[3], fitted_line2[2]), 
      4, 
      Scalar(0, 255, 0));
  line(field,
      Point(fitted_line2[3] - 50*fitted_line2[1], (fitted_line2[2]) - 50*fitted_line2[0]),
      Point(fitted_line2[3] + 50*fitted_line2[1], (fitted_line2[2]) + 50*fitted_line2[0]),
      Scalar(0, 255, 0));
  circle(background_border, 
      Point(fitted_line2[3], fitted_line2[2]), 
      4, 
      255);
  line(background_border,
      Point(fitted_line2[3] - 50*fitted_line2[1], (fitted_line2[2]) - 50*fitted_line2[0]),
      Point(fitted_line2[3] + 50*fitted_line2[1], (fitted_line2[2]) + 50*fitted_line2[0]),
      255);

  slope = fitted_line3[1] / fitted_line3[0];
  circle(field, 
      Point(fitted_line3[3], fitted_line3[2]), 
      4, 
      Scalar(255, 0, 0));
  line(field,
      Point(fitted_line3[3] - 50*fitted_line3[1], (fitted_line3[2]) - 50*fitted_line3[0]),
      Point(fitted_line3[3] + 50*fitted_line3[1], (fitted_line3[2]) + 50*fitted_line3[0]),
      Scalar(255, 0, 0));
  circle(background_border, 
      Point(fitted_line3[3], fitted_line3[2]), 
      4, 
      255);
  line(background_border,
      Point(fitted_line3[3] - 50*fitted_line3[1], (fitted_line3[2]) - 50*fitted_line3[0]),
      Point(fitted_line3[3] + 50*fitted_line3[1], (fitted_line3[2]) + 50*fitted_line3[0]),
      255);

	imshow("field", field);
	//imshow("posts", posts);
	//imshow("lines", lines);
  imshow("background border", background_border);
}
