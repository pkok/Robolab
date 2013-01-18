#include <iostream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>

using namespace cv;
using namespace std;

const int BACK_THRESHOLD = 6;


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
			if(hsv_range(image.at<Vec3b>(i,j), 20, 38, 100, 255, 100, 255))
			{
				ass_val_pixel(posts.at<Vec3b>(i,j), 255, 255, 255);
				ass_val_pixel2pixel(field.at<Vec3b>(i,j), image.at<Vec3b>(i,j));
			}

			// check for the horizontal start of the field
			if(background)
			{
				if(hsv_range(image.at<Vec3b>(i,j), 38, 75, 50, 255, 50, 255))
				{
					counter ++;
					ass_val_pixel(field.at<Vec3b>(i,j), 0, 0, 0);
					if(counter > BACK_THRESHOLD)
					{
						for( int k = 0; k < BACK_THRESHOLD; k++)
						{
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
				ass_val_pixel2pixel(field.at<Vec3b>(i,j), image.at<Vec3b>(i,j));
			}
		}
	}
	imshow("field", field);
	imshow("posts", posts);


	for(int j = 0; j < image.cols; j++)
	{
		background = true;
		counter = 0;
		for(int i = 0; i < image.rows; i++)
		{
			if(hsv_range(field.at<Vec3b>(i,j), 0, 255, 0, 60, 220, 255))
			{
				ass_val_pixel(lines.at<Vec3b>(i,j), 255, 255, 255);
			}
			else
			{
				ass_val_pixel(lines.at<Vec3b>(i,j), 0, 0, 0);
			}
		}
	}
	imshow("lines", lines);
}

vector<Vec4i> probabilistic_hough_trans(Mat src)
{

	Mat dst, color_dst;
	Canny( src, dst, 50, 200, 3 );
	cvtColor( dst, color_dst, CV_GRAY2BGR );
	imshow("grey", color_dst);

	vector<Vec4i> lines;
	HoughLinesP( dst, lines, 1, CV_PI/300, 20, 10, 10 );
	for( size_t i = 0; i < lines.size(); i++ )
	{
		line( color_dst, Point(lines[i][0], lines[i][1]),
		      Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
	}
	imshow("lines", color_dst);
	return lines;
}

int main(int argc, char** argv)
{
	clock_t startTime = clock();
	Mat img_rgb, img_hsv;
	Mat img_lines, img_posts, img_ball;

	if( argc != 2 || !(img_rgb=imread(argv[1], 1)).data)
		return -1;
	imshow("original", img_rgb);

	cvtColor(img_rgb,img_hsv,CV_BGR2HSV);

	remove_background(img_hsv, img_lines, img_posts, img_ball);

	probabilistic_hough_trans(img_lines);

	std::cout << double( clock() - startTime )*1000 / (double)CLOCKS_PER_SEC<< " ms." << std::endl;
	waitKey(0);
	return 0;
}
