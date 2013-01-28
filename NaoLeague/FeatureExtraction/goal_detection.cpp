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

void goalPostDetection(Mat image, vector<Point> goalRoots, double* hor_hist, int* ver_hist)
{
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

	// imshow("possible roots",temp);
	// for( int i = 0; i < temp.cols; i++ )
	// {
	// 	circle(temp, Point(i, temp.rows - 1 - floor(hor_hist[i]*temp.rows)), 2, Scalar(0,0,255), 1, 8, 0);
	// }
	// imshow("possible",temp1);


	// for( int i = 0; i < candidate_cols.size(); i++ )
	// {
	// 	line( temp1, Point(candidate_cols[i], 0),
	// 	      Point(candidate_cols[i], temp1.rows), Scalar(255,255,255), 1, 8 );

	// }
	// imshow("posts2",temp1);
	
	Mat cropped;
	int right_threshold = 0;
	bool right = false;
	bool left = false;
	int left_thershold = 0;
	int top_interest = 0;
	int bottom_interest = 0;
	if(candidate_cols.size() == 0)
	{
		return;
	}
	else if(candidate_cols.size() >= 1)
	{
		//horizontal area of interest
		for (int i = 0; i < image.rows; i++){
			if(ver_hist[i] > 0.0){
				top_interest = i;
				break;
			}
		}
		for (int i = image.rows; i >= 0; i--){
			if(ver_hist[i] > 0.0){
				bottom_interest = i;
				break;
			}
		}

		// vertical area of interest...
		int counter = 0;
		int candidate = 0;
		for( int i=candidate_cols[candidate]; i >= 0; i--){
			cout << "l" << hor_hist[i] << endl;
			if(hor_hist[i] == 0){
				counter ++;
				if(counter == CROP_THRESHOLD){
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
		candidate = candidate_cols.size() - 1;
		for( int i=candidate_cols[candidate]; i < image.cols; i++){
			if(hor_hist[i] == 0){
				counter ++;
				if(counter == CROP_THRESHOLD){
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

	}
	cout << "r" << top_interest << endl;
	cout << "l" << bottom_interest << endl;
	int x = (left) ? (left_thershold) : 0;
	int y = top_interest;
	int width = (right) ? (right_threshold - x) : (image.cols - x);
	int height = bottom_interest - top_interest;
	cropped = image(Rect(x,y,width,height));
	// Mat dst = Mat::zeros(post_image.rows * 2, post_image.cols * 2, CV_8UC3);
	// resize(post_image, dst, dst.size(), 0, 0, 0);
	imshow("post_cropped", cropped);


	vector<Vec4i> lines;
	line_extraction(cropped, lines, 5, 0);

	for(int i = 0; i < lines.size(); i ++)
	{
		line( cropped, Point(lines[i][1], lines[i][0]),
		      Point(lines[i][3],lines[i][2]), Scalar(0,0,255), 2, 8 );
		
	}
	imshow("s", cropped);

	return;
}