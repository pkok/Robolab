#include <iostream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>
#include "geometry_utils.h"
#include "goal_detection.h" 

using namespace std;
using namespace cv;

void goalPostDetection(Mat yellow, vector<Point> goalRoots, double* hor_hist)
{

	Mat temp = Mat::zeros(yellow.rows, yellow.cols, CV_8UC3);
	Mat temp1 = Mat::zeros(yellow.rows, yellow.cols, CV_8UC3);
	yellow.copyTo(temp);
	imshow("yellow",temp);
	for( int i = 0; i < goalRoots.size(); i++ )
	{
		circle(temp, Point(goalRoots[i].y, goalRoots[i].x), 2, Scalar(0,0,255), 1, 8, 0);
		hor_hist[goalRoots[i].y] *= ROOT_GAIN;

	}

	vector<int> candidate_cols;
	bool inTransition = false;
	double last_maximum = 0.0;
	int last_candidate = 0;
	for( int i = 0; i < temp.cols; i++ )
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

	imshow("possible roots",temp);
	for( int i = 0; i < temp.cols; i++ )
	{
		circle(temp1, Point(i, temp.rows - 1 - floor(hor_hist[i]*temp.rows)), 2, Scalar(0,0,255), 1, 8, 0);
	}
	imshow("possible",temp1);


	for( int i = 0; i < candidate_cols.size(); i++ )
	{
		line( temp1, Point(candidate_cols[i], 0),
		      Point(candidate_cols[i], temp1.rows), Scalar(255,255,255), 1, 8 );

	}

	imshow("posts2",temp1);
	
	Mat post_image;
	if(candidate_cols.size() == 0)
	{
		// no candidate..
		return;
	}
	else if(candidate_cols.size() == 1)
	{
		//horizontal area of interest...

		// vertical area of interest...
		int left_thershold = 0;
		bool left = false;
		int counter = 0;
		for( int i=candidate_cols[0]; i >= 0; i--){
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
		int right_threshold = 0;
		bool right = false;
		counter = 0; 
		for( int i=candidate_cols[0]; i < temp1.cols; i++){
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
		cout << "r" << right << endl;
		cout << "l" << left << endl;
		int x = (left) ? (left_thershold) : 0;
		int y = 0;
		int width = (right) ? (candidate_cols[0] + right_threshold) : (temp1.cols - candidate_cols[0]);
		int height = temp1.rows;
		post_image = temp(Rect(x,y,width,height));
		imshow("post_cropped", post_image);
	}
















	return;
}