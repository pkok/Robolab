#include <iostream>
#include <fstream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>
#include "img_processing.h"
#include "line_detection.h"
#include "line_feature_detection.h"
#include "ellipse_detector.h"
#include "goal_detection.h"
#include "feature_extraction.h"
#include "geometry_utils.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	Mat img_rgb;
	ofstream myfile;
  	myfile.open ("example.txt");
	for (int i = 10; i < 39; ++i)
	{
		stringstream ss;
		ss << i;
		img_rgb=imread("../dataset_triwalk/0000"+ss.str()+".png", 1);
		myfile << "0000"+ss.str() << " ";

		vector<field_point> result_intersections;

		vector<goalposts> goalPosts;

		extract_features(img_rgb, result_intersections, goalPosts);

		
		for (int i = 0; i < result_intersections.size(); ++i)
		{
			myfile << " f ";
			Point newpoint = result_intersections[i].position;
			myfile <<result_intersections[i].type << " ";
			myfile << (float)newpoint.x/img_rgb.rows << " " << (float)newpoint.y/img_rgb.cols << " ";
		}

		
		for (int i = 0; i < goalPosts.size(); ++i)
		{
			myfile << " g ";
			Point newpoint = goalPosts[i].root_position;
			myfile << goalPosts[i].type << " ";
			myfile << (float)newpoint.x/img_rgb.rows << " " << (float)newpoint.y/img_rgb.cols  << " "	;
		}
		myfile << endl;	
	}
	myfile.close();
	return 0;
}



	// goalpost detection function
	// return a vector with goalpost elements.
	// you get the position of the root, the top,
	// the line representing the post, the type of the 
	// goal post, and the confidence for the bottom to be 
	// the actual bottom of the post.

		// the next two lines find intersections and return
	// only the most probable cross type for each intersectiion
	// with it confidence, orientation, position



	// // the next two lines find intersections and return
	// // a triple X,T,L with the confidence, orientation, position
	// // for each one...
	// vector<field_intersection> result_intersections;
	// line_features(img_lines_binary, lines, result_intersections);
	// for (int i = 0; i < result_intersections.size(); ++i)
	// {
	// 	Mat sth;
	// 	img_rgb.copyTo(sth);
	// 	circle(sth, Point(result_intersections[i].position.x, result_intersections[i].position.y), 3, Scalar(0,0,255), 3, 8, 0);
	// 	cout << "num " << i << "  T: " <<  result_intersections[i].t.confidence << " L: " << result_intersections[i].l.confidence << " X: " << result_intersections[i].x.confidence<< endl;
	// 	stringstream ss;
	// 	ss << i;
	// 	imshow("feature_points"+ss.str(), sth);
	// }