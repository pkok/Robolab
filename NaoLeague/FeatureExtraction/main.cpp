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
	if( argc != 2 || !(img_rgb=imread(argv[1], 1)).data)
		return -1;
	vector<field_point> result_intersections;
	vector<goalposts> goalPosts;
	extract_features(img_rgb, result_intersections, goalPosts);
	waitKey(0);
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