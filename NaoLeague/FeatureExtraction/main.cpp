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

