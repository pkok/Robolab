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
#include "dis_ang_translation.h"

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

	for (int i = 0; i < goalPosts.size(); ++i)
	{
		if(goalPosts[i].type !=3){
			circle(img_rgb, Point(goalPosts[i].root_position.y, goalPosts[i].root_position.x), 2, Scalar(0,0,255), 2, 8, 0);
			cout << "GoalPost type: " << goalPosts[i].type << endl;
			cout << "pixel " << normalizePixelPosition(img_rgb, goalPosts[i].root_position) << endl;
			dis_bear test = pixel2dis_bear(normalizePixelPosition(img_rgb, goalPosts[i].root_position));
			cout << "distance " << test.distance << endl;
			cout << "angle " << test.bearing << endl;
			cout << "--------------" << endl;
		}
	}
	imshow("va",img_rgb);
	waitKey(0);
	return 0;
}

