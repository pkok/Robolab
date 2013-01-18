#include <iostream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>
#include "lines.h"

using namespace cv;
using namespace std;

double line_angle(Vec4i line)
{
	double angle = atan2(line[1] - line[3], line[0] - line[2]);
	double res = angle * 180 / CV_PI;
	return abs(res);
}


double similarity_measure(Vec4i line1, Vec4i line2)
{

}

void initialize_clusters(vector<Vec4i> lines, vector< vector<Vec4i> > &cluster)
{
	for(int i = 0; i < lines.size(); i++)
	{
		vector<Vec4i> cluster_elements;
		cluster_elements.push_back(lines[i]);
		cluster.push_back(cluster_elements);
		cluster_elements.clear();
	}
	return;
}


void line_clustering(Mat image, vector<Vec4i> lines, vector<Vec4i> &clustered_lines)
{
	if(lines.size() > 1)
	{
		// create clusters
		vector< vector<Vec4i> > cluster;
		initialize_clusters(lines, cluster);

		Mat field = Mat::zeros(image.rows, image.cols, CV_8UC3);

		for( size_t i = 0; i < lines.size(); i++ )
		{
			line( field, Point(lines[i][0], lines[i][1]),
			      Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 1, 8 );
		}
		imshow("lines_after", field);

	}
	else
	{
		return;
	}
}
