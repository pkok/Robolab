#include <iostream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>
#include "lines.h"

using namespace cv;
using namespace std;

#define ANGLE_THR 10

double line_angle(Vec4i line)
{
	double angle = atan2(line[2]-line[0],line[3]-line[1]);
	angle = angle * (180 / CV_PI);
	if( angle < 0 )
		angle += 180;
	return angle;
}

Point line_center(Vec4i line)
{
	int x = line[0] + line[2];
	int y = line[1] + line[3];
	return Point(x/2,y/2);	
}

double points_distance(Point point1, Point point2)
{
	double dx = point2.x - point1.x;
	double dy = point2.y - point1.y;
	double distance = sqrt(dx*dx + dy*dy);
	return distance;
}

double point_line_distance(Point point, Vec4i line)
{
	double normalLength = hypot(line[2] - line[0], line[3] - line[1]);
 	return abs((point.x - line[0]) * (line[3] - line[1]) - (point.y - line[1]) * (line[2] - line[0])) / normalLength;
}

// measure which is used in order to find clusters that contain
// similar lines which are propably belong into the same big line
double similarity_measure(vector<Vec4i> cluster1, vector<Vec4i> cluster2)
{
	double similarity = 0;
	double min_angle_diff = DBL_MAX;
	double points_distance = 0;
	for( int i=0; i < cluster1.size(); i++){
		for( int j=0; j < cluster2.size(); j++){
			for(int ii=0; ii < 2; ii ++){
				points_distance += point_line_distance(Point(cluster2[j][0], cluster2[j][1]), cluster1[i]) + 
				point_line_distance(Point(cluster2[j][2], cluster2[j][3]), cluster1[i]);
			}
			// angle difference between two lines of the two
			// compared clusters
			double angle_diff = abs(line_angle(cluster1[i]) - line_angle(cluster2[j]));
			if(angle_diff <  min_angle_diff){
				min_angle_diff = angle_diff;
			}
		}
	}
	
	similarity = min_angle_diff + 0.3 * points_distance;

	return similarity;
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

void merge_clusters(vector<Vec4i> &cluster_dst, vector<Vec4i> &cluster_src)
{
	for(int i=0; i<cluster_src.size(); i++){
		cluster_dst.push_back(cluster_src[i]);
	}
	for(int j=0; j<cluster_src.size(); j++){
		cluster_src.erase(cluster_src.begin() + j);
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
		Mat black = Mat::zeros(image.rows, image.cols, CV_8UC3);

		double min_similarity;
		int cluster_src;
		int cluster_dst;
		do
		{
			min_similarity = DBL_MAX;
			for( int i  = 0; i < cluster.size(); i++)
			{
				for( int j  = 0; j < cluster.size(); j++)
				{
					if( i != j)
					{
						double similarity = similarity_measure(cluster[i], cluster[j]);
						if( similarity < min_similarity)
						{
							min_similarity = similarity;
							cluster_dst = i;
							cluster_src = j;
						}
					}
				}
			}
			merge_clusters(cluster[cluster_dst], cluster[cluster_src]);
			cluster.erase(cluster.begin() + cluster_src);
			if(cluster.size() == 1)
				break;
		}
		while(min_similarity < 20);

		// if(cluster.size() > 0)
		// {
		// 	for( int i  = 0; i < cluster.size(); i++)
		// 	{

		// 		Mat temp;
		// 		black.copyTo(temp);
		// 		for( int j  = 0; j < cluster[i].size(); j++)
		// 		{
		// 			line( temp, Point(cluster[i][j][0], cluster[i][j][1]),
		// 		      Point(cluster[i][j][2], cluster[i][j][3]), Scalar(0,0,255), 1, 8 );
		// 		}
				
		// 		stringstream ss;
		// 		ss << i;
		// 		imshow("cluster"+ss.str(),temp);
		// 		cout << "cluster" << i << " " << cluster[i].size() << endl;
		// 	}	
		// }
		Mat temp;
				black.copyTo(temp);
		if(cluster.size() > 0)
		{
			for( int i  = 0; i < cluster.size(); i++)
			{	
				double max_length = 0;
				Point point1;
				Point point2;
				for( int j  = 0; j < cluster[i].size(); j++)
				{
					for( int jj  = 0; jj < cluster[i].size(); jj++)
					{
						for( int k  = 0; k < 2; k++)
						{
							for( int p  = 0; p < 2; p++)
							{
								if(points_distance(Point(cluster[i][j][2*k],cluster[i][j][2*k+1]),Point(cluster[i][jj][2*p],cluster[i][jj][2*p+1])) > max_length){
									max_length = points_distance(Point(cluster[i][j][2*k],cluster[i][j][2*k+1]),Point(cluster[i][jj][2*p],cluster[i][jj][2*p+1]));
									point1 = Point(cluster[i][j][2*k],cluster[i][j][2*k+1]);
									point2 = Point(cluster[i][jj][2*p],cluster[i][jj][2*p+1]);
								}
							}
						}
					}
				}
				line( temp, point1,
				      		point2, Scalar(0,0,255), 2, 8 );				
			}
		}
		imshow("lines_awesome", temp);

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
