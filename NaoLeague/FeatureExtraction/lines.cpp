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
	for( int i=0; i < cluster1.size(); i++){
		for( int j=0; j < cluster2.size(); j++){

			// angle difference between two lines of the two
			// compared clusters
			double angle_diff = abs(line_angle(cluster1[i]) - line_angle(cluster2[j]));
			if(angle_diff <  min_angle_diff){
				min_angle_diff = angle_diff;
			}
		}
	}

	// double most_distant_points = 0;
	// double most_distant_points_angle = 0;
	// for( int i=0; i < cluster1.size(); i++){
	// 	for( int j=0; j < cluster2.size(); j++){
	// 		for(int ii=0; ii < 2; ii ++){
	// 			for(int jj=0; jj < 2; jj ++){
	// 				double distance = points_distance(Point(cluster1[i][2*ii], cluster1[i][2*ii+1]),
	// 					Point(cluster1[j][2*jj], cluster1[j][2*jj+1]));
	// 				if( distance > most_distant_points){
	// 					most_distant_points = distance;
	// 					most_distant_points_angle = abs(line_angle(cluster1[i]) - line_angle(Vec4i(cluster1[i][2*ii],
	// 						cluster1[i][2*ii+1], cluster2[j][2*jj], cluster2[j][2*jj+1])));
	// 				}
	// 			}
	// 		}
	// 	}
	// }

	// if(min_angle_diff < 5){
	// 	similarity = most_distant_points*most_distant_points_angle;
	// }else{
	similarity = min_angle_diff;
	// }
	
	
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

		double d = point_line_distance(Point(1,0),Vec4i(0,0,10,10));
		cout << d << endl;
		d = point_line_distance(Point(1,0),Vec4i(2,0,12,10));
		cout << d << endl;
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
		}
		while(min_similarity < 2);

		if(cluster.size() > 0)
		{
			for( int i  = 0; i < cluster.size(); i++)
			{

				Mat temp;
				black.copyTo(temp);
				for( int j  = 0; j < cluster[i].size(); j++)
				{
					line( temp, Point(cluster[i][j][0], cluster[i][j][1]),
				      Point(cluster[i][j][2], cluster[i][j][3]), Scalar(0,0,255), 1, 8 );
				}
				
				stringstream ss;
				ss << i;
				imshow("cluster"+ss.str(),temp);
				cout << "cluster" << i << " " << cluster[i].size() << endl;
			}	
		}

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
