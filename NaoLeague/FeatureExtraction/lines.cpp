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
	double angle = atan2(line[2]-line[0],line[3]-line[1]);
	angle = angle * (180 / CV_PI);
	if( angle < 0 )
		angle += 180;
	return angle;
}


double similarity_measure(Vec4i line1, Vec4i line2)
{
	return abs(line_angle(line1) - line_angle(line2));
}

void initialize_clusters(vector<Vec4i> lines, vector< vector<Vec4i> > &cluster)
{
	for(int i = 0; i < lines.size(); i++)
	{
		vector<Vec4i> cluster_elements;
		cluster_elements.push_back(lines[i]);
		line_angle(Vec4i(0,0,-10,-10));
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
		Mat black = Mat::zeros(image.rows, image.cols, CV_8UC3);

		bool end = false;

		do
		{
			end = true;
			for( int i  = 0; i < cluster.size(); i++)
			{
				for( int j  = 0; j < cluster.size(); j++)
				{
					if( i != j)
					{
						for( int ii  = 0; ii < cluster[i].size(); ii++)
						{
							for( int jj  = 0; jj < cluster[j].size(); jj++)
							{
								if( similarity_measure(cluster[i][ii], cluster[j][jj]) < 10)
								{
									end = false;
									cluster[i].push_back(cluster[j][jj]);
									cluster[j].erase(cluster[j].begin() + jj);									
								}
							}
						}
					}
				}
			}
		}
		while(!end);

		for( int i  = 0; i < cluster.size(); i++)
		{
			if(cluster.size() > 0)
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
