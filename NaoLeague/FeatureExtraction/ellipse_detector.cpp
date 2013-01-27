#include <iostream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>
#include "geometry_utils.h"

using namespace std;
using namespace cv;


void detect_ellipse(Mat image, vector<Vec4i> lines)
{
	Mat black = Mat::zeros(image.rows, image.cols, CV_8UC3);
	vector< vector<Vec4i> > ellipse_lines;

	vector<Vec4i> temp;
	temp.push_back(lines[0]);
	ellipse_lines.push_back(temp);
	
	for (int i = 1; i < lines.size(); i++)
	{
		double angle_current = points_angle_360(Point(lines[i][1], lines[i][0]),
	      Point(lines[i][3],lines[i][2]));

		double best_fit = DBL_MAX;
		int cluster_fit = 0;
		for (int j = 0; j < ellipse_lines.size(); j++)
		{
			double angle_current = points_angle_360(Point(lines[j][1], lines[j][0]), Point(lines[j][3],lines[j][2]));
			for (int jj = 0; jj < ellipse_lines[j].size(); jj++)
			{
				double angle_stored = points_angle_360(Point(ellipse_lines[j][jj][1], ellipse_lines[j][jj][0]), Point(ellipse_lines[j][jj][3],ellipse_lines[j][jj][2]));
				double angle_diff = 180 - abs(abs(angle_current - angle_stored) - 180);
				double min_dis = DBL_MAX;
				if(angle_diff < 50){
					for(int j1=0; j1 < 2; j1++ )
					{
						for(int j2=0; j2 < 2; j2++ )
						{
							double temp = points_distance(Point(ellipse_lines[j][jj][2*j1+1], ellipse_lines[j][jj][2*j1]),
								Point(lines[i][2*j2+1], lines[i][2*j2]));
							if(temp < min_dis)
							{
								min_dis = temp;
							}
						}
					}
					double fit = min_dis;
					if(fit < best_fit)
					{
						best_fit = fit;
						cluster_fit = j;
					}
				}
			}
		}
		if(best_fit < 35)
		{
			ellipse_lines[cluster_fit].push_back(lines[i]);
		}
		else
		{
			std::vector<Vec4i> temp_line;
			temp_line.push_back(lines[i]);
			ellipse_lines.push_back(temp_line);
		}
	}

	int cluster = 0;
	for(int i = 0; i < ellipse_lines.size(); i ++)
	{
		cluster ++;
		Mat temp;
		black.copyTo(temp);
		for (int j = 0; j < ellipse_lines[i].size(); j++)
		{
			line( temp, Point(ellipse_lines[i][j][1], ellipse_lines[i][j][0]),
		      Point(ellipse_lines[i][j][3],ellipse_lines[i][j][2]), Scalar(0,0,255), 1, 8 );
		}
		
		stringstream ss;
		ss << cluster;

		imshow("cluster"+ss.str(), temp);
		
	}

	return;
}