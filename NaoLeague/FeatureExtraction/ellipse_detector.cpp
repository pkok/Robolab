#include <iostream>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>
#include "geometry_utils.h"

using namespace std;
using namespace cv;


void detect_ellipse(Mat image, vector<Vec4i> lines, vector<Vec4i> &ellipse_prob_lines)
{
	Mat black = Mat::zeros(image.rows, image.cols, CV_8UC3);
	vector< vector<Vec4i> > ellipse_lines;

	bool added;
	while(lines.size() != 0)
	{
		vector<Vec4i> temp;
		temp.push_back(lines[0]);
		lines.erase(lines.begin() + 0);
		do
		{
			added = false;
			double min_angle = DBL_MAX;
			double min_dis = DBL_MAX;
			int best_line = 0;
			for (int i = 0; i < lines.size(); i++)
			{
				double best_angle = DBL_MAX;
				double best_dis = DBL_MAX;
				for (int j = 0; j < temp.size(); j++)
				{
					double angle_diff;
					Point* inters = intersection(temp[j], lines[i], image);
					double min_temp = DBL_MAX;
					if(inters != NULL)
					{
						double min_dis1 = DBL_MAX;
						double min_dis2 = DBL_MAX;
						int temp_close = 0;
						int stored_close = 0;
						Point intersect = Point(inters->x, inters->y);
						for(int j2=0; j2 < 2; j2++ )
						{
							double temp_dis1 = points_distance( Point(inters->x, inters->y), Point(temp[j][2*j2], temp[j][2*j2+1]));
							double temp_dis2 = points_distance( Point(inters->x, inters->y), Point(lines[i][2*j2], lines[i][2*j2+1]));
							if(temp_dis1 < min_dis1)
							{
								min_dis1 = temp_dis1;
								temp_close = j2;
							}
							if(temp_dis2 < min_dis2)
							{
								min_dis2 = temp_dis2;
								stored_close = j2;
							}
						}
						double angle_current = points_angle_360(Point(temp[j][2*temp_close], temp[j][2*temp_close+1]), line_middle_point(Vec4i(temp[j][0], temp[j][1],temp[j][2],temp[j][3])));
						double angle_stored = points_angle_360(line_middle_point(Vec4i(lines[i][0], lines[i][1],lines[i][2],lines[i][3])),Point(lines[i][2*stored_close], lines[i][2*stored_close+1]));
						angle_diff = 180 - abs(abs(angle_current - angle_stored) - 180);
						double angle_ratio = angle_diff /180;
						min_temp = 20 * angle_ratio + (min_dis2 + min_dis1);
					}
					else
					{
						min_temp = DBL_MAX;
					}
					if(min_temp < best_dis)
					{
						best_dis = min_temp;
					}
				}
				if(best_dis < min_dis)
				{
					min_dis = best_dis;
					best_line = i;
				}
			}
			if(min_dis < 25)
			{
				added = true;
				temp.push_back(lines[best_line]);
				lines.erase(lines.begin() + best_line);
			}
		}
		while(added);
		ellipse_lines.push_back(temp);
		temp.clear();
	}

	int cluster = 0;
	int max_cluster_size = 0;
	for(int i = 0; i < ellipse_lines.size(); i ++)
	{
		if (ellipse_lines[i].size() > max_cluster_size)
		{
			max_cluster_size = ellipse_lines[i].size();
		}
	}
	if(max_cluster_size > 7)
	{
		for (int j = 0; j < ellipse_lines[cluster].size(); j++)
		{
			ellipse_prob_lines.push_back(ellipse_lines[cluster][j]);
		}
	}
	return;
}

// cout << "cluster " << i << "  size " << ellipse_lines[i].size() << endl;
// cluster ++;
// Mat temp;
// black.copyTo(temp);
// for (int j = 0; j < ellipse_lines[i].size(); j++)
// {
// 	circle(temp, Point(ellipse_lines[i][j][1], ellipse_lines[i][j][0]), 2, Scalar(0,0,255), 2, 8, 0);
// 	circle(temp, Point(ellipse_lines[i][j][3],ellipse_lines[i][j][2]), 2, Scalar(0,0,255), 2, 8, 0);
// 	circle(temp, line_middle_point(Vec4i(ellipse_lines[i][j][1], ellipse_lines[i][j][0],ellipse_lines[i][j][3], ellipse_lines[i][j][2])), 2, Scalar(0,0,255), 2, 8, 0);
// 	// line( temp, Point(ellipse_lines[i][j][1], ellipse_lines[i][j][0]),
// 	//      Point(ellipse_lines[i][j][3],ellipse_lines[i][j][2]), Scalar(0,0,255), 1, 8 );
// }
// stringstream ss;
// ss << cluster;
// imshow("cluster"+ss.str(), temp);
