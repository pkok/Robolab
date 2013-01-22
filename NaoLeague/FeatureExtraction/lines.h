#ifndef LINES_H
#define LINES_H

#include <iostream>
#include <math.h>
#include <cv.h>
#include <highgui.h>

using namespace cv;

void line_clustering(Mat image);

double similarity_measure(Vec4i line1, Vec4i line2);

double similarity_measure(Vec4i line1, Vec4i line2);

void fit_line(Mat image, Vec4i line, int minimum_datapoints_used, int minimum_iterations, double threshold, int close_data_values, Mat &output_lines, Mat &used_data, double &estimation_error);

void RANSAC_line(vector<Vec2i> points, int minimum_datapoints_used, int iterations, double threshold, int close_data_values, Vec4f &fitted_line, vector<Vec2i> &used_data, double &estimation_error);
#endif
