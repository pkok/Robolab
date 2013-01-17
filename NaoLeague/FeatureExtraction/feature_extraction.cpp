#include <cv.h>
#include <highgui.h>
#include <math.h>
#include <time.h>

using namespace cv;
using namespace std;

const double HUE_WHITE_THR_LOW = 0.0;
const double HUE_WHITE_THR_HIGH = 0.14;
const double SAT_WHITE_THR_LOW = 0.0;
const double SAT_WHITE_THR_HIGH = 0.2;

const double HUE_YELLOW_THR_LOW = 0.0;
const double HUE_YELLOW_THR_HIGH = 0.14;
const double SAT_YELLOW_THR_LOW = 0.0;
const double SAT_YELLOW_THR_HIGH = 0.14;

const double HUE_ORANGE_THR_LOW = 0.0;
const double HUE_ORANGE_THR_HIGH = 0.14;
const double SAT_ORANGE_THR_LOW = 0.0;
const double SAT_ORANGE_THR_HIGH = 0.14;

const double HUE_GREEN_THR_LOW = 0.0;
const double HUE_GREEN_THR_HIGH = 0.14;
const double SAT_GREEN_THR_LOW = 0.0;
const double SAT_GREEN_THR_HIGH = 0.14;



void remove_background(Mat image, Mat &lines, Mat &posts, Mat &ball){
//	for(int i=0; i < image.rows; i++){
//		for(int j=0; j < image.cols; j++){
//			cout << (double)image.at<cv::Vec3b>(i,j)[1] << endl;
//		}
//	}
	Mat bwy, bwg, bww;
	// yellow
	inRange(image, Scalar(20, 100, 100), Scalar(38, 255, 255), bwy);
	imshow("posts",bwy);
	
	// green
	inRange(image, Scalar(38, 50, 50), Scalar(75, 255, 255), bwg);
	imshow("field",bwg);
}



int main(int argc, char** argv)
{
	clock_t startTime = clock();
	Mat img_rgb, img_hsv;
	Mat img_lines, img_posts, img_ball;
	
	if( argc != 2 || !(img_rgb=imread(argv[1], 1)).data)
		return -1;
	imshow("original", img_rgb);
	
	cvtColor(img_rgb,img_hsv,CV_BGR2HSV);
	remove_background(img_hsv, img_lines, img_posts, img_ball);
	
	std::cout << double( clock() - startTime )*1000 / (double)CLOCKS_PER_SEC<< " ms." << std::endl;
	waitKey(0);
	return 0;
}