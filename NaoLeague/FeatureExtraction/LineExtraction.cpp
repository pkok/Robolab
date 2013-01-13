#include <cv.h>
#include <highgui.h>
#include <math.h>
#include <time.h>
using namespace cv;

int GREEN_THRESHOLD = 35;
int BINARY_TYPE = 0;
int MAX_BINARY_VALUE = 255;

Mat extract_green(Mat image){
    Mat green = cvCreateMat(image.rows, image.cols, CV_8U);
    for(int i = 0; i < image.rows; i ++){
        for(int j = 0; j < image.cols; j ++){
            (green.data + green.step * i)[j] = max(image.at<cv::Vec3b>(i,j)[1]
            -image.at<cv::Vec3b>(i,j)[2]/2-image.at<cv::Vec3b>(i,j)[0]/2,0);
        }
    }
    return green;
}

Mat backgroung_removal(Mat image){
    Mat green_thr;
    Mat green = extract_green(image);
    threshold( green, green_thr, GREEN_THRESHOLD, MAX_BINARY_VALUE, BINARY_TYPE );

    // this for loop can be done in the above for loop.
    // I leave it like this for now.
    for(int j = 0; j < image.cols; j ++){
        for(int i = 0; i < image.rows; i ++){
            if((green_thr.data + green_thr.step * i)[j] > 0){
                break;
            }else{
                image.at<cv::Vec3b>(i,j)[0] = 0;
                image.at<cv::Vec3b>(i,j)[1] = 0;
                image.at<cv::Vec3b>(i,j)[2] = 0;
            }
        }
    }
    return image;
}

Mat line_binarization(Mat image){
	//this for loop can be handle in the same
	//as the above functions, for now we do it 
	//step by step.
	for(int j = 0; j < image.cols; j ++){
        for(int i = 0; i < image.rows; i ++){
			//if almost white make it white...
			if(image.at<cv::Vec3b>(i,j)[0] > 200 &&
            image.at<cv::Vec3b>(i,j)[1] > 200 &&
            image.at<cv::Vec3b>(i,j)[2] > 200){
				image.at<cv::Vec3b>(i,j)[0] = 255;
				image.at<cv::Vec3b>(i,j)[1] = 255;
				image.at<cv::Vec3b>(i,j)[2] = 255;
			// else make it black...
			}else{
				image.at<cv::Vec3b>(i,j)[0] = 0;
				image.at<cv::Vec3b>(i,j)[1] = 0;
				image.at<cv::Vec3b>(i,j)[2] = 0;
			}
		}
	}
	return image;	
}


int main(int argc, char** argv)
{
    Mat src, dst, color_dst, src_no_backgr;
    Mat src_line_binary;
    if( argc != 2 || !(src=imread(argv[1], 1)).data)
        return -1;
    
    // Measuring performance
    clock_t startTime = clock();

    src_no_backgr = backgroung_removal(src);
    
    src_line_binary = line_binarization(src_no_backgr);

    namedWindow( "Detected Lines", 1 );
    imshow( "Detected Lines", src_line_binary );

    
    Canny( src_line_binary, dst, 50, 200, 3 );
    cvtColor( dst, color_dst, CV_GRAY2BGR );

    vector<Vec4i> lines;
    HoughLinesP( dst, lines, 1, CV_PI/300, 10, 10, 20 );
    for( size_t i = 0; i < lines.size(); i++ )
    {
        line( color_dst, Point(lines[i][0], lines[i][1]),
            Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 1, 8 );
    }

    std::cout << double( clock() - startTime )*1000 / (double)CLOCKS_PER_SEC<< " ms." << std::endl;
    std::cout << lines.size() << std::endl;
    namedWindow( "Detected Lines", 1 );
    imshow( "Detected Lines", color_dst );

    waitKey(0);
    return 0;
}
