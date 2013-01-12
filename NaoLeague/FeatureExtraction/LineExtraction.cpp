/* This is a standalone program. Pass an image name as the first parameter
of the program.  Switch between standard and probabilistic Hough transform
by changing "#if 1" to "#if 0" and back */
#include <cv.h>
#include <highgui.h>
#include <math.h>

using namespace cv;

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
    threshold( green, green_thr, 35, 255,0 );

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

int main(int argc, char** argv)
{
    Mat src, dst, color_dst, src_no_backgr;
    if( argc != 2 || !(src=imread(argv[1], 1)).data)
        return -1;
    
    src_no_backgr = backgroung_removal(src);
    
    imshow( "Detected Lines", src );
    waitKey();


    // clock_t startTime = clock();
    // Canny( src, dst, 50, 200, 3 );
    // cvtColor( src, color_dst, CV_GRAY2BGR );

    // vector<Vec4i> lines;
    // HoughLinesP( dst, lines, 1, CV_PI/300, 10, 10, 20 );
    // for( size_t i = 0; i < lines.size(); i++ )
    // {
    //     line( color_dst, Point(lines[i][0], lines[i][1]),
    //         Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 1, 8 );
    // }

    // std::cout << lines.size() << std::endl;
    // namedWindow( "Source", 1 );
    // imshow( "Source", src );

    // namedWindow( "Detected Lines", 1 );
    // imshow( "Detected Lines", color_dst );

    // waitKey(0);
    return 0;
}