#include <cv.h>
#include <highgui.h>
#include <math.h>
#include <time.h>
using namespace cv;

int GREEN_THRESHOLD = 35;
int BINARY_TYPE = 0;
int MAX_BINARY_VALUE = 255;
double LINE_CONN_THERSHOLD = 0.7;
double LINE_ANGLE_THERSHOLD = 15.0;
double YEL_THR_R = 0.4;
double YEL_THR_B = 0.2;
double YEL_THR_G = 0.4;
const double PI  =3.141592653589793238462;

Mat remove_background(Mat image){
    Mat green = cvCreateMat(image.rows, image.cols, CV_8U);
    int count;
    for(int j = 0; j < image.cols; j ++){
        count = 0;
        bool background = true;
        for(int i = 0; i < image.rows; i ++){
            if (background){
                double jr = (double)image.at<cv::Vec3b>(i,j)[2];
                double jg = (double)image.at<cv::Vec3b>(i,j)[1];
                double jb = (double)image.at<cv::Vec3b>(i,j)[0];
                double justGreen = jg - jr/2 - jb/2;
                if(justGreen < 0)
                    justGreen = 0;
                
                (green.data + green.step * i)[j] = 255;
                if (justGreen > 34){
                    double r = ((double)image.at<cv::Vec3b>(i,j)[2] / 
                        ((double)image.at<cv::Vec3b>(i,j)[0]+(double)image.at<cv::Vec3b>(i,j)[1]+(double)image.at<cv::Vec3b>(i,j)[2]));
                    double g = ((double)image.at<cv::Vec3b>(i,j)[1] / 
                        ((double)image.at<cv::Vec3b>(i,j)[0]+(double)image.at<cv::Vec3b>(i,j)[1]+(double)image.at<cv::Vec3b>(i,j)[2]));
                    double b = ((double)image.at<cv::Vec3b>(i,j)[0] / 
                        ((double)image.at<cv::Vec3b>(i,j)[0]+(double)image.at<cv::Vec3b>(i,j)[1]+(double)image.at<cv::Vec3b>(i,j)[2]));
                    if(r>0.38 && g>0.38 && b<0.2){
                        (green.data + green.step * i)[j] = 0;
                    }else if(image.at<cv::Vec3b>(i,j)[2]>230 && image.at<cv::Vec3b>(i,j)[1]>230 && image.at<cv::Vec3b>(i,j)[0]<160){
                        (green.data + green.step * i)[j] = 0;
                    }else{
                        (green.data + green.step * i)[j] = 255;
                    }
                }else{
                    (green.data + green.step * i)[j] = 0;
                }
                if((green.data + green.step * i)[j] > 0){
                    count ++;
                    image.at<cv::Vec3b>(i,j)[0] = 0;
                    image.at<cv::Vec3b>(i,j)[1] = 0;
                    image.at<cv::Vec3b>(i,j)[2] = 0;
                    if(count > 5)
                        background = false;
                }else{
                    image.at<cv::Vec3b>(i,j)[0] = 0;
                    image.at<cv::Vec3b>(i,j)[1] = 0;
                    image.at<cv::Vec3b>(i,j)[2] = 0;
                }
            }
            else
            {
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
    }
    return image;
}

double distance_point_line(int cx, int cy, int ax, int ay, int bx, int by){

    double r_numerator = (cx-ax)*(bx-ax) + (cy-ay)*(by-ay);
    double r_denomenator = (bx-ax)*(bx-ax) + (by-ay)*(by-ay);
    double r = r_numerator / r_denomenator;
    double px = ax + r*(bx-ax);
    double py = ay + r*(by-ay);     
    double s =  ((ay-cy)*(bx-ax)-(ax-cx)*(by-ay) ) / r_denomenator;
    double distanceLine;

    distanceLine = fabs(s)*sqrt(r_denomenator);

    return distanceLine;
}

double angle_bet_lines(int a1x, int a1y, int b1x, int b1y, int a2x, int a2y, int b2x, int b2y){

    double angle1 = atan2(a1y - b1y, a1x - b1x);
    double angle2 = atan2(a2y - b2y, a2x - b2x);
    double res = (angle1-angle2) * 180 / PI;
    return abs(res); 
}


int main(int argc, char** argv)
{
    Mat src, dst, color_dst, src_no_backgr;
    Mat src_line_binary;
    if( argc != 2 || !(src=imread(argv[1], 1)).data)
        return -1;
    
    // Measuring performance
    clock_t startTime = clock();

    src_line_binary = remove_background(src);
    
    Canny( src_line_binary, dst, 50, 200, 3 );
    cvtColor( dst, color_dst, CV_GRAY2BGR );

    vector<Vec4i> lines;
    HoughLinesP( dst, lines, 1, CV_PI/300, 10, 20, 20 );

    for( size_t i = 0; i < lines.size(); i++ ){
        for( size_t j = 0; j < lines.size(); j++ ){
            if(i != j){
                for( size_t p = 0; p < 2; p++ ){
                    if(distance_point_line(lines[i][2*p], lines[i][2*p+1], lines[j][0], lines[j][1], lines[j][2], lines[j][3]) < LINE_CONN_THERSHOLD){
                        if(angle_bet_lines(lines[j][0], lines[j][1], lines[j][2], lines[j][3],lines[i][0], lines[i][1], lines[i][2], lines[i][3]) > LINE_ANGLE_THERSHOLD){
                            circle(color_dst, Point(lines[i][2*p], lines[i][2*p+1]), 5, Scalar(0,255,0), 1, 8, 0);
                        }
                    }
                }   
            }
        }
    }

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
