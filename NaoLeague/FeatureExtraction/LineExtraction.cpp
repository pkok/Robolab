/* This is a standalone program. Pass an image name as the first parameter
of the program.  Switch between standard and probabilistic Hough transform
by changing "#if 1" to "#if 0" and back */
#include <cv.h>
#include <math.h>
#include <highgui.h>
#include <iostream>
#include <time.h>

using namespace cv;

int main(int argc, char** argv)
{
    Mat src, dst, color_dst;
    if( argc != 2 || !(src=imread(argv[1], 0)).data)
        return -1;

    Canny( src, dst, 50, 200, 3 );
    cvtColor( dst, color_dst, CV_GRAY2BGR );

	//image – 8-bit, single-channel binary source image. The image may be modified by the function.
	//lines – Output vector of lines. Each line is represented by a 4-element vector   , where   and   are the ending points of each detected line segment.
	//rho – Distance resolution of the accumulator in pixels.
	//theta – Angle resolution of the accumulator in radians.
	//threshold – Accumulator threshold parameter. Only those lines are returned that get enough votes (  ).
	//minLineLength – Minimum line length. Line segments shorter than that are rejected.
	//maxLineGap – Maximum allowed gap between points on the same line to link them.

    vector<Vec4i> lines;
    HoughLinesP( dst, lines, 1, CV_PI/180, 80, 30, 10 );
    for( size_t i = 0; i < lines.size(); i++ )
    {
        line( color_dst, Point(lines[i][0], lines[i][1]),
            Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
    }

    namedWindow( "Source", 1 );
    imshow( "Source", src );

    namedWindow( "Detected Lines", 1 );
    imshow( "Detected Lines", color_dst );

    waitKey(0);
    return 0;
}
