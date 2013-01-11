/* This is a standalone program. Pass an image name as the first parameter
of the program.  Switch between standard and probabilistic Hough transform
by changing "#if 1" to "#if 0" and back */
#include <cv.h>
#include <math.h>
#include <highgui.h>
#include <iostream>
#include <time.h>

using namespace cv;
//function headers
void drawLine( Mat img, Point start, Point end );
void drawRectangle(Mat img, Point one,Point two,Point three,Point four);
void drawParticle(Mat img,Point location, int rotation,int certainty);

int main(int argc, char** argv)
{
    Mat src, dst, color_dst;
    //if( argc != 2 || !(src=imread(argv[1], 0)).data)
    //    return -1;

    //Canny( src, dst, 150, 200, 3 );
    //cvtColor( dst, color_dst, CV_GRAY2BGR );

	//image – 8-bit, single-channel binary source image. The image may be modified by the function.
	//lines – Output vector of lines. Each line is represented by a 4-element vector   , where   and   are the ending points of each detected line segment.
	//rho – Distance resolution of the accumulator in pixels.
	//theta – Angle resolution of the accumulator in radians.
	//threshold – Accumulator threshold parameter. Only those lines are returned that get enough votes (  ).
	//minLineLength – Minimum line length. Line segments shorter than that are rejected.
	//maxLineGap – Maximum allowed gap between points on the same line to link them.

    /*vector<Vec4i> lines;
    HoughLinesP( dst, lines, 1, CV_PI/180, 10, 20, 20 );
    for( size_t i = 0; i < lines.size(); i++ )
    {
        line( color_dst, Point(lines[i][0], lines[i][1]),
            Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 2, 8 );
    }

	printf("%d",lines.size());
    namedWindow( "Source", 1 );*/



	//1 pxl = 1 cm
	int h = 540;
	int w = 740;



	//rectangles
	//x1,y1,x2,y2..
	int outer_line[] = {70,70,
			670,70,
			670,470,
			70,470};
	
	int goal_line_1[] = {70,160,
			130,160,
			130,380,
			70,380};
	
	int goal_line_2[] = {610,160,
			670,160,
			670,380,
			610,380};	
	
	//lines
	int middle_line[] = {370,70,
			370,470};

	//points:
	//x,y,diameter
	int penalty1[] = {250,270,10};
	int penalty2[] = {490,270,10};
	
	int goalpost11[] = {70,200,10};
	int goalpost12[] = {70,350,10};
	int goalpost21[] = {670,200,10};
	int goalpost22[] = {670,350,10};
	//circles:
	//x,y,diameter
	int middle_circle[] = {370,270,120};


	//create empty image container:

	Mat field_img = Mat::zeros( h, w, CV_8UC3 );
	
	//fill with grass

	for (int x = 1; x < w*3; x=x+3)
	{
		for (int y = 0; y< h; y++)
		{
			field_img.at<uchar>(y,x) = 230;
			//Set2D(field_img, x, y, CvScalar(0,0,255)0);
		}
	}


	//////draw field:

	// most outward line
	drawRectangle(field_img,Point(outer_line[0],outer_line[1]),Point(outer_line[2],outer_line[3]),Point(outer_line[4],outer_line[5]),Point(outer_line[6],outer_line[7]));

	// goal line 1	
	drawRectangle(field_img,Point(goal_line_1[0],goal_line_1[1]),Point(goal_line_1[2],goal_line_1[3]),Point(goal_line_1[4],goal_line_1[5]),Point(goal_line_1[6],goal_line_1[7]));
	
	// goal line 2	
	drawRectangle(field_img,Point(goal_line_2[0],goal_line_2[1]),Point(goal_line_2[2],goal_line_2[3]),Point(goal_line_2[4],goal_line_2[5]),Point(goal_line_2[6],goal_line_2[7]));

	// middle line
	drawLine(field_img,Point(middle_line[0],middle_line[1]),Point(middle_line[2],middle_line[3]));
	
	//middle circle  circle( img, center, w/32, Scalar( 0, 0, 255 ), thickness, lineType );
	circle(field_img, Point(middle_circle[0],middle_circle[1]), middle_circle[2]/2, Scalar(255,255,255),2,8);

	//penalty point 1,2
	circle(field_img,Point(penalty1[0],penalty1[1]),penalty1[2]/2,Scalar(255,255,255),-1,8);
	circle(field_img,Point(penalty2[0],penalty2[1]),penalty2[2]/2,Scalar(255,255,255),-1,8);


	//goal post1:
	circle(field_img,Point(goalpost11[0],goalpost11[1]),goalpost11[2]/2,Scalar(0,255,255),-1,8);
	circle(field_img,Point(goalpost12[0],goalpost12[1]),goalpost12[2]/2,Scalar(0,255,255),-1,8);
	//goal post2:
	circle(field_img,Point(goalpost21[0],goalpost21[1]),goalpost21[2]/2,Scalar(0,255,255),-1,8);
	circle(field_img,Point(goalpost22[0],goalpost22[1]),goalpost22[2]/2,Scalar(0,255,255),-1,8);



	Mat current_frame;
	
	int rot = 0;

			namedWindow("Simulator", 1);
	for(int frame_no = 0; frame_no<10; frame_no++)
	{
		field_img.copyTo(current_frame);
		
		for(int n = 0; n<10;n++)
		{
			std::cout << "creating background image" << std::endl;
			int random_x = 20+rand()%700;
			int random_y = 20+rand()%500;
			double random_rot = rand()%630/360;
			int random_certainty = rand()%100;
			drawParticle(current_frame,Point(random_x,random_y),random_rot,random_certainty);

		}	

			imshow("Simulator",current_frame);
			waitKey(0);
	}


	/*for(int x = 100;x<300;x = x+10)
	{
		for(int y= 100;y < 300; y = y+10)
		{
			rot++;
			drawParticle(field_img,Point(x,y),rot);
		}
	}*/

	//drawLine(field_img,Point(100,100),Point(200,200));
	//show image

    //field_img = cvCreateMat(Size(320,240),CV_8UC3);
    //imshow( "Source", src );
	
    //namedWindow( "Detected Lines", 1 );
    //imshow( "Detected Lines", color_dst );




    return 0;
}
/**
 * @function drawParticle
 * @brief draws a particle of a robot
**/
void drawParticle(Mat img,Point location, int angle, int certainty)
{
	//orientation line:
	int x = 10;
	int y = 0;
	//rotate point around 0,0
	double sn = sin(angle);
	double cs = cos(angle);
	double inv_certainty =1.0- (double)certainty/100.0;
	int px = cs*x - sn*y;
	int py = sn*x + cs*y;
	
	//draw circle
	circle(img,location,5,Scalar(0,230*inv_certainty,0),-1,8);
	//draw direction
	line(img,location,Point(location.x+px,location.y+py),Scalar(0,0,255),1,8);
}
/**
 * @function drawRectangle
 * @brief Draws simple rectangle
**/
void drawRectangle(Mat img, Point one,Point two,Point three,Point four)
{
	drawLine(img,one,two);
	drawLine(img,two,three);
	drawLine(img,three,four);
	drawLine(img,four,one);
}


/**
 * @function drawLine
 * @brief Draw a simple line
 */
void drawLine( Mat img, Point start, Point end )
{
  int thickness = 2;
  int lineType = 8;
  line( img,
    start,
    end,
    Scalar( 255, 255, 255 ),
    thickness,
    lineType );
}
