#include <cv.h>
#include <math.h>
#include <highgui.h>
#include <iostream>
#include <time.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>


#include "LocationVisualizer.h"


using namespace cv;
using namespace std;
//function headers


/*
 * constructor
 *
 */
LocationVisualizer::LocationVisualizer(){
	//read parameter
	read_file(&h, &w, outer_line, goal_line_1, goal_line_2, middle_line, penalty1, penalty2, goalpost11, goalpost12, goalpost21, goalpost22, middle_circle);
	//draw parameters on field
	draw_field();
	draw_field_lines();
}
LocationVisualizer::~LocationVisualizer(){
}

/*
 * draws lines on the field
 */
void LocationVisualizer::draw_field_lines(){
	//////draw field:

	// most outward line
	draw_rectangle(field_img,Point(outer_line[0],outer_line[1]),Point(outer_line[2],outer_line[3]),Point(outer_line[4],outer_line[5]),Point(outer_line[6],outer_line[7]));

	// goal line 1
	draw_rectangle(field_img,Point(goal_line_1[0],goal_line_1[1]),Point(goal_line_1[2],goal_line_1[3]),Point(goal_line_1[4],goal_line_1[5]),Point(goal_line_1[6],goal_line_1[7]));

	// goal line 2
	draw_rectangle(field_img,Point(goal_line_2[0],goal_line_2[1]),Point(goal_line_2[2],goal_line_2[3]),Point(goal_line_2[4],goal_line_2[5]),Point(goal_line_2[6],goal_line_2[7]));

	// middle line
	draw_line(field_img,Point(middle_line[0],middle_line[1]),Point(middle_line[2],middle_line[3]));

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

	field_img.copyTo(current_frame);
}
/*
 * creates new matrix with lines read from read_file()
 */
void LocationVisualizer::draw_field(){
	//create empty image container:

	field_img = Mat::zeros( h, w, CV_8UC3 );

	//fill with grass

	for (int x = 1; x < w*3; x=x+3)
	{
		for (int y = 0; y< h; y++)
		{
			field_img.at<uchar>(y,x) = 230;
		}
	}
	this->draw_field_lines();
}
/*
 * creates window with height and size from current frame
 */
void LocationVisualizer::create_window(){

	imshow("Simulator",current_frame);
	waitKey(0);
}

/*
 * converts from soccer field coordinat system to image coordinate system
 */
void LocationVisualizer::convert_to_image_coordinates(int* x,int* y)
{
	int helper = *x;
	*x = *x + 370;
	*y = *y + 270;
}

/**
 * @function readFile
 * @brief reads parameter.txt 
**/
void LocationVisualizer::read_file(int* h, int* w, int outer_line[], int goal_line1[], int goal_line2[],int middle_line[], int penalty1[], int penalty2[], int goalpost11[], int goalpost12[], int goalpost21[], int goalpost22[], int middle_circle[])
{
	std::ifstream infile("parameter.txt");
	std::string line;
	std::string token;
	
	//keep track which parameters are loaded
	int object_counter = 0;

	while (std::getline(infile, line))
	{
	    std::istringstream iss(line);
		//dont read files with // at beginning
		if(line.compare(0,1,"/") != 0)
		{

			//read all parameters
			switch ( object_counter )
		      	{
			 case 0: //h
				std::cout << "h = " << line << std::endl;
			    *h = atoi(line.c_str()); 
			    break;
			 case 1: //w
			    *w = atoi(line.c_str());
				std::cout << "w = " << line << std::endl;
			    break;
			case 2: //outer_line[8];
				{	
					std::cout << "outer line "<< line;
					outer_line[0] = atoi(line.c_str());
					for (int i = 1; i<8; i++)
					{
						std::getline(infile,line);
						std::istringstream iss(line);
						outer_line[i] = atoi(line.c_str());
						std::cout << " "<< line;
					}				
					std::cout<<std::endl;
				break;
				}
			case 3: //goal_line1[8];
				{		std::cout << "goal_line2 "<< line;
					goal_line1[0] = atoi(line.c_str());
					for (int i = 1; i<8; i++)
					{
						std::getline(infile,line);
						std::istringstream iss(line);
						goal_line1[i] = atoi(line.c_str());std::cout << " "<< line;
					}				std::cout<<std::endl;
				break;
				}
			case 4: //goal_line2[8];
				{		std::cout << "goal_line2 "<< line;
					goal_line2[0] = atoi(line.c_str());
					for (int i = 1; i<8; i++)
					{
						std::getline(infile,line);
						std::istringstream iss(line);
						goal_line2[i] = atoi(line.c_str());std::cout << " "<< line;
					}	std::cout<<std::endl;			
				break;
				}
			case 5: //middle_line[4];
				{	std::cout << "middle line "<< std::endl;
					std::cout <<  line<<std::endl;	
					middle_line[0] = atoi(line.c_str());			
					for (int i = 1; i<4; i++)
					{
						std::getline(infile,line);
						std::istringstream iss(line);
						middle_line[i] = atoi(line.c_str());
						std::cout <<  line<<std::endl;		
					}				
				break;
				}		
			case 6: //penalty1[3];
				{	std::cout << "penalty1 "<< std::endl;
					std::cout <<  line<<std::endl;	
					penalty1[0] = atoi(line.c_str());			
					for (int i = 1; i<3; i++)
					{
						std::getline(infile,line);
						std::istringstream iss(line);
						penalty1[i] = atoi(line.c_str());

						std::cout <<  line<<std::endl;			
					}				
				break;
				}		
			case 7: //penalty2[3];
				{	
					std::cout << "penalty2 "<< std::endl;
					std::cout <<  line<<std::endl;	
					penalty2[0] = atoi(line.c_str());			
					for (int i = 1; i<3; i++)
					{
						std::getline(infile,line);
						std::istringstream iss(line);
						penalty2[i] = atoi(line.c_str());	
						std::cout <<  line<<std::endl;		
					}				
				break;
				}
			case 8: //goalpost11[3];
				{	
					std::cout << "goalpost11 "<< std::endl;
					std::cout <<  line<<std::endl;	
					goalpost11[0] = atoi(line.c_str());			
					for (int i = 1; i<3; i++)
					{
						std::getline(infile,line);
						std::istringstream iss(line);
						goalpost11[i] = atoi(line.c_str());
						std::cout <<  line<<std::endl;		
					}				
				break;
				}			
			case 9: //goalpost12[3];
				{	
					std::cout << "goalpost12 "<< std::endl;
					std::cout <<  line<<std::endl;
					goalpost12[0] = atoi(line.c_str());			
					for (int i = 1; i<3; i++)
					{
						std::getline(infile,line);
						std::istringstream iss(line);
						goalpost12[i] = atoi(line.c_str());
						std::cout <<  line<<std::endl;		
					}				
				break;
				}
			case 10: //goalpost21[3];
				{	
					std::cout << "goalpost21 "<< std::endl;
					std::cout <<  line<<std::endl;
					goalpost21[0] = atoi(line.c_str());			
					for (int i = 1; i<3; i++)
					{
						std::getline(infile,line);
						std::istringstream iss(line);
						goalpost21[i] = atoi(line.c_str());	
						std::cout <<  line<<std::endl;	
					}				
				break;
				}			
			case 11://goalpost22[3];
				{	
					std::cout << "goalpost22 "<< std::endl;
					std::cout <<  line<<std::endl;
					goalpost22[0] = atoi(line.c_str());			
					for (int i = 1; i<3; i++)
					{
						std::getline(infile,line);
						std::istringstream iss(line);
						goalpost22[i] = atoi(line.c_str());
					std::cout <<  line<<std::endl;		
					}				
				break;
				}		
			case 12://middle_circle[3];
				{	
					std::cout << "middle_circle "<< std::endl;
					std::cout <<  line<<std::endl;
					middle_circle[0] = atoi(line.c_str());			
					for (int i = 1; i<3; i++)
					{
						std::getline(infile,line);
						std::istringstream iss(line);
						middle_circle[i] = atoi(line.c_str());	
						std::cout <<  line<<std::endl;	
					}				
				break;
				}
			 default:
				break;
			}
			object_counter++;	
		}
	}
	this->convert_all();
}
/*
 * converts all coordinates in window coordinates (of field variables)
 */
void LocationVisualizer::convert_all(){
	//rectangles

	//outer_line
	for(int i = 0; i< 8; i+=2){

		//cout<<"before:"<<outer_line[i]<<" "<<outer_line[i+1]<<endl;
		convert_to_image_coordinates(&outer_line[i],&outer_line[i+1]);
		//cout<<"after:"<<outer_line[i]<<" "<<outer_line[i+1]<<endl;
	}

	//goal_line_1[8];
	for(int i = 0; i< 8; i+=2){
			convert_to_image_coordinates(&goal_line_1[i],&goal_line_1[i+1]);
		}
	// goal_line_2[8];
	for(int i = 0; i< 8; i+=2){
			convert_to_image_coordinates(&goal_line_2[i],&goal_line_2[i+1]);
		}
	//lines

	//int middle_line[4];
	for(int i = 0; i< 4; i+=2){
				convert_to_image_coordinates(&middle_line[i],&middle_line[i+1]);
			}


	//points
	//int penalty1[3];
	convert_to_image_coordinates(&penalty1[0],&penalty1[1]);

	//int penalty2[3];
	convert_to_image_coordinates(&penalty2[0],&penalty2[1]);

	//int goalpost11[3];
	convert_to_image_coordinates(&goalpost11[0],&goalpost11[1]);
	//int goalpost12[3];
	convert_to_image_coordinates(&goalpost12[0],&goalpost12[1]);
	//int goalpost21[3];
	convert_to_image_coordinates(&goalpost21[0],&goalpost21[1]);
	//int goalpost22[3];
	convert_to_image_coordinates(&goalpost22[0],&goalpost22[1]);

	//int middle_circle[3];
	convert_to_image_coordinates(&middle_circle[0],&middle_circle[1]);
}
/*
 * copies the field image over displayed image.
 */
void LocationVisualizer::clear_buffer(){
	field_img.copyTo(current_frame);
}
/**
 * @function draw_particle
 * @brief draws all particles in list given
 */
void LocationVisualizer::draw_particle(vector<Particle> p_vec){

	//find max
	double max = -100000;
	for (int i = 0; i< p_vec.size(); i++){
		if(p_vec[i].weight > max){
			max = p_vec[i].weight;
		}
	}


	for(int i = 0; i < p_vec.size(); i++){
		int x = p_vec[i].x;
		int y = p_vec[i].y;
		//convert coordinates:
		convert_to_image_coordinates(&x,&y);
		double rot = p_vec[i].rot;
		double prob = (double)p_vec[i].weight/max;
		this->draw_particle(Point(x,y),rot,prob);
	}
	/*
	while(p_ptr != 0)
	{
		int x = p_ptr->x;
		int y = p_ptr->y;
		//convert coordinates:
		convert_to_image_coordinates(&x,&y);
		double rot = p_ptr->rot;
		double prob = p_ptr->weight;
		this->draw_particle(Point(x,y),rot,prob);
		p_ptr = p_ptr->next;
	}
*/
}
/**
 * @function draw_particle
 * @brief draws a particle of a robot
**/
void LocationVisualizer::draw_particle(Point location, double angle, double certainty)
{
	//cout<<"drawing particle"<<endl;
	//orientation line:
	int x = 10;
	int y = 0;
	//rotate point around 0,0
	double sn = sin(angle);
	double cs = cos(angle);
	double inv_certainty =1.0 - certainty;
	//cout<<"inv_certainty "<<inv_certainty<<endl;
	int px = cs*x - sn*y;
	int py = sn*x + cs*y;
	
	//draw circle
	circle(this->current_frame,location,5,Scalar(0,230*inv_certainty,0),-1,8);
	//draw direction
	line(this->current_frame,location,Point(location.x+px,location.y+py),Scalar(0,0,255),1,8);
}

/**
 * @function draw_rectangle
 * @brief Draws simple rectangle
**/
void LocationVisualizer::draw_rectangle(Mat img, Point one,Point two,Point three,Point four)
{
	draw_line(img,one,two);
	draw_line(img,two,three);
	draw_line(img,three,four);
	draw_line(img,four,one);
}


/**
 * @function draw_line
 * @brief Draw a simple line
 */
void LocationVisualizer::draw_line( Mat img, Point start, Point end )
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
