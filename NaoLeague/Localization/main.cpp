/*
 * main.cpp
 *
 *  Created on: Jan 30, 2013
 *      Author: owner
 */



#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <queue>
#include <algorithm>
#include <iterator>
#include <vector>
#include <cv.h>
#include <string.h>
#include <highgui.h>

#include "ParticleFilter.h"
#include "InputGenerator.h"
//#include "ExtendedKalmanFilter.h"
#include "LocationVisualizer.h"

#define PI 3.14159

using namespace std;
using namespace cv;

vector<string> &split(const string &s, char delim, vector<string> &elems) {
  stringstream ss(s);
  string item;
  while(getline(ss, item, delim)) {
    elems.push_back(item);
  }
  return elems;
}

vector<string> split(const string &s, char delim) {
  vector<string> elems;
  return split(s, delim, elems);
}

template <class T>
bool from_string(T& t, const string& s, ios_base& (*f)(ios_base&)) {
  istringstream iss(s);
  return !(iss >> f >> t).fail();
}

string processed_line(string* filename,vector<VisualFeature>* vf,vector<string> tokens) {
  stringstream oss;
  vector<float> coordinate(2, 0.0f);
  vector<float> transformedCoordinate(2, 0.0f);

  oss << tokens[0] << ' '; // filename
  *filename = tokens[0];
  for (vector<string>::iterator token = tokens.begin() + 1; token != tokens.end(); ++token) {
		VisualFeature f;
		if ((*token).compare("f") == 0) {
			++token;
			if ((*token).compare("0") == 0) { //l_crossing
				f.type = l_crossing;
				//save coordinates
			    from_string<float>(coordinate[0], *++token, dec); // range
			    from_string<float>(coordinate[1], *++token, dec); // rad
			    //save feature range and bearing
			    f.range = coordinate[0]*100; //convert to cm
			    f.bearing = coordinate[1];
			    vf->push_back(f);
			}else if((*token).compare("1") == 0){//t_crossing
				f.type = t_crossing;
				//save coordinates
			    from_string<float>(coordinate[0], *++token, dec); // range
			    from_string<float>(coordinate[1], *++token, dec); // rad
			    //save feature range and bearing
			    f.range = coordinate[0]*100; //convert to cm
			    f.bearing = coordinate[1];
			    vf->push_back(f);
			}else if((*token).compare("3") == 0){//x_crossing
				f.type = x_crossing;
				//save coordinates
			    from_string<float>(coordinate[0], *++token, dec); // range
			    from_string<float>(coordinate[1], *++token, dec); // rad
			    //save feature range and bearing
			    f.range = coordinate[0]*100; //convert to cm
			    f.bearing = coordinate[1];
			    vf->push_back(f);
			}else{++token;++token;} //skip next type


		}
		else if((*token).compare("g") == 0){
			++token; //fuck right or left goal post

			f.type = goal_post;

			from_string<float>(coordinate[0], *++token, dec); // range
		    from_string<float>(coordinate[1], *++token, dec); // rad
			f.range = coordinate[0]*100; //convert to cm
			f.bearing = coordinate[1];
			vf->push_back(f);
		}
    // do stuff witch coordinates

    //convert to visual features:
    //range
    //bearing
    //id



  }
  return oss.str();
}





int main(int argc, char** argv) {
  /*if (argc < 1) {
    cout << "Need 1 argument: input_file" << endl;
    return -1;
  }*/
  //ifstream infile(argv[1]);

	//declare particle filter and Location Visualizer for processing:
	ParticleFilter pf;
	pf.set_params(); //TODO: CHANGE!
	pf.add_particles(200,-300,0,0,1);
	//pf.add_particles(200);

	InputGenerator ig;

	LocationVisualizer lv;
	  lv.draw_particle(pf.particles);
		lv.create_window();
		lv.clear_buffer();




	ifstream infile("straight_processed_patrick(1).txt");//"straight_processed_patrick_experimental.txt");//"straightwalk_binary_IPM_RnB.txt");//"straight_processed_patrick.txt");
	string line;
  istringstream iss;
  vector<string> tokens;
  int counter = 0;
  if (infile.is_open()) {
    while (infile.good()) {
      getline(infile, line);
      tokens = split(line, ' ');
      cout<<"line size:"<<tokens.size()<<endl;

      for(int i = 0; i< tokens.size();i++){
    	  cout<<tokens[i]<<endl;
      }
      if ((tokens.size() - 1) % 4 == 0 && tokens.size() > 1) {


    	  ////FILE PROCESSING
    	  ///new frame !!!
    	  string filename;
    	  vector<VisualFeature> vf;
    	  processed_line(&filename,&vf,tokens);
    	  filename += ".png";
    	  filename = "./dataset_straightwalk/" + filename;
    	  cout<<"processing:"<<filename<<"coutner:"<<counter<<endl;
    	  counter++;

    	  //read file
    	  Mat image;
    	  image = imread(filename,1);
    	  namedWindow( "Display window", CV_WINDOW_AUTOSIZE );// Create a window for display.
    	  imshow( "Display window", image );                   // Show our image inside it.
/*

    	  vector<VisualFeature> vis_feat;
    	ig.generate_features(counter * 20,0, 0,pf.feature_map, &vis_feat);
    	ig.calculate_range_bearing(pf.feature_map,counter*20, 0, 0, vis_feat);
    	for(int i = 0; i< vis_feat.size();i++){
    		cout<<"visual feature: "<<vis_feat[i].range<<" "<<vis_feat[i].bearing<<endl;
    	}
    	  counter++;*/
    	  ////////DO PARTICLE PROCESSING
    	  //shity implementation of odometry data:

    	  OdometryInformation of;
    	  of.rot = 0;
    	  of.x = 20;
    	  of.y = 0;

    	  pf.dynamic(of,vf);
    	  pf.resample();

    	  lv.draw_particle(pf.particles);
    		lv.create_window();


    		//make clean window (footballfield);
    		lv.clear_buffer();

      }
    }
    infile.close();
  }

  return 0;
}
