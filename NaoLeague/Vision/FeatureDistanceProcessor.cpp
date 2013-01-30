/**
 * 
 *
 * @author    Patrick de Kok (patrick@cxiu.nl)
 * @version   1.0
 * @date      01/30/2013, 06:02:34 PM
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <queue>
#include <algorithm>
#include <iterator>

#include <alvision/alvisiondefinitions.h>
#include "FeatureDistance.h"

using namespace std;

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

string processed_line(vector<string> tokens, FeatureDistance *fd) {
  stringstream oss;
  vector<float> coordinate(2, 0.0f);
  vector<float> transformedCoordinate(2, 0.0f);

  oss << tokens[0] << ' ';
  for (vector<string>::iterator token = tokens.begin() + 1; token != tokens.end(); ++token) {
    oss << *token << ' '; // F or G
    oss << *++token << ' '; // type_id
    from_string<float>(coordinate[0], *++token, dec); // pix_x / max_x
    from_string<float>(coordinate[1], *++token, dec); // pix_y / max_y
    transformedCoordinate = fd->getRangeAndBearingFromImagePosition(coordinate);
    oss << transformedCoordinate[0] << " "; // range
    oss << transformedCoordinate[1] << " "; // bearing
  }
  return oss.str();
}

int main(int argc, char** argv) {
  if (argc < 2) {
    cout << "Need 2 arguments: input_file and output_file" << endl;
    return -1;
  }
  ifstream infile(argv[1]);
  ofstream outfile(argv[2]);
  string line;
  istringstream iss;
  FeatureDistance featureDistance("127.0.0.1", AL::kBottomCamera);
  vector<string> tokens;

  if (infile.is_open()) {
    while (infile.good()) {
      getline(infile, line);
      tokens = split(line, ' ');
      if ((tokens.size() - 1) % 4 == 0 && tokens.size() > 1) {
        outfile << processed_line(tokens, &featureDistance) << endl;
      }
    }
    infile.close();
    outfile.close();
  }

  return 0;
}
