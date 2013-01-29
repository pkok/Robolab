#include <iostream>
//#include <string>
#include <sstream>

#include "FeatureDistance.h"
#include "../CommonSource/Macros.h"

#include <alvision/alvisiondefinitions.h>
#include <alproxies/alrobotpostureproxy.h>

int main(int argc, char **argv) {
  ASSERT(argc >= 2, "Need an argument: IP-address of robot");
  ASSERT(argc >= 4, "Need an argument: (normalized) position of a feature");
  if (argv[1][0] == '1' && argv[1][1] == '2' && argv[1][2] == '7') {
    std::cout << "Virtual robot!" << std::endl;
    AL::ALRobotPostureProxy *rpp = new AL::ALRobotPostureProxy(argv[1], 9559);
    rpp->goToPosture("StandInit", 1.0f);
  }

  FeatureDistance fd = FeatureDistance(argv[1], AL::kBottomCamera);
  std::vector<float> pix = std::vector<float>(2, 0.0f);
  std::stringstream(argv[2]) >> pix[0];
  std::stringstream(argv[3]) >> pix[1];
  if (pix[0] > 1.0f) { 
    pix[0] /= 320.0f;
  }
  if (pix[1] > 1.0f) {
    pix[1] /= 240.0f;
  }
  std::vector<float> pos = fd.getFeaturePositionFromImagePosition(pix);
  for (std::vector<float>::iterator iter = pos.begin(); iter != pos.end(); ++iter) {
    std::cout << *iter << std::endl;
  }

  return 0;
}
