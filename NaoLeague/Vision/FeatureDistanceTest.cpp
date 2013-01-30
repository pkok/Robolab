#include <iostream>
//#include <string>
#include <sstream>

#include "FeatureDistance.h"
#include "../CommonSource/Macros.h"

#include <alvision/alvisiondefinitions.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>

int main(int argc, char **argv) {
  ASSERT(argc >= 2, "Need an argument: IP-address of robot");
  if (argv[1][0] == '1' && argv[1][1] == '2' && argv[1][2] == '7') {
    std::cout << "Virtual robot!" << std::endl;
    AL::ALRobotPostureProxy *rpp = new AL::ALRobotPostureProxy(argv[1], 9559);
    rpp->goToPosture("StandInit", 1.0f);
    AL::ALMotionProxy *motion = new AL::ALMotionProxy(argv[1], 9559);
    motion->setAngles("HeadPitch", -0.538476f, 1.0f);
    motion->setAngles("HeadYaw", 0.0f, 1.0f);
  }

  FeatureDistance fd = FeatureDistance(argv[1], AL::kBottomCamera);
  std::cout << "Type input! Either:" << std::endl;
  std::cout << "- a pixel position in QVGA image (2 ints)" << std::endl;
  std::cout << "- a normalized point (2 floats between 0 and 1)" << std::endl;
  std::cout << "- 'q' to quit" << std::endl;

  std::vector<float> pix = std::vector<float>(2, 0.0f);
  std::string input;
  std::cout << "=> ";
  std::getline(std::cin, input);
  while (input.compare("q")) {
    std::stringstream(input) >> pix[0] >> pix[1];
    if (pix[0] > 1.0f || pix[1] > 1.0f) { 
      pix[0] /= 320.0f;
      pix[1] /= 240.0f;
      std::cout << "Input transformed to: " << pix << std::endl;
    }
    std::vector<float> pos = fd.getFeaturePositionFromImagePosition(pix);
    for (std::vector<float>::iterator iter = pos.begin(); iter != pos.end(); ++iter) {
      std::cout << *iter << " ";
    }
    std::cout << std::endl;
    std::cout << "=> ";
    std::getline(std::cin, input);
  }

  return 0;
}
