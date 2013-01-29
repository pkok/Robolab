/**
 * Computes the position of the horizon in the camera image.
 *
 * @author    Patrick de Kok (patrick@cxiu.nl)
 */
#ifndef _VISUALHORIZON
#define _VISUALHORIZON

#include <string>

#include <alproxies/almemoryproxy.h>
#include <alvision/alvisiondefinitions.h>
#include <opencv2/core/core.hpp>

class VisualHorizon {
  public:
    VisualHorizon(std::string robotIP, int camera = AL::kTopCamera);
    ~VisualHorizon();
    cv::Vec3f getHorizon(); // normal vector to the horizon's plane in the selected camera's image
    cv::Vec<cv::Point2f, 2> getHorizonLine();

  private:
    void createProxies();
    void destroyProxies();

    std::string robotIP;
    int camera; // which camera to use - AL::kTopCamera is default
    AL::ALMemoryProxy *memoryProxy;
    RobotConfiguration *config;
};

#endif
