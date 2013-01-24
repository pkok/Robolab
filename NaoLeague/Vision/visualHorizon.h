/**
 * Computes the position of the horizon in the camera image.
 *
 * @author    Patrick de Kok (patrick@cxiu.nl)
 */
#ifndef _VISUALHORIZON
#define _VISUALHORIZON

#include <string>

#include <alvision/alvisiondefinitions.h>
#include <opencv2/core/core.hpp>

class VisualHorizon {
  public:
    VisualHorizon(std::string robotIP, int camera = AL::kTopCamera);
    ~VisualHorizon();
    Vec2f getHorizon();

  private:
    void createProxies();
    void destroyProxies();

    std::string robotIP;
    int camera; // which camera to use - AL::kTopCamera is default
    AL::ALMemoryProxy *memoryProxy;
};

#endif
