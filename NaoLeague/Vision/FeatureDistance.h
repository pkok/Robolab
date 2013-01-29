/**
 * 
 *
 * @author    Patrick de Kok (patrick@cxiu.nl)
 */

#include <string>

#include <alproxies/almotionproxy.h>
#include <alproxies/alvideodeviceproxy.h>

class FeatureDistance {
  public:
    FeatureDistance(std::string robotIP, int camera);
    ~FeatureDistance();
    std::vector<float> getFeaturePositionFromImagePosition(const std::vector<float>& imagePosition, const int& space);

  private:
    void createProxies();
    void destroyProxies();

    std::string robotIP;
    int camera;
    AL::ALMotionProxy *motionProxy;
    AL::ALVideoDeviceProxy *videoProxy;
};
