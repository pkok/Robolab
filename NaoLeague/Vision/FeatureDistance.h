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
    std::vector<float> getRangeAndBearingFromImagePosition(const std::vector<float>& imagePosition);
    std::vector<float> getRangeAndBearingFromImagePosition(float imagePositionX, float imagePositionY);
    std::vector<float> getPositionFromImagePosition(const std::vector<float>& imagePosition);
    std::vector<float> getPositionFromImagePosition(const float imagePositionX, float imagePositionY);

  private:
    void createProxies();
    void destroyProxies();

    std::string robotIP;
    int camera;
    AL::ALMotionProxy *motionProxy;
    AL::ALVideoDeviceProxy *videoProxy;
};
