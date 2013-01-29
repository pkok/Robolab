/**
 * An index of joint and sensor distances per body version, as defined in the
 * documentation.
 *
 * @author    Patrick de Kok (patrick@cxiu.nl)
 */
#ifndef _BODYCONFIGURATION
#define _BODYCONFIGURATION

#include <string>

#include <alproxies/almotion.h>
#include <opencv2/core/core.hpp>
#include <boost/unordered_map.hpp>

/**
 * Type of chassis.  This defines broadly which actuators are connected.
 */
const int MODEL_TYPE_H25 = 3;
const int MODEL_TYPE_H21 = 2;
const int MODEL_TYPE_T14 = 1;
const int MODEL_TYPE_T2 = 0;

/**
 * Version constants.  Used for determining head, body and arm type.
 */
const int NAO_VERSION_40 = 2;
const int NAO_VERSION_33 = 1;
const int NAO_VERSION_32 = 0;


class RobotConfiguration {
  public:
    RobotConfiguration(std::string robotIP);
    ~RobotConfiguration();
    float getData(std::string key);
    //Vec3f getDistance(std::string fromJoint, std::string toJoint);

  private:
    void createProxies();
    void destroyProxies();
    void setRobotVersion();
    void initializeBodyConfiguration();
    void initializeBodyConfigurationV40();
    void initializeBodyConfigurationV33();
    void initializeBodyConfigurationV32();
    void initializeHeadConfiguration();
    void initializeHeadConfigurationV40();
    void initializeHeadConfigurationV33();
    void initializeHeadConfigurationV32();

    const int modelType; // H25, H21, T14 or T2
    const int headVersion; // v4.0, v3.3 or v3.2
    const int bodyVersion; // v4.0, v3.3 or v3.2
    const bool laser;
    const bool legs;
    const bool arms;
    const bool extendedArms;
    const bool hands;
    const int armVersion; // v4.0, v3.3 or v3.2
    const int legCount; // 0 or 2
    const int armCount; // 0 or 2
    const int handCount; // 0 or 2

    boost::unordered_map<std::string, float> configuration;
    AL::ALMotionProxy *motionProxy;
};

#endif
