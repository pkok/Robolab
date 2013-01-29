#include "visualHorizon.h"
#include "../CommonSource/RobotConfiguration.h"

#include <alproxies/almemoryproxy.h>
#include <alvision/alvisiondefinitions.h>

VisualHorizon::VisualHorizon(std::string robotIP, int camera) {
  this->robotIP = robotIP;
  this->camera = camera; // default on TopCamera

  this->createProxies();

  RobotConfiguration *config = new RobotConfiguration(robotIP);

  // look up the angle of the camera with a unrotated head
}

VisualHorizon::~VisualHorizon() {
  this->destroyProxies();
}

cv::Vec3f VisualHorizon::getHorizon() {
  cv::Vec3f horizonDirection = cv::Vec3f();
  cv::Vec3f torsoAngles = cv::Vec3f();
  cv::Vec3f headPitch = cv::Vec3f();
  cv::Vec3f headYaw = cv::Vec3f();
  cv::Vec3f cameraOrientation = cv::Vec3f();

  horizonDirection[2] = 1.0f;

  torsoAngles[0] = this->memory->getData("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value");
  torsoAngles[1] = this->memory->getData("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value");
  torsoAngles[2] = this->memory->getData("Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value");

  headPitch[1] = this->memory->getData("Device/SubDeviceList/HeadPitch/Position/Actuator/Value");

  headYaw[2] = this->memory->getData("Device/SubDeviceList/HeadYaw/Position/Actuator/Value");

  if (this->camera == AL::kTopCamera) {
    cameraOrientation[0] = this->config->getData("CameraTopWX");
    cameraOrientation[1] = this->config->getData("CameraTopWY");
    cameraOrientation[2] = this->config->getData("CameraTopWZ");
  }
  else if (this->camera == AL::kBottomCamera) {
    cameraOrientation[0] = this->config->getData("CameraBottomWX");
    cameraOrientation[1] = this->config->getData("CameraBottomWY");
    cameraOrientation[2] = this->config->getData("CameraBottomWZ");
  }
  return rotate_vector(cameraOrientation, rotate_vector(headPitch, rotate_vector(headYaw, rotate_vector(torsoAngles, horizonDirection))));
}

cv::Vec<cv::Point2f, 2> VisualHorizon::getHorizonLine() {
  cv::Vec3f horizon = this->getHorizon();
  cv::Point2f leftPoint = rotate_point(angle_between(horizon, unitVectorZ), Point(0, 0.5));
  cv::Point2f rightPoint = rotate_point(angle_between(horizon, unitVectorZ), Point(1, 0.5));
  return cv::Vec(leftPoint, rightPoint);
}

void VisualHorizon::createProxies() {
  this->destroyProxies();
  this->memoryProxy = new AL::ALMemoryProxy(this->robotIP);
}

void VisualHorizon::destroyProxies() {
  
}
