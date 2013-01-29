#include "FeatureDistance.h"

#include <math.h>

//#include <almath>
#include <alproxies/almotionproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alvisiondefinitions.h>

#include <opencv2/core/core.hpp>

FeatureDistance::FeatureDistance(std::string robotIP, int camera) {
  this->robotIP = robotIP;
  this->camera = camera; // defaults to AL::kTopCamera

  this->createProxies();
}

FeatureDistance::~FeatureDistance() {
  this->destroyProxies();
}

void FeatureDistance::createProxies() {
  this->destroyProxies();
  this->motionProxy = new AL::ALMotionProxy(this->robotIP);
  this->videoProxy = new AL::ALVideoDeviceProxy(this->robotIP);
}

void FeatureDistance::destroyProxies() {
  // destroy this->motionProxy
  // destroy this->videoProxy
}

std::vector<float> FeatureDistance::getFeaturePositionFromImagePosition(const std::vector<float>& imagePosition) {
  const std::string cameraName = (this->camera == AL::kTopCamera) ? "CameraTop" : "CameraBottom";
  std::vector<float> cameraPosition = this->motionProxy->getPosition(cameraName, 2 /* FRAME_TORSO */, true);
  std::vector<float> featureAngle = this->videoProxy->getAngularPositionFromImagePosition(this->camera, imagePosition);

  const float cx = cos(cameraPosition[3]);
  const float cy = cos(cameraPosition[4]);
  const float cz = cos(cameraPosition[5]);
  const float sx = sin(cameraPosition[3]);
  const float sy = sin(cameraPosition[4]);
  const float sz = sin(cameraPosition[5]);

  std::vector<float> normalizedCameraDirection = std::vector<float>(cameraPosition.begin(), cameraPosition.begin() + 3);
  float norm = sqrt(normalizedCameraDirection[0] * normalizedCameraDirection[0] + normalizedCameraDirection[1] * normalizedCameraDirection[1] + normalizedCameraDirection[2] * normalizedCameraDirection[2]);
  normalizedCameraDirection[0] /= norm;
  normalizedCameraDirection[1] /= norm;
  normalizedCameraDirection[2] /= norm;

  std::vector<float> featureDirection = std::vector<float>(3, 0.0f);
  featureDirection[1] = cos(-featureAngle[1]);
  featureDirection[2] = cos(featureAngle[0]);
  norm = sqrt(featureDirection[1] * featureDirection[1] + featureDirection[2] * featureDirection[2]);
  featureDirection[1] /= norm;
  featureDirection[2] /= norm;

  // feature direction in world coordinates
  std::vector<float> rotatedDirection = std::vector<float>(3, 0.0f);
  rotatedDirection[0] = cz * sy * (sx * featureDirection[1] - cx * featureDirection[2]) - sz * (cx * featureDirection[1] - sx * featureDirection[2]);
  rotatedDirection[1] = sz * sy * (sx * featureDirection[1] - cx * featureDirection[2]) + cz * (cx * featureDirection[1] - sx * featureDirection[2]);
  rotatedDirection[2] = cy * (sx * featureDirection[1] + cx * featureDirection[2]);

  float angle_x = acos(rotatedDirection[0] * normalizedCameraDirection[0] + rotatedDirection[2] * normalizedCameraDirection[2]);
  float angle_y = acos(rotatedDirection[1] * normalizedCameraDirection[1] + rotatedDirection[2] * normalizedCameraDirection[2]);
  std::vector<float> result = std::vector<float>(2);
  result[0] = tan(angle_x) * sqrt(cameraPosition[0] * cameraPosition[0] + cameraPosition[2] * cameraPosition[2]); 
  result[1] = tan(angle_y) * sqrt(cameraPosition[1] * cameraPosition[1] + cameraPosition[2] * cameraPosition[2]);
  return result;
 }
