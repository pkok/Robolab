#include "FeatureDistance.h"

#include <math.h>

//#include <almath>
#include <almath/tools/almath.h>
#include <almath/tools/almathio.h>
#include <almath/tools/altransformhelpers.h>
#include <almath/types/altransform.h>
#include <almath/types/alrotation3d.h>
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

std::vector<float> fromXYtoRangeBearing(std::vector<float> xyCoordinates) {
  float range = sqrt(xyCoordinates[0] * xyCoordinates[0] + xyCoordinates[1] * xyCoordinates[1]);
  float bearing = 1.57079633f - atan2(xyCoordinates[1], xyCoordinates[0]);
  std::vector<float> newCoordinates(2, 0.0f);
  newCoordinates[0] = range;
  newCoordinates[1] = bearing;
  return newCoordinates;
}

std::vector<float> FeatureDistance::getRangeAndBearingFromImagePosition(float imagePositionX, float imagePositionY) {
  return fromXYtoRangeBearing(this->getPositionFromImagePosition(imagePositionX, imagePositionY));
}

std::vector<float> FeatureDistance::getRangeAndBearingFromImagePosition(const std::vector<float>& imagePosition) {
  return fromXYtoRangeBearing(this->getPositionFromImagePosition(imagePosition));
}

std::vector<float> FeatureDistance::getPositionFromImagePosition(const float imagePositionX, float imagePositionY) {
  std::vector<float> imagePosition(2, 0.0f);
  imagePosition[0] = imagePositionX;
  imagePosition[1] = imagePositionY;
  return this->getPositionFromImagePosition(imagePosition);
}

std::vector<float> FeatureDistance::getPositionFromImagePosition(const std::vector<float>& imagePosition) {
  const std::string cameraName = (this->camera == AL::kTopCamera) ? "CameraTop" : "CameraBottom";
  std::vector<float> cameraPosition = this->motionProxy->getPosition(cameraName, 2 /* FRAME_ROBOT */, true);
  std::vector<float> tmp = this->videoProxy->getAngularPositionFromImagePosition(this->camera, imagePosition);

  AL::Math::Transform featureAngle = AL::Math::Transform::from3DRotation(0.0f, -tmp[1], -tmp[0]);

  AL::Math::Rotation3D r(std::vector<float>(cameraPosition.begin()+3, cameraPosition.end()));
  AL::Math::Transform t = AL::Math::transformFromPosition6D(AL::Math::Position6D(cameraPosition));

  AL::Math::Transform totalTransform;
  AL::Math::changeReferenceTransform(featureAngle, t, totalTransform);
  r = AL::Math::rotation3DFromTransform(totalTransform);

  AL::Math::Position3D pluckerLineU(r.toVector());
  pluckerLineU /= pluckerLineU.norm();
  AL::Math::Position3D pluckerLineV = pluckerLineU.crossProduct(AL::Math::position3DFromTransform(totalTransform));

  AL::Math::Position3D pluckerPlane(0.0f, 0.0f, 1.0f);
  float pluckerPlaneWeight = 0.0;

  AL::Math::Position3D intersection = pluckerLineV.crossProduct(pluckerPlane);
  intersection -= (pluckerPlaneWeight * pluckerLineU);
  intersection /= pluckerLineU.dotProduct(pluckerPlane);
  intersection.y = sqrt(intersection.y);

  std::vector<float> result(2, 0.0f);
  result[0] = intersection.x;
  result[1] = intersection.y;
  return result;
}
