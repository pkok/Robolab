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

//std::vector<float> FeatureDistance::getFeaturePositionFromImagePosition(const std::vector<float>& imagePosition) {
//  const std::string cameraName = (this->camera == AL::kTopCamera) ? "CameraTop" : "CameraBottom";
//  std::vector<float> cameraPosition = this->motionProxy->getPosition(cameraName, 2 /* FRAME_TORSO */, true);
//  std::vector<float> featureAngle = this->videoProxy->getAngularPositionFromImagePosition(this->camera, imagePosition);
//  std::cerr << "featureAngle: " << featureAngle << std::endl;
//
//  const float cx = cos(-cameraPosition[3]);
//  const float cy = cos(-cameraPosition[4]);
//  const float cz = cos(-cameraPosition[5]);
//  const float sx = sin(-cameraPosition[3]);
//  const float sy = sin(-cameraPosition[4]);
//  const float sz = sin(-cameraPosition[5]);
//
//  std::vector<float> normalizedCameraDirection = std::vector<float>(cameraPosition.begin(), cameraPosition.begin() + 3);
//  float norm = sqrt(normalizedCameraDirection[0] * normalizedCameraDirection[0] + normalizedCameraDirection[1] * normalizedCameraDirection[1] + normalizedCameraDirection[2] * normalizedCameraDirection[2]);
//  normalizedCameraDirection[0] /= norm;
//  normalizedCameraDirection[1] /= norm;
//  normalizedCameraDirection[2] /= norm;
//
//  std::vector<float> featureDirection = std::vector<float>(3, 0.0f);
//  //featureDirection[0] = 1.0f;
//  featureDirection[1] = cos(-featureAngle[1]);
//  featureDirection[2] = cos(featureAngle[0]);
//  norm = sqrt(featureDirection[0] * featureDirection[0] + featureDirection[1] * featureDirection[1] + featureDirection[2] * featureDirection[2]);
//  featureDirection[0] /= norm;
//  featureDirection[1] /= norm;
//  featureDirection[2] /= norm;
//  std::cerr << "featureDirection: " << featureDirection << std::endl;
//
//  // feature direction in world coordinates
//  std::vector<float> rotatedDirection = std::vector<float>(3, 0.0f);
//  rotatedDirection[0] = cz * sy * (sx * featureDirection[1] - cx * featureDirection[2]) - sz * (cx * featureDirection[1] - sx * featureDirection[2]);
//  rotatedDirection[1] = sz * sy * (sx * featureDirection[1] - cx * featureDirection[2]) + cz * (cx * featureDirection[1] - sx * featureDirection[2]);
//  rotatedDirection[2] = cy * (sx * featureDirection[1] + cx * featureDirection[2]);
//
//  /*
//  // inverse translation!!!
//  rotatedDirection[0] -= cameraPosition[0];
//  rotatedDirection[1] -= cameraPosition[1];
//  rotatedDirection[2] -= cameraPosition[2];
//  */
//
//  norm = sqrt(rotatedDirection[0] * rotatedDirection[0] + rotatedDirection[2] * rotatedDirection[2]);
//  float angle_x = acos(rotatedDirection[0]/norm * normalizedCameraDirection[0] + rotatedDirection[2]/norm * normalizedCameraDirection[2]);
//  norm = sqrt(rotatedDirection[1] * rotatedDirection[1] + rotatedDirection[2] * rotatedDirection[2]);
//  float angle_y = acos(rotatedDirection[1]/norm * normalizedCameraDirection[1] + rotatedDirection[2]/norm * normalizedCameraDirection[2]);
//  norm = sqrt(rotatedDirection[0] * rotatedDirection[0] + rotatedDirection[1] * rotatedDirection[1] + rotatedDirection[2] * rotatedDirection[2]);
//  float angle_z = acos(rotatedDirection[0]/norm * normalizedCameraDirection[0] + rotatedDirection[1]/norm * normalizedCameraDirection[1] + rotatedDirection[2]/norm * normalizedCameraDirection[2]);
//
//  std::vector<float> result = std::vector<float>(4);
//  // x,y coordinates
//  result[0] = tan(angle_x) * sqrt(cameraPosition[0] * cameraPosition[0] + cameraPosition[2] * cameraPosition[2]); 
//  result[1] = tan(angle_y) * sqrt(cameraPosition[1] * cameraPosition[1] + cameraPosition[2] * cameraPosition[2]);
//
//  // distance and angle
//  result[2] = tan(angle_z) * sqrt(cameraPosition[0] * cameraPosition[0] + cameraPosition[1] * cameraPosition[1] + cameraPosition[2] * cameraPosition[2]);
//  result[3] = angle_z;
//  return result;
// }

std::vector<float> FeatureDistance::getFeaturePositionFromImagePosition(const std::vector<float>& imagePosition) {
  const std::string cameraName = (this->camera == AL::kTopCamera) ? "CameraTop" : "CameraBottom";
  std::vector<float> cameraPosition = this->motionProxy->getPosition(cameraName, 2 /* FRAME_ROBOT */, true);
  std::vector<float> tmp = this->videoProxy->getAngularPositionFromImagePosition(this->camera, imagePosition);

  AL::Math::Transform featureAngle = AL::Math::Transform::from3DRotation(0.0f, tmp[1], tmp[0]);

  AL::Math::Rotation3D r(std::vector<float>(cameraPosition.begin()+3, cameraPosition.end()));
  AL::Math::Transform t = AL::Math::transformFromRotation3D(r);
  std::cerr << "r: " << r << std::endl;

  AL::Math::Transform totalTransform;
  AL::Math::changeReferenceTransform(t, featureAngle, totalTransform);
  r = AL::Math::rotation3DFromTransform(totalTransform);
  std::cerr << "r: " << r << std::endl;

  AL::Math::Position3D pluckerLineU(r.toVector());
  AL::Math::Position3D pluckerLineV = pluckerLineU.crossProduct(AL::Math::position3DFromTransform(totalTransform));

  AL::Math::Position3D pluckerPlane(0.0f, 0.0f, 1.0f);
  float pluckerPlaneWeight = 0.0;

  AL::Math::Position3D intersection = pluckerLineV.crossProduct(pluckerPlane) - (pluckerPlaneWeight * pluckerLineU);
  intersection /= pluckerLineU.dotProduct(pluckerPlane);
  std::cerr << intersection << std::endl; 

  return intersection.toVector();
}
