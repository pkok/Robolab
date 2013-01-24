#include "visualHorizon.h"

#include <alproxies/almemory.h>

VisualHorizon::VisualHorizon(std::string robotIP, int camera) {
  this->robotIP = robotIP;
  this->camera = camera; // default on TopCamera

  this->createProxies();

  // look up the angle of the camera with a unrotated head
}

VisualHorizon::~VisualHorizon() {
  this->destroyProxies();
}

Vec2f VisualHorizon::getHorizon() {
  float angle_x = this->memory->getData("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value");
  float angle_y = this->memory->getData("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value");
  float head_pitch = this->memory->getData("Device/SubDeviceList/HeadPitch/Position/Actuator/Value");
  float head_yaw = this->memory->getData("Device/SubDeviceList/HeadYaw/Position/Actuator/Value");

  Vec2f horizon_direction = Vec2f();

  // find final rotation of head,
  return horizon_direction;
}

void VisualHorizon::createProxies() {
  this->destroyProxies();
  this->memoryProxy = new AL::ALMemoryProxy(this->robotIP);
}

void VisualHorizon::destroyProxies() {
  
}
