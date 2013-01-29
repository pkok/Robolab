#include "RobotConfiguration.h"

#include <alproxies/almotion.h>
#include <boost/unordered_map.hpp>

RobotConfiguration::RobotConfiguration(std::string robotIP) {
  this->robotIP = robotIP;

  this->createProxies();
  this->setRobotVersion();
  this->initializeBodyConfiguration();
  this->initializeHeadConfiguration();

  // Connection to robot is no longer required.
  this->destroyProxies();
}

RobotConfiguration::~RobotConfiguration() {
  this->destroyProxies();
}

float RobotConfiguration::getData(std::string offsetName) {
  if (this->configuration.find(offsetName) == this->configuration.end()) {
    throw Exception("Unknown configuration key");
  }
  return this->configuration[offsetName];
}

void RobotConfiguration::createProxies() {
  this->destroyProxies();
  this->motionProxy = new AL::ALMotionProxy(this->robotIP);
}

void RobotConfiguration::destroyProxies() {
  // disconnect this->motionProxy
}

int decypherModel(std::string modelString) {
  if (robotConfig[1][i].compare("naoH25")) {
    return MODEL_TYPE_H25;
  }
  else if (robotConfig[1][i].compare("naoH21") == 0) {
    return MODEL_TYPE_H21;
  } 
  else if (robotConfig[1][i].compare("naoT14") == 0) {
    return MODEL_TYPE_T14;
  }
  else if (robotConfig[1][i].compare("naoT2") == 0) {
    return MODEL_TYPE_T2;
  }
  throw Exception("Unknown Nao model");
}

int decypherVersion(std::string versionString) {
  if (versionString.compare("VERSION_40") == 0) {
    return NAO_VERSION_40;
  }
  else if (versionString.compare("VERSION_33") == 0) {
    return NAO_VERSION_33;
  }
  else if (versionString.compare("VERSION_32") == 0) {
    return NAO_VERSION_32;
  }
  throw Exception("Unknown Nao version");
}

void RobotConfiguration::setRobotVersion() {
  AL::ALValue robotConfig = this->motionProxy->getRobotConfig();
  for (int i = 0; i < robotConfig[0].getSize(); ++i) {
    if (robotConfig[0][i].compare("Model Type") == 0) {
      this->modelType = decypherModel(robotConfig[1][i]);
    }
    else if (robotConfig[0][i].compare("Head Version") == 0) {
      this->headVersion = decypherVersion(robotConfig[1][i]);
    }
    else if (robotConfig[0][i].compare("Body Version") == 0) {
      this->bodyVersion = decypherVersion(robotConfig[1][i]);
    }
    else if (robotConfig[0][i].compare("Laser") == 0) {
      this->laser = (bool) robotConfig[1][i];
    }
    else if (robotConfig[0][i].compare("Legs") == 0) {
      this->legs = (bool) robotConfig[1][i];
    }
    else if (robotConfig[0][i].compare("Arms") == 0) {
      this->arms = (bool) robotConfig[1][i];
    }
    else if (robotConfig[0][i].compare("Extended Arms") == 0) {
      this->extendedArms = (bool) robotConfig[1][i];
    }
    else if (robotConfig[0][i].compare("Hands") == 0) {
      this->hands = (bool) robotConfig[1][i];
    }
    else if (robotConfig[0][i].compare("Arm Version") == 0) {
      this->armVersion = decypherVersion(robotConfig[1][i]);
    }
    else if (robotConfig[0][i].compare("Number of Legs") == 0) {
      this->legCount = (bool) robotConfig[1][i];
    }
    else if (robotConfig[0][i].compare("Number of Arms") == 0) {
      this->armCount = (bool) robotConfig[1][i];
    }
    else if (robotConfig[0][i].compare("Number of Hands") == 0) {
      this->handCount = (bool) robotConfig[1][i];
    }
  }
}

void RobotConfiguration::initializeBodyConfiguration() {
  if (this->bodyVersion == NAO_VERSION_40) {
    this->initializeBodyConfigurationV40();
  }
  else if (this->bodyVersion == NAO_VERSION_33) {
    this->initializeBodyConfigurationV33();
  }
  else if (this->bodyVersion == NAO_VERSION_32) {
    this->initializeBodyConfigurationV32();
  }
  else {
    throw Exception("Undefined body version");
  }

  this->configuration["AccelerometerOffsetX"] = -8.00f;
  this->configuration["AccelerometerOffsetY"] = 6.06f;
  this->configuration["AccelerometerOffsetZ"] = 27.00f;
  this->configuration["GyrometerOffsetX"] = -8.00f;
  this->configuration["GyrometerOffsetY"] = 6.00f;
  this->configuration["GyrometerOffsetZ"] = 29.00f;

  this->configuration["USSensor1OffsetX"] = 53.70f;
  this->configuration["USSensor1OffsetY"] = -34.10f;
  this->configuration["USSensor1OffsetZ"] = 69.80f;
  this->configuration["USSensor2OffsetX"] = 47.70f;
  this->configuration["USSensor2OffsetY"] = -41.60f;
  this->configuration["USSensor2OffsetZ"] = 50.90f;
  this->configuration["USSensor3OffsetX"] = 53.70f;
  this->configuration["USSensor3OffsetY"] = 34.10f;
  this->configuration["USSensor3OffsetZ"] = 69.80f;
  this->configuration["USSensor4OffsetX"] = 47.70f;
  this->configuration["USSensor4OffsetY"] = 41.60f;
  this->configuration["USSensor4OffsetZ"] = 50.90f;
  
  if (this->legs) {
    this->configuration["LFsrFLOffsetX"] = 70.25f;
    this->configuration["LFsrFLOffsetY"] = 29.90f;
    this->configuration["LFsrFROffsetX"] = 70.25f;
    this->configuration["LFsrFROffsetY"] = -23.10f;
    this->configuration["LFsrRLOffsetX"] = -30.25f;
    this->configuration["LFsrRLOffsetY"] = 29.90f;
    this->configuration["LFsrRROffsetX"] = -29.65f;
    this->configuration["LFsrRROffsetY"] = -19.10f;
    this->configuration["RFsrFLOffsetX"] = 70.25f;
    this->configuration["RFsrFLOffsetY"] = 23.10f;
    this->configuration["RFsrFROffsetX"] = 70.25f;
    this->configuration["RFsrFROffsetY"] = -29.9f;
    this->configuration["RFsrRLOffsetX"] = -30.25f;
    this->configuration["RFsrRLOffsetY"] = 19.10f;
    this->configuration["RFsrRROffsetX"] = -29.65f;
    this->configuration["RFsrRROffsetY"] = -29.90f;
  }
}

void RobotConfiguration::initializeBodyConfigurationV40() {
  if (this->modelType >= MODEL_TYPE_T2) {
    this->configuration["NeckOffsetZ"] = 126.50f;
  }
  if (this->modelType >= MODEL_TYPE_T14) { 
    this->configuration["ShoulderOffsetY"] = 98.00f;
    this->configuration["ElbowOffsetY"] = 15.00f;
    this->configuration["UpperArmLength"] = 105.00f;
    this->configuration["LowerArmLength"] = 55.95f;
    this->configuration["ShoulderOffsetZ"] = 100.00f;
    this->configuration["HandOffsetX"] = 57.75f;
    this->configuration["HandOffsetZ"] = 12.31f;
  }
  if (this->modelType >= MODEL_TYPE_H21) {
    this->configuration["HipOffsetZ"] = 85.00f;
    this->configuration["HipOffsetY"] = 50.00f;
    this->configuration["ThighLength"] = 100.00f;
    this->configuration["TibiaLength"] = 102.90f;
    this->configuration["FootHeight"] = 45.19f;
  }
}

void RobotConfiguration::initializeBodyConfigurationV33() {
  if (this->modelType >= MODEL_TYPE_T2) {
    this->configuration["NeckOffsetZ"] = 126.50f;
  }
  if (this->modelType >= MODEL_TYPE_T14) { 
    this->configuration["ShoulderOffsetY"] = 98.00f;
    this->configuration["ElbowOffsetY"] = 15.00f;
    this->configuration["UpperArmLength"] = 105.00f;
    this->configuration["LowerArmLength"] = 55.95f;
    this->configuration["ShoulderOffsetZ"] = 100.00f;
    this->configuration["HandOffsetX"] = 57.75f;
    this->configuration["HandOffsetZ"] = 12.31f;
  }
  if (this->modelType >= MODEL_TYPE_H21) {
    this->configuration["HipOffsetZ"] = 85.00f;
    this->configuration["HipOffsetY"] = 50.00f;
    this->configuration["ThighLength"] = 100.00f;
    this->configuration["TibiaLength"] = 102.90f;
    this->configuration["FootHeight"] = 45.19f;
  }
}

void RobotConfiguration::initializeBodyConfigurationV32() {
  if (this->modelType >= MODEL_TYPE_T2) {
    this->configuration["NeckOffsetZ"] = 126.50f;
  }
  if (this->modelType >= MODEL_TYPE_T14 || this->arms) { 
    this->configuration["ShoulderOffsetY"] = 98.00f;
    this->configuration["ElbowOffsetY"] = 0.00f;
    this->configuration["UpperArmLength"] = 90.00f;
    this->configuration["LowerArmLength"] = 50.55f;
    this->configuration["ShoulderOffsetZ"] = 100.00f;
    this->configuration["HandOffsetX"] = 58.00f;
    this->configuration["HandOffsetZ"] = 15.90f;
  }
  if (this->modelType >= MODEL_TYPE_H21 || this->legs) {
    this->configuration["HipOffsetZ"] = 85.00f;
    this->configuration["HipOffsetY"] = 50.00f;
    this->configuration["ThighLength"] = 100.00f;
    this->configuration["TibiaLength"] = 102.90f;
    this->configuration["FootHeight"] = 45.11f;
  }
}

void RobotConfiguration::initializeHeadConfiguration() {
  if (this->headVersion == NAO_VERSION_40) {
    this->initializeHeadConfigurationV40();
  }
  else if (this->headVersion == NAO_VERSION_33) {
    this->initializeHeadConfigurationV33();
  }
  else if (this->headVersion == NAO_VERSION_32) {
    this->initializeHeadConfigurationV32();
  }
  else {
    throw Exception("Undefined head version");
  }

  this->configuration["LLoudSpeakerOffsetX"] = 3.80f;
  this->configuration["LLoudSpeakerOffsetY"] = 45.30f;
  this->configuration["LLoudSpeakerOffsetZ"] = 52.60f;
  this->configuration["RLoudSpeakerOffsetX"] = 3.80f;
  this->configuration["RLoudSpeakerOffsetY"] = -45.30f;
  this->configuration["RLoudSpeakerOffsetZ"] = 52.60f;

  this->configuration["LIROffsetX"] = 44.10f;
  this->configuration["LIROffsetY"] = -25.70f;
  this->configuration["LIROffsetZ"] = 46.60f;
  this->configuration["RIROffsetX"] = 44.10f;
  this->configuration["RIROffsetY"] = 25.70f;
  this->configuration["RIROffsetZ"] = 46.60f;

  if (this->laser) {
    this->configuration["LaserOffsetZ"] = 106.60f;
  }
}

void RobotConfiguration::initializeHeadConfigurationV40() {
  this->configuration["MicroFrontOffsetX"] = 48.90f;
  this->configuration["MicroFrontOffsetY"] = 0.00f;
  this->configuration["MicroFrontOffsetZ"] = 76.00f;
  this->configuration["MicroRearOffsetX"] = -46.00f;
  this->configuration["MicroRearOffsetY"] = 0.00f;
  this->configuration["MicroRearOffsetZ"] = 81.40f;
  this->configuration["MicroLeftOffsetX"] = -19.50f;
  this->configuration["MicroLeftOffsetY"] = 60.60f;
  this->configuration["MicroLeftOffsetZ"] = 33.10f;
  this->configuration["MicroRightOffsetX"] = -19.50f;
  this->configuration["MicroRightOffsetY"] = -60.60f;
  this->configuration["MicroRightOffsetZ"] = 33.10f;

  this->configuration["CameraTopOffsetX"] = 58.71f;
  this->configuration["CameraTopOffsetY"] = 0.00f;
  this->configuration["CameraTopOffsetZ"] = 63.64f;
  this->configuration["CameraBottomOffsetX"] = 50.71f;
  this->configuration["CameraBottomOffsetY"] = 0.00f;
  this->configuration["CameraBottomOffsetZ"] = 17.74f;
  // Not really configuration, but where to put them otherwise?
  // Maybe change the name of this->configuration.
  this->configuration["CameraTopWX"] = 0.0f;
  this->configuration["CameraTopWY"] = 0.0209f;
  this->configuration["CameraTopWZ"] = 0.0f;
  this->configuration["CameraBottomWX"] = 0.0f;
  this->configuration["CameraBottomWY"] = 0.6929f;
  this->configuration["CameraBottomWZ"] = 0.0f;
  this->configuration["CameraTopHFOV"] = 1.06412724f;
  this->configuration["CameraTopVFOV"] = 0.831474856f;
  this->configuration["CameraBottomHFOV"] = 1.06412724f;
  this->configuration["CameraBottomVFOV"] = 0.831474856f;
}

void RobotConfiguration::initializeHeadConfigurationV33() {
  this->configuration["MicroFrontOffsetX"] = 48.90f;
  this->configuration["MicroFrontOffsetY"] = 0.00f;
  this->configuration["MicroFrontOffsetZ"] = 76.00f;
  this->configuration["MicroRearOffsetX"] = -46.00f;
  this->configuration["MicroRearOffsetY"] = 0.00f;
  this->configuration["MicroRearOffsetZ"] = 81.40f;
  this->configuration["MicroLeftOffsetX"] = -19.50f;
  this->configuration["MicroLeftOffsetY"] = 60.60f;
  this->configuration["MicroLeftOffsetZ"] = 33.10f;
  this->configuration["MicroRightOffsetX"] = -19.50f;
  this->configuration["MicroRightOffsetY"] = -60.60f;
  this->configuration["MicroRightOffsetZ"] = 33.10f;

  this->configuration["CameraTopOffsetX"] = 53.90f;
  this->configuration["CameraTopOffsetY"] = 0.00f;
  this->configuration["CameraTopOffsetZ"] = 67.90f;
  this->configuration["CameraBottomOffsetX"] = 48.80f;
  this->configuration["CameraBottomOffsetY"] = 0.00f;
  this->configuration["CameraBottomOffsetZ"] = 23.81f;
  // Not really configuration, but where to put them otherwise?
  // Maybe change the name of this->configuration.
  this->configuration["CameraTopWX"] = 0.0f;
  this->configuration["CameraTopWY"] = 0.0f;
  this->configuration["CameraTopWZ"] = 0.0f;
  this->configuration["CameraBottomWX"] = 0.0f;
  this->configuration["CameraBottomWY"] = 0.6981f;
  this->configuration["CameraBottomWZ"] = 0.0f;
  this->configuration["CameraTopHFOV"] = 0.834267382f;
  this->configuration["CameraTopVFOV"] = 0.60737458f;
  this->configuration["CameraBottomHFOV"] = 0.834267382;
  this->configuration["CameraBottomVFOV"] = 0.60737458f;
}

void RobotConfiguration::initializeHeadConfigurationV32() {
  this->configuration["MicroFrontOffsetX"] = 41.00f;
  this->configuration["MicroFrontOffsetY"] = 0.00f;
  this->configuration["MicroFrontOffsetZ"] = 91.50f;
  this->configuration["MicroRearOffsetX"] = -57.70f;
  this->configuration["MicroRearOffsetY"] = 0.00f;
  this->configuration["MicroRearOffsetZ"] = 69.30f;
  this->configuration["MicroLeftOffsetX"] = -19.50f;
  this->configuration["MicroLeftOffsetY"] = 60.60f;
  this->configuration["MicroLeftOffsetZ"] = 33.10f;
  this->configuration["MicroRightOffsetX"] = -19.50f;
  this->configuration["MicroRightOffsetY"] = -60.60f;
  this->configuration["MicroRightOffsetZ"] = 33.10f;

  this->configuration["CameraTopOffsetX"] = 53.90f;
  this->configuration["CameraTopOffsetY"] = 0.00f;
  this->configuration["CameraTopOffsetZ"] = 67.90f;
  this->configuration["CameraBottomOffsetX"] = 48.80f;
  this->configuration["CameraBottomOffsetY"] = 0.00f;
  this->configuration["CameraBottomOffsetZ"] = 23.81f;
  // Not really configuration, but where to put them otherwise?
  // Maybe change the name of this->configuration.
  this->configuration["CameraTopWX"] = 0.0f;
  this->configuration["CameraTopWY"] = 0.0f;
  this->configuration["CameraTopWZ"] = 0.0f;
  this->configuration["CameraBottomWX"] = 0.0f;
  this->configuration["CameraBottomWY"] = 0.6981f;
  this->configuration["CameraBottomWZ"] = 0.0f;
  this->configuration["CameraTopHFOV"] = 0.834267382f;
  this->configuration["CameraTopVFOV"] = 0.60737458f;
  this->configuration["CameraBottomHFOV"] = 0.834267382;
  this->configuration["CameraBottomVFOV"] = 0.60737458f;
}
