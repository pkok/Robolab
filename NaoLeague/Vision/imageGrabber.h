// Author: Tijmen Blankevoort

// construct imageGrabber with the IP address of the robot, grab an image from its camera
#ifndef _IMAGEGRABBER
#define _IMAGEGRABBER

#include <string>

#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <alerror/alerror.h>

#include<opencv2/core/core.hpp>

//possibly make a class for this in a separate file
struct ImageContainer{
	cv::Mat image;
	int timeStampMicroSeconds;
	int timeStampSeconds;
	int cameraID;

	float leftAngle;
	float topAngle;
	float rightAngle;
	float bottomAngle;
};

//Class
class ImageGrabber{
public:
	//constructor
	//create with or without choosing the camera resolution
	// 4VGA (1280* 960 ) = AL::k4VGA  = 3
	// VGA (640 * 480 )  = AL::kVGA   = 2
 	// QVGA (320 * 240)  = AL::kQVGA  = 1
	// QQVGA (160 * 120) = AL::kQQVGA  = 0
	//also set color space, default is HSY, other common possibilities are AL::kYUV422ColorSpace (native) (or AL::kYuvColorSpace for single channel
	//, AL::kRGBColorSpace, AL::kBGRColorSpace
	// defaults to bottom camera
	// see http://www.aldebaran-robotics.com/documentation/naoqi/vision/alvideodevice-indepth.html
	ImageGrabber(std::string robotIP, int cameraType = AL::k4VGA, int cameraColorSpace = AL::kYuvColorSpace, int FPS = 30); 
	~ImageGrabber();

	ImageContainer* pollImage();

	void setRobotIP(std::string robotIP);
	void setCameraType(int cameraType);
	void setCameraColorSpace(int cameraColorSpace);
	void setFPS(int FPS);
	void setUsedCamera(int cameraNumber); //AL::kTopCamera = 0, AL::kBottomCamera = 1
	
	cv::Size getImageSize();

private:
	void setCamProxy();
	void destroyCameraProxy(); //destroys the proxy attached to AL::ALVideoDeviceProxy * cameraProxy;
	
	AL::ALVideoDeviceProxy * cameraProxy;
	AL::ALValue polledImage;
	std::string robotIP;
	std::string clientDefaultInputName;
	std::string clientName;
	int cameraType; //which camera to use
	int cameraColorSpace;
	int FPS;
};

#endif _IMAGEGRABBER
