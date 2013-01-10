#include"imageGrabber.h"
#include <alproxies/alvideodeviceproxy.h>
#include "../CommonSource/Macros.h"
#include <opencv2/highgui/highgui.hpp>

ImageGrabber::ImageGrabber(std::string robotIP, int cameraType, int cameraColorSpace, int FPS){
	cameraProxy = NULL;
	//set default params
	this->robotIP					= robotIP; //set this objects robotIP to robotIP
	this->cameraType				= cameraType;//default on 4VGA camera, 1280*960 960p
	this->clientDefaultInputName	= "imageGrabberClient";
	this->cameraColorSpace			= cameraColorSpace;
	this->FPS						= FPS;
	this->setCamProxy();
}

ImageGrabber::~ImageGrabber(){
	destroyCameraProxy();
}

void ImageGrabber::setUsedCamera(int cameraNumber){
	DASSERT(cameraNumber == AL::kBottomCamera || cameraNumber == AL::kTopCamera, "cameraNumber is not valid");
	DASSERT(cameraProxy != NULL, "cameraProxy == NULL")
	cameraProxy->setActiveCamera(clientDefaultInputName, cameraNumber);
}

void ImageGrabber::destroyCameraProxy(){
	if(cameraProxy != NULL){
		try {
			cameraProxy->unsubscribe(clientDefaultInputName);
		}
		catch (AL::ALError e) {
		}
		delete cameraProxy;
	}
	cameraProxy = NULL;
}


void ImageGrabber::setCamProxy(){
	destroyCameraProxy();
	cameraProxy = new AL::ALVideoDeviceProxy(this->robotIP);
	try {
		cameraProxy->unsubscribe(clientDefaultInputName);
	}
	catch (AL::ALError e) {
	}
	clientName = cameraProxy->subscribe(clientDefaultInputName, cameraType, cameraColorSpace, FPS);
	cameraProxy->setActiveCamera(clientDefaultInputName, AL::kBottomCamera);
}

void ImageGrabber::setRobotIP(std::string robotIP){
	if(this->robotIP.compare(robotIP) != 0){ //true if the robotIPs are different
		this->robotIP = robotIP;
		setCamProxy();
	}
}

void ImageGrabber::setCameraType(int cameraType){
	DASSERT(cameraType >= AL::kQQVGA && cameraType <= AL::k4VGA, "Illegal cameraType");
	cameraProxy->resolutionToSizes(cameraType);
}

void ImageGrabber::setCameraColorSpace(int cameraColorSpace){
	DASSERT(cameraColorSpace >= AL::kYuvColorSpace && cameraColorSpace <= AL::kHSMixedColorSpace, "Illegal colorspace");
	cameraProxy->setColorSpace(clientName, cameraColorSpace);
}

void ImageGrabber::setFPS(int FPS){
	DASSERT(FPS > 0, "Illegal FPS");
	cameraProxy->setFrameRate(clientName, FPS);
}

//ALValue datastruct data found here:
//http://www.aldebaran-robotics.com/documentation/naoqi/vision/alvideodevice-api.html
ImageContainer* ImageGrabber::pollImage(){
	int numberOfLayers = AL::getNumLayersInColorSpace(this->cameraColorSpace);
	DASSERT(numberOfLayers >= 1 && numberOfLayers <= 3, "not a valid amount of layers");

	ImageContainer* processedImage = NULL;
	processedImage = new ImageContainer;
	switch(numberOfLayers){ //choose the correct number of channels
		case 1:	processedImage->image = cv::Mat(getImageSize(), CV_8UC1); break;	//create a cv::Mat to store the image in
		case 2:	processedImage->image = cv::Mat(getImageSize(), CV_8UC2); break;	//create a cv::Mat to store the image in
		case 3:	processedImage->image = cv::Mat(getImageSize(), CV_8UC3); break;	//create a cv::Mat to store the image in
	}

	this->polledImage						= cameraProxy->getImageRemote(clientName);	//data is returned as ALValue
	processedImage->image.data				= (uchar*)(polledImage[6].GetBinary());	//copy the data
	processedImage->cameraID				= static_cast<int>  (polledImage[7]);		//add the camera ID
	processedImage->timeStampSeconds		= static_cast<int>  (polledImage[4]);		// add timestamps
	processedImage->timeStampMicroSeconds	= static_cast<int>  (polledImage[5]);
	processedImage->leftAngle				= static_cast<float>(polledImage[8]);		//add angles of the image
	processedImage->topAngle				= static_cast<float>(polledImage[9]);
	processedImage->rightAngle				= static_cast<float>(polledImage[10]);
	processedImage->bottomAngle				= static_cast<float>(polledImage[11]);
	cameraProxy->releaseImage(clientName);									//return the image buffer to the driver
	return processedImage;
}

//return the parameters of the camera as indicated here:
//http://www.aldebaran-robotics.com/documentation/naoqi/vision/alvideodevice-api.html
//keep in mind that these have to be changed when the camera changes somehow...
cv::Size ImageGrabber::getImageSize(){
	int resolutionParameter = cameraProxy->getResolution(clientName);
	
	switch(resolutionParameter){ 
	case AL::kQQVGA: return cv::Size(160,120)  ; break;
	case AL::kQVGA : return cv::Size(320,240)  ; break;
	case AL::kVGA  : return cv::Size(640,480)  ; break;
	case AL::k4VGA : return cv::Size(1280,960) ; break;
	}
	
	return cv::Size(0,0);
}
