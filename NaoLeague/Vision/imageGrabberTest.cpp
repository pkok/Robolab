#include <iostream>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include "imageGrabber.h"

int main()
{
	ImageGrabber imageGrabber("169.254.32.200");
	ImageContainer* imageContainer;
	while (true){
		imageContainer = imageGrabber.pollImage();
		cv::imshow("Polled Image", imageContainer->image);
		cv::waitKey(1);
	}
}
