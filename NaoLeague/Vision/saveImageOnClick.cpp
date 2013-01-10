#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <alvision/alvisiondefinitions.h>
#include "imageGrabber.h"

/* Save the camera image of Nao to file on a single mouse click. */

static bool left_mouse_clicked = false;
static int click_count = 0;

static void saveOnMouse(int event, int x, int y, int flags, void* userdata)
{
  if (event == CV_EVENT_LBUTTONDOWN) {
    left_mouse_clicked = true;
    click_count++;
  }
}


int main(int argc, char** argv)
{
  if (argc < 2) {
    std::cout << "Need at least 1 argument, the IP of the robot" << std::endl;
    return -1;
  }
	ImageGrabber imageGrabber(argv[1], AL::kQVGA, AL::kBGRColorSpace);
	ImageContainer* imageContainer;
  cv::namedWindow("Polled Image", 0);
  cv::setMouseCallback("Polled Image", saveOnMouse, NULL);

	while (true){
		imageContainer = imageGrabber.pollImage();
		cv::imshow("Polled Image", imageContainer->image);
		cv::waitKey(1);
    if (left_mouse_clicked) {
      char filename[15];
      sprintf(filename, "img_%06d.jpg", click_count);
      cv::imwrite(filename, imageContainer->image);
      left_mouse_clicked = false;
    }
	}

  return 0;
}
