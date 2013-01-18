#include <iostream>

#include <alcommon/almodule.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <alvision/alvisiondefinitions.h>
#include <alvalue/alvalue.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../Vision/imageGrabber.h"

/* Save the camera image of Nao to file on a single mouse click. */

class EventWatcher : public AL::ALModule {
  public:
    static void head_callback(const std::string &key, const AL::ALValue &value);
    static void saveOnMouse(int event, int x, int y, int flags, void* userdata);
};

static bool left_mouse_clicked = false;
static bool head_touched = false;
static int click_count = 0;
//static AL::ALMemoryProxy memory;
//static AL::ALMotionProxy motion;

void EventWatcher::saveOnMouse(int event, int x, int y, int flags, void* userdata)
{
  if (event == CV_EVENT_LBUTTONDOWN) {
    left_mouse_clicked = true;
    click_count++;
  }
}


void EventWatcher::head_callback(const std::string &key, const AL::ALValue &value) {
  head_touched = true;
}


int main(int argc, char** argv)
{
  if (argc < 2) {
    std::cout << "Need at least 1 argument, the IP of the robot" << std::endl;
    return -1;
  }
  AL::ALMemoryProxy memory(argv[1]);
  AL::ALMotionProxy motion(argv[1]);
  AL::ALRobotPostureProxy rpp(argv[1]);
  AL::ALTextToSpeechProxy tts = AL::ALTextToSpeechProxy(argv[1]);
  std::string bullshit;

  //memory.subscribeToEvent("LeftBumperPressed", "EventWatcher", "head_callback");
  //memory.subscribeToEvent("RightBumperPressed", "EventWatcher", "head_callback");
  std::cout << "Starting picture id: ";
  std::cin >> click_count;
  std::getline(std::cin, bullshit);
  std::cout << "Ready! Type text to make me stand." << std::endl;
  tts.say("Ready! Type text to make me stand.");
  std::getline(std::cin, bullshit);
  motion.setStiffnesses("Body", 1.0);
  tts.say(bullshit);
  rpp.goToPosture("StandInit", 1.0f);
  //motion.moveInit();
  motion.setAngles("HeadPitch", -0.538476, 0.5f);
  motion.setAngles("HeadYaw", 0, 0.5f);
  std::cout << "Click on the image to save it. Then input extra data for the filename. Files are saved according to the following scheme:" << std::endl;
  std::cout << "  IMG_NUMBER USER_TEXT HEAD_PITCH HEAD_YAW RESOLUTION.png" << std::endl;

	ImageGrabber imageGrabber(argv[1], AL::k4VGA, AL::kBGRColorSpace);
	ImageContainer *imageContainer;
  cv::namedWindow("Polled Image", 0);
  cv::setMouseCallback("Polled Image", EventWatcher::saveOnMouse, NULL);

	while (true){
		imageContainer = imageGrabber.pollImage();
		cv::imshow("Polled Image", imageContainer->image);
    if (left_mouse_clicked) {
      char buffer[7];
      char head_yaw[11];
      char head_pitch[11];
      sprintf(buffer, "%06d ", click_count);
      std::string filename;
      std::cout << "Filename: ";
      std::getline(std::cin, filename);
      sprintf(head_pitch, " %f", motion.getAngles("HeadPitch", true).front());
      sprintf(head_yaw, " %f ", motion.getAngles("HeadYaw", true).front());
      filename = buffer + filename + head_pitch + head_yaw;
      cv::imwrite(filename + "4VGA.png", imageContainer->image);
      std::cout << "Wrote image to file: " << filename << std::endl;
      left_mouse_clicked = false;
    }
		cv::waitKey(1);
	}

  return 0;
}
