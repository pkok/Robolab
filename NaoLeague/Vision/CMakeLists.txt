cmake_minimum_required(VERSION 2.8)
project(Vision)

find_package(qibuild)

include_directories("${PROJECT_SOURCE_DIR}/../CommonSource")

#qi_create_bin(SaveImageOnClick saveImageOnClick.cpp imageGrabber.cpp imageGrabber.h CommonSource/Macros.h)
#qi_use_lib(SaveImageOnClick ALCOMMON ALVISION OPENCV2_CORE OPENCV2_HIGHGUI)
#qi_create_bin(SaveImageOnClick Vision/saveImageOnClick.cpp Vision/imageGrabber.cpp Vision/imageGrabber.h CommonSource/Macros.h)
#qi_use_lib(SaveImageOnClick ALCOMMON ALVISION OPENCV2_CORE OPENCV2_HIGHGUI)
