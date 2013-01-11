cmake_minimum_required(VERSION 2.8)
project(NaoLeague)

find_package(qibuild)

add_subdirectory(${PROJECT_SOURCE_DIR}/Vision)
add_subdirectory(${PROJECT_SOURCE_DIR}/CommonSource)

# Create a executable named NaoLeague
# with the source file: soul.cpp
qi_create_bin(soul "soul.cpp")
#qi_create_bin(ImageGrabberTest "imageGrabberTest.cpp" imageGrabber.cpp imageGrabber.h)

qi_create_bin(ImageGrabberTest Vision/imageGrabberTest.cpp Vision/imageGrabber.cpp Vision/imageGrabber.h CommonSource/Macros.h)
qi_use_lib(ImageGrabberTest ALCOMMON ALVISION OPENCV2_CORE OPENCV2_HIGHGUI)

qi_create_bin(SaveImageOnClick Vision/saveImageOnClick.cpp Vision/imageGrabber.cpp Vision/imageGrabber.h CommonSource/Macros.h)
qi_use_lib(SaveImageOnClick ALCOMMON ALVISION OPENCV2_CORE OPENCV2_HIGHGUI)
