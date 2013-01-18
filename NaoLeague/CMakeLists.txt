cmake_minimum_required(VERSION 2.8)
project(NaoLeague)

find_package(qibuild REQUIRED)
find_package(OpenCV REQUIRED)

add_subdirectory(${PROJECT_SOURCE_DIR}/CommonSource)
add_subdirectory(${PROJECT_SOURCE_DIR}/Utilities)
add_subdirectory(${PROJECT_SOURCE_DIR}/Vision)

# Create a executable named NaoLeague
# with the source file: soul.cpp
qi_create_bin(soul "soul.cpp")
#qi_create_bin(ImageGrabberTest "imageGrabberTest.cpp" imageGrabber.cpp imageGrabber.h)

qi_create_bin(ImageGrabberTest Vision/imageGrabberTest.cpp Vision/imageGrabber.cpp Vision/imageGrabber.h CommonSource/Macros.h)
qi_use_lib(ImageGrabberTest ALCOMMON ALVISION OPENCV2_CORE OPENCV2_HIGHGUI)

qi_create_bin(MakeVisualDataset Utilities/makeVisualDataset.cpp Vision/imageGrabber.cpp Vision/imageGrabber.h CommonSource/Macros.h)
qi_use_lib(MakeVisualDataset ALCOMMON ALPROXIES ALVALUE ALVISION OPENCV2_CORE OPENCV2_HIGHGUI)

#qi_create_bin(FeatureExtraction FeatureExtraction/feature_extraction.cpp)
##qi_use_lib(FeatureExtraction ALCOMMON ALVISION OPENCV2_CORE OPENCV2_HIGHGUI OPENCV2_IMGPROC)
#target_link_libraries(feature_extraction ${OpenCV_LIBS})
#qi_use_lib(FeatureExtraction ALCOMMON ALVISION ${OpenCV_LIBS})
