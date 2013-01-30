cmake_minimum_required(VERSION 2.8)
project(NaoLeague)

find_package(OpenCV REQUIRED)
find_package(qibuild REQUIRED)

add_subdirectory(${PROJECT_SOURCE_DIR}/CommonSource)
add_subdirectory(${PROJECT_SOURCE_DIR}/Utilities)
add_subdirectory(${PROJECT_SOURCE_DIR}/Vision)

# Create a executable named NaoLeague
# with the source file: soul.cpp
qi_create_bin(soul "soul.cpp")

qi_create_bin(FeatureDistanceProcessor 
  Vision/FeatureDistanceProcessor.cpp 
  Vision/FeatureDistance.h 
  Vision/FeatureDistance.cpp
  )
qi_use_lib(FeatureDistanceProcessor ALMATH ALPROXIES)

qi_create_bin(FeatureDistanceTest 
  Vision/FeatureDistanceTest.cpp 
  Vision/FeatureDistance.h 
  Vision/FeatureDistance.cpp
  )
qi_use_lib(FeatureDistanceTest ALMATH ALPROXIES)

qi_create_bin(ImageGrabberTest 
  Vision/imageGrabberTest.cpp 
  Vision/imageGrabber.cpp 
  Vision/imageGrabber.h 
  CommonSource/Macros.h
  )
qi_use_lib(ImageGrabberTest ALCOMMON ALVISION OPENCV2_CORE OPENCV2_HIGHGUI)

qi_create_bin(MakeVisualDataset 
  Utilities/makeVisualDataset.cpp 
  Vision/imageGrabber.cpp 
  Vision/imageGrabber.h 
  CommonSource/Macros.h
  )
qi_use_lib(MakeVisualDataset ALCOMMON ALPROXIES ALVALUE ALVISION OPENCV2_CORE OPENCV2_HIGHGUI)
