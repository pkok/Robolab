cmake_minimum_required(VERSION 2.8)
project(feature_extraction)
find_package(OpenCV REQUIRED)
add_executable(feature_extraction
line_detection.h
line_detection.cpp
hough_line_detection.h
hough_line_detection.cpp
img_processing.h
img_processing.cpp
geometry_utils.h
geometry_utils.cpp
line_feature_detection.h
line_feature_detection.cpp
feature_extraction.cpp)

target_link_libraries(feature_extraction ${OpenCV_LIBS})
