cmake_minimum_required(VERSION 2.8)
project(main)
find_package(OpenCV REQUIRED)
add_executable(main
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
ellipse_detector.h
ellipse_detector.cpp
goal_detection.h
goal_detection.cpp
feature_extraction.h
feature_extraction.cpp
main.cpp)

target_link_libraries(main ${OpenCV_LIBS})
