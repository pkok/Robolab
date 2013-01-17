project(feature_extraction)
find_package(OpenCV REQUIRED)
add_executable(feature_extraction feature_extraction.cpp)
target_link_libraries(feature_extraction ${OpenCV_LIBS})
