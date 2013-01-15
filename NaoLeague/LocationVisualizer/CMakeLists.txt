project(LocationVisualizer)
find_package(OpenCV REQUIRED)
add_executable(LocationVisualizer LocationVisualizer.h LocationVisualizer.cpp)
target_link_libraries(LocationVisualizer ${OpenCV_LIBS})
