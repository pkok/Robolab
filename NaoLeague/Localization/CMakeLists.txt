project(main)
find_package(OpenCV REQUIRED)
add_executable(main  InputGenerator.cpp InputGenerator.h main.cpp FeatureMap.h FeatureMap.cpp LocationVisualizer.h LocationVisualizer.cpp  ParticleFilter.cpp ParticleFilter.h Particle.cpp Particle.h)
target_link_libraries(main ${OpenCV_LIBS})
