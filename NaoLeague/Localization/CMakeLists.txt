project(main)
find_package(OpenCV REQUIRED)
add_executable(main main.cpp  ParticleFilter.cpp ParticleFilter.h Particle.cpp Particle.h)
target_link_libraries(main ${OpenCV_LIBS})
