add_executable(mcslam_app
  src/mc_slam_app.cpp
  src/ParseSettings.cpp
)
target_include_directories(mcslam_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mcslam_app PRIVATE MCSlamCore Boost::program_options)

# Offline check of the stereo-inertial alignment against EuRoC ground truth
add_executable(vi_align_check src/vi_align_check.cpp)
target_link_libraries(vi_align_check PRIVATE MCSlamCore)
