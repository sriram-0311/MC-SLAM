add_executable(mcslam_app
  src/mc_slam_app.cpp
  src/ParseSettings.cpp
)
target_include_directories(mcslam_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mcslam_app PRIVATE MCSlamCore Boost::program_options)
