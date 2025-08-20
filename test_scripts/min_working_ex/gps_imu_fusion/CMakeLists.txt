cmake_minimum_required(VERSION 3.0.0)
project(gps_imu)
add_compile_options(-std=c++17)

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -O3")

add_executable(gps_imu_fusion src/gps_imu_fusion.cpp)
target_link_libraries(gps_imu_fusion ${GTSAM_LIBRARIES} ${catkin_LIBRARIES} ${Boost_GENERAL} ${OpenCV_LIBS} gflags glog yaml-cpp new_gps_factor)

add_executable(mwe_gps_imu_fusion src/gps_imu_fusion_mwe.cpp)
target_link_libraries(mwe_gps_imu_fusion ${GTSAM_LIBRARIES} ${catkin_LIBRARIES} ${Boost_GENERAL} ${OpenCV_LIBS} gflags glog yaml-cpp new_gps_factor)



