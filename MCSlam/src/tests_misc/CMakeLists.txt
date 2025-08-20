cmake_minimum_required(VERSION 3.0.0)
#project(reduced_camera_system)
project(relocalization)
add_compile_options(-std=c++17)

find_package(yaml-cpp REQUIRED)

include_directories(include)
include_directories(${YAML_CPP_INCLUDE_DIR})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g -O3 -fsanitize=address")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -O3 -fsanitize=address")
#add_library(relocalization.h)
add_executable(relocalization Relocalization.cpp)
#target_link_libraries(reduced_camera_system ${GTSAM_LIBRARIES} ${Boost_GENERAL} ${OpenCV_LIBS} gflags glog yaml-cpp)
#target_link_libraries(relocalization ${GTSAM_LIBRARIES} ${Boost_GENERAL} ${OpenCV_LIBS} gflags glog yaml-cpp)



