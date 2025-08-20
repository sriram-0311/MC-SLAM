cmake_minimum_required (VERSION 3.0.0)

project(MCDataUtils)
## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++17)

#add_compile_options(-fsanitize=leak)
#add_link_options(-fsanitize=leak)
#add_compile_options(-fsanitize-recover=address)
#add_link_options(-fsanitize-recover=address)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O2 ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2 ")
#
## Find OpenCV
#find_package(OpenCV REQUIRED)
#message("---OpenCV----")
#message("OpenCV Libraries: " ${OpenCV_LIBS})
#message("OpenCV Include Dirs: " ${OpenCV_INCLUDE_DIRS})
#if(NOT OpenCV_FOUND)
#  message("OpenCV not found!")
#endif()
#
#
## Boost Libraries
#find_package(Boost)
#if(Boost_FOUND)
#  find_package ( Boost COMPONENTS program_options filesystem system serialization chrono REQUIRED )
#  set(Boost_GENERAL ${Boost_LIBRARIES})
#elseif(NOT Boost_FOUND)
#  message("Boost not found!")
#endif()

#temporarily gtsam is needed
#message(${GTSAM_UNSTABLE_DIR})
#
#find_package(GTSAMCMakeTools)
## Find GTSAM components
#find_package(GTSAM REQUIRED) # Uses installed package
#message("gtsam include : " ${GTSAM_UNSTABLE_DIR})
#find_package(GTSAM_UNSTABLE REQUIRED)
#message("gtsam include : " ${GTSAM_INCLUDE_DIR})
#message("gtsam include : " ${GTSAM_UNSTABLE_INCLUDE_DIR})
#
#include_directories(${GTSAM_INCLUDE_DIR})
#include_directories( ${OpenCV_INCLUDE_DIRS} )
#include_directories( ${Boost_INCLUDE_DIR} )
#
#set(GTSAM_LIBRARIES gtsam)
#set (LIBS ${OpenCV_LIBS} ${Boost_GENERAL} glog)


#catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy rosbag message_filters
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
  common_utils
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs common_utils
  DEPENDS OpenCV
)

include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS} )

FILE(GLOB ${PROJECT_NAME}_SRC "src/*.cpp")

add_library (${PROJECT_NAME} ${${PROJECT_NAME}_SRC})
#        src/NavabilityParser.cpp)
add_dependencies(${PROJECT_NAME}  ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${LIBS} ${catkin_LIBRARIES} gtsam gtsam_unstable)

#add_executable(NavabilityParser src/NavabilityParser.cpp)
#add_dependencies(NavabilityParser  ${catkin_EXPORTED_TARGETS})
#target_link_libraries(NavabilityParser ${LIBS} ${catkin_LIBRARIES})


message(${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
