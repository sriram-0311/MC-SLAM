cmake_minimum_required (VERSION 3.0.0)

project(unit_tests)
#add_compile_options(-std=c++17)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
#
## FIND REQUIRED PACKAGES
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

#include_directories("/usr/local/include/eigen3")
#find_package(Eigen3 REQUIRED)
#message("EIGEN_INCLUDE_DIR :" ${EIGEN_INCLUDE_DIR})
#set(ADDITIONAL_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR}/unsupported)

#FIND Python
#set(Python_ADDITIONAL_VERSIONS "2.7")
#find_package(PythonLibs REQUIRED)
#message("---PYTHON--- ")
#message("Python Libraries: " ${PYTHON_LIBRARIES} )
#message("Python Include Dirs: " ${PYTHON_INCLUDE_DIRS})
#if (NOT PYTHONLIBS_FOUND)
#  message("Python not found! Python components will not be built.")
#endif()


# Find OpenCV
#find_package(OpenCV 4.2.0 REQUIRED)
#message("---OpenCV----")
#message("opencv root: " ${OpenCV_DIR})
#message("opencv root: " ${OpenCV_DIR})
#message("OpenCV Libraries: " ${OpenCV_LIBS})
#message("OpenCV Include Dirs: " ${OpenCV_INCLUDE_DIRS})
#if(NOT OpenCV_FOUND)
#  message("OpenCV not found!")
#endif()

#catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
  common_utils

)
find_package(yaml-cpp REQUIRED)
# find_package(new_custom_factors REQUIRED)

# Boost Libraries
#find_package(Boost)
#if(Boost_FOUND)
#  find_package ( Boost COMPONENTS program_options filesystem system serialization chrono REQUIRED )
#  set(Boost_GENERAL ${Boost_LIBRARIES})
#elseif(NOT Boost_FOUND)
#  message("Boost not found!")
#endif()



#message(${GTSAM_UNSTABLE_DIR})
#
## find_package(GTSAMCMakeTools)
## Find GTSAM components
#find_package(GTSAM REQUIRED) # Uses installed package
#find_package(GTSAM_UNSTABLE REQUIRED)
#include_directories(${GTSAM_INCLUDE_DIR})
#set(GTSAM_LIBRARIES gtsam)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        common_utils
        new_custom_factors
        )

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs common_utils new_custom_factors
  DEPENDS OpenCV  
)


#include_directories( ${Boost_INCLUDE_DIR} )
#include_directories( ${PYTHON_INCLUDE_DIRS}  /usr/include/python2.7/)
#include_directories( ${ADDITIONAL_INCLUDE_DIRS}  ) #This is for eigen and eigne unsupported${EIGEN_INC_DIR}
#include_directories( ${OpenCV_INCLUDE_DIRS} )

#include_directories( ${GeographicLib_INCLUDE_DIRS})
#include_directories("home/neural/catkin_ws/src/TRI-SLAM/test_scripts/new_custom_factors/src/newGPSFactor.h")
#
#set (LIBS ${OpenCV_LIBS} ${PYTHON_LIBRARIES} ${Boost_GENERAL} glog ${GeographicLib_LIBRARIES})

## fPIC flags, needed?
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
#add_definitions (-march=native) #This is what made openGV work for me.
#set(CMAKE_CXX_FLAGS "-msse3")

include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS} )

FILE(GLOB ${PROJECT_NAME}_SRC "src/*.cpp")


add_executable (test_newGPSFactor  src/testnewGPSFactor.cpp)
add_dependencies(test_newGPSFactor  ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_newGPSFactor  ${LIBS} ${catkin_LIBRARIES} gtsam gtsam_unstable  ${GeographicLib_LIBRARIES} yaml-cpp new_gps_factor)


add_executable (unit_newGPSFactor  src/unitnewGPSFactor.cpp
        src/unitnewGPSFactor.cpp)
add_dependencies(unit_newGPSFactor  ${catkin_EXPORTED_TARGETS})
target_link_libraries(unit_newGPSFactor  ${LIBS} ${catkin_LIBRARIES} gtsam gtsam_unstable  ${GeographicLib_LIBRARIES} yaml-cpp new_gps_factor CppUnitLite)

add_executable (testDVLFactor  src/testDVLFactor.cpp)
add_dependencies(testDVLFactor  ${catkin_EXPORTED_TARGETS})
target_link_libraries(testDVLFactor  ${LIBS} ${catkin_LIBRARIES} gtsam gtsam_unstable  ${GeographicLib_LIBRARIES} yaml-cpp dvlFactor CppUnitLite)


message( ${catkin_INCLUDE_DIRS})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
