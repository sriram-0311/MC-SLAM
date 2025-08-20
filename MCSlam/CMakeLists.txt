cmake_minimum_required (VERSION 3.0.0)

project(MCSlam)
## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++17)
#add_compile_options(-fsanitize=leak)
#add_link_options(-fsanitize=leak)
#add_compile_options(-fsanitize-recover=address)
#add_link_options(-fsanitize-recover=address)

# These all from slam branch
#SET(OpenCV_DIR "~/software/opencv/build"  CACHE PATH "OpenCV directory")
#set(DBoW2_DIR "~/software/DBoW2/build" CACHE PATH "DBoW2 directory")
#set(opengv_DIR "~/software/opengv/build/" CACHE PATH "opengv directory")
#set(GTSAM_DIR "~/software/gtsam/build" CACHE PATH "Path to GTSAM root")
#set(GTSAM_UNSTABLE_DIR "~/software/gtsam/build"  )
#set(opengv_INC_DIR "~/software/opengv/include" CACHE PATH "Path to Eigen Directory")
#set(DLib_DIR "~/software/DLib/build" CACHE PATH "Path to DLib root Directory")

# To include the DLoopDetector build the folder using the following :
#cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install
#-DDBOW2_DIR=/home/$USER/catkin_ws/Third_party/DBoW2/build
#-DDLib_DIR=/home/$USER/catkin_ws/Third_party/DLib/build
#-DDBoW2_INCLUDE_DIR=/home/$USER/catkin_ws/Third_party/DBoW2/include
#-DLib_INCLUDE_DIR=/home/$USER/catkin_ws/Third_party/DLib/include
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O2 ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2 ")
##add_definitions(-DCOMPILEDWITHC11)
## fPIC flags, needed?
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
##add_definitions (-march=native) #This is what made openGV work for me.
#set(CMAKE_CXX_FLAGS "-msse3")
#
#
## FIND REQUIRED PACKAGES
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")
#
#find_package(Eigen REQUIRED)
##set(EIGEN_INCLUDE_DIR "/usr/include/eigen3" CACHE PATH "Path to Eigen Directory")
#message("EIGEN_INCLUDE_DIR :" ${EIGEN_INCLUDE_DIR})
#set(ADDITIONAL_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR}/unsupported)
#
##FIND Python
##set(Python_ADDITIONAL_VERSIONS "2.7")
#find_package(PythonLibs REQUIRED)
#message("---PYTHON--- ")
#message("Python Libraries: " ${PYTHON_LIBRARIES} )
#message("Python Include Dirs: " ${PYTHON_INCLUDE_DIRS})
#if (NOT PYTHONLIBS_FOUND)
#  message("Python not found! Python components will not be built.")
#endif()
#
#
## Find OpenCV
#find_package(OpenCV 4.2.0 REQUIRED)
#message("---OpenCV----")
## SET(OpenCV_DIR "/home/marley/neu_ws/Third_party/opencv/build")
## SET(OpenCV_LIBS "/home/marley/neu_ws/Third_party/opencv/install/lib")
## SET(OpenCV_INCLUDE_DIRS "//home/marley/neu_ws/Third_party/opencv/install/include")
#message("opencv root: " ${OpenCV_DIR})
#message("OpenCV Libraries: " ${OpenCV_LIBS})
#message("OpenCV Include Dirs : " ${OpenCV_INCLUDE_DIRS})
#if(NOT OpenCV_FOUND)
#  message("OpenCV not found!")
#endif()
#
## message(${DBoW2_DIR})
#message(${DBoW2_DIR})
#
##find package DBoW2
#find_package(DBoW2 REQUIRED)
#if(DBoW2_FOUND)
#  message("DBoW2 library found, using it from the system")
#  message(${DBoW2_DIR})
#  message(${DBoW2_INCLUDE_DIRS})
#else()
#  message("DBoW2 not found!")
#endif()
#
#find_package(fbow REQUIRED)
#if(fbow_FOUND)
#  message("fbow library found, using it from the system")
#  message(${fbow_DIR})
#  message(${fbow_INCLUDE_DIRS})
#  message(${fbow_LIBS})
##  set(BoW_LIBS ${fbow_LIBS})
##  set(BoW_INCLUDE_DIRS ${fbow_INCLUDE_DIRS})
##  set(BoW_DIR ${fbow_DIR})
#else()
#  message("fbow not found!")
#endif()
#
## message(${DLib_DIR})
#message(${DLib_DIR})
#
#find_package(DLib REQUIRED)
#if(DLib_FOUND)
#  message("DLib library found, using it from the system")
#  message("Dlib LIBS" ${DLib_LIBS})
#  message("Dlib includes" ${DLib_INCLUDE_DIRS})
#
#else()
#  message("DLib not found!")
#endif()
#
## DLoopDetector. Note: It does not have a cpp file (cause Templates) so no Libs to include.
#message(${DLoopDetector_DIR})
#
#find_package(DLoopDetector REQUIRED)
#if(DLoopDetector_FOUND)
#  message("DLoopDetector library found, using it from the system")
#  message("DLoopDetector includes" ${DLoopDetector_INCLUDE_DIRS})
#else()
#  message("DLoopDetector not found!")
#endif()
#
## message(${opengv_DIR})
#
#message(${opengv_DIR})
##find package opengv
#find_package(opengv REQUIRED)
#if(opengv_FOUND)
#  message("opengv library found, using it from the system")
#  message(${opengv_DIR})
#
#else()
#  message("opengv not found!")
#endif()
#
#find_package(Pangolin REQUIRED)
#if(Pangolin_FOUND)
#  message("Pangolin library found, using it from the system")
#  message(${Pangolin_DIR})
#
#else()
#  message("Pangolin not found!")
#endif()
#
#
## Boost Libraries
#find_package (Boost)
#if(Boost_FOUND)
#  message("Boost library found, using it from the system")
#  find_package (Boost COMPONENTS program_options filesystem system serialization json chrono REQUIRED)
#  set(Boost_GENERAL ${Boost_LIBRARIES})
#elseif(NOT Boost_FOUND)
#  message("Boost not found!")
#endif()
#
#
#
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
#set(GTSAM_LIBRARIES gtsam)
#
#
##include(GtsamMakeConfigFile)
#
#
## geographiclib
##
##SET(GeographicLib_DIR "/home/neural/catkin_ws/Third_party/geographiclib/build"  CACHE PATH "geographiclib directory")
##
#find_package(GeographicLib REQUIRED)
#if(GeographicLib_FOUND)
#  message("GeographicLib library found, using it from the system")
#  message(${GeographicLib_INCLUDE_DIRS})
#elseif(NOT GeographicLib_FOUND)
#  message("GeographicLib not found!")
#endif()
#
#
#include_directories( ${Boost_INCLUDE_DIR} )
#include_directories( ${PYTHON_INCLUDE_DIRS}  /usr/include/python3.10/)
#include_directories( ${ADDITIONAL_INCLUDE_DIRS}  ) #This is for eigen and eigne unsupported${EIGEN_INC_DIR}
#include_directories( ${OpenCV_INCLUDE_DIRS} )
#include_directories( ${DLib_INCLUDE_DIRS} ${DBoW2_INCLUDE_DIRS} ${DLoopDetector_INCLUDE_DIRS})
#include_directories( ${opengv_INC_DIR} )
#include_directories( ${Pangolin_INCLUDE_DIRS} )
#include_directories( ${GeographicLib_INCLUDE_DIRS})
#include_directories(${fbow_INCLUDE_DIRS})
#
##set(fbow_lib_path "/usr/local/lib/libfbow.so")
##set(fbow_lib_path_two "/usr/local/lib/libfbow.so.0.0")
##set(fbow_lib_path_three "/usr/local/lib/libfbow.so.0.0.1")
#
#set (LIBS ${OpenCV_LIBS} ${PYTHON_LIBRARIES} ${Boost_GENERAL} ${DLib_LIBS} ${DBoW2_LIBS} ${fbow_LIBS} ${Pangolin_LIBRARIES} opengv glog ${GeographicLib_LIBRARIES} )


#catkin
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        cv_bridge
        image_transport
        sensor_msgs
        common_utils
        MCDataUtils
        )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs common_utils
  DEPENDS OpenCV  DBoW2 fbow opengv DLib DLoopDetector
)

include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS} )

FILE(GLOB ${PROJECT_NAME}_SRC "src/*.cpp")
FILE(GLOB ELAS_SRC "include/MCSlam/elas/*.cpp")

add_library (${PROJECT_NAME} ${${PROJECT_NAME}_SRC} ${ELAS_SRC} include/MCSlam/relocalization.h
        include/MCSlam/Tracking.h
        src/Tracking.cpp
        src/newGPSFactor.cpp)
add_dependencies(${PROJECT_NAME}  ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${LIBS} ${catkin_LIBRARIES} gtsam gtsam_unstable  ${GeographicLib_LIBRARIES} )

message(${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#add_subdirectory(src/tests_misc)

