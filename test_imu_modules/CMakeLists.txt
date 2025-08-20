cmake_minimum_required (VERSION 3.0.0)

project(IMU_test_module)
add_compile_options(-std=c++17)
## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++17)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
##add_definitions(-DCOMPILEDWITHC11)



## FIND REQUIRED PACKAGES
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")
#
#include_directories("/usr/local/include/eigen3")
#find_package(Eigen3 REQUIRED)
#message("EIGEN_INCLUDE_DIR :" ${EIGEN_INCLUDE_DIR})
#set(ADDITIONAL_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR}/unsupported)
#
##FIND Python
#set(Python_ADDITIONAL_VERSIONS "2.7")
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

## Boost Libraries
#find_package(Boost)
#if(Boost_FOUND)
#  find_package ( Boost COMPONENTS program_options filesystem system serialization chrono REQUIRED )
#  set(Boost_GENERAL ${Boost_LIBRARIES})
#elseif(NOT Boost_FOUND)
#  message("Boost not found!")
#endif()
#
#
#
#message(${GTSAM_UNSTABLE_DIR})
#
## find_package(GTSAMCMakeTools)
## Find GTSAM components
#find_package(GTSAM REQUIRED) # Uses installed package
#message("gtsam include : " ${GTSAM_UNSTABLE_DIR})
#find_package(GTSAM_UNSTABLE REQUIRED)
#message("gtsam include : " ${GTSAM_INCLUDE_DIR})
#message("gtsam include : " ${GTSAM_UNSTABLE_INCLUDE_DIR})
#include_directories(${GTSAM_INCLUDE_DIR})
#set(GTSAM_LIBRARIES gtsam)


#include(GtsamMakeConfigFile)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs common_utils
  DEPENDS OpenCV  
)

#include_directories(include)
#include_directories( ${Boost_INCLUDE_DIR} )
#include_directories( ${PYTHON_INCLUDE_DIRS}  /usr/include/python2.7/)
#include_directories( ${ADDITIONAL_INCLUDE_DIRS}  ) #This is for eigen and eigne unsupported${EIGEN_INC_DIR}
#include_directories( ${OpenCV_INCLUDE_DIRS} )
#include_directories( ${catkin_INCLUDE_DIRS} )
#
#set (LIBS ${OpenCV_LIBS} ${PYTHON_LIBRARIES} ${Boost_GENERAL} glog)
#
## fPIC flags, needed?
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
#add_definitions (-march=native) #This is what made openGV work for me.
#set(CMAKE_CXX_FLAGS "-msse3")

FILE(GLOB ${PROJECT_NAME}_SRC "src/*.cpp")


add_executable (imu_bias_init  src/imu_unit_test1.cpp )
add_dependencies(imu_bias_init  ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_bias_init  ${LIBS} ${catkin_LIBRARIES} gtsam gtsam_unstable)

add_executable (imu_preinteg_test src/imu_preintegration_test.cpp)
add_dependencies(imu_preinteg_test  ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_preinteg_test  ${LIBS} ${catkin_LIBRARIES} gtsam gtsam_unstable)

add_executable (imu_vectorNav_test  src/imu_unit_test3.cpp )
add_dependencies(imu_vectorNav_test  ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_vectorNav_test  ${LIBS} ${catkin_LIBRARIES} gtsam gtsam_unstable)


# add_executable (kitti_gtsam_ex_test  src/kitti_gtsam_test.cpp )
# add_dependencies(kitti_gtsam_ex_test  ${catkin_EXPORTED_TARGETS})
# target_link_libraries(kitti_gtsam_ex_test  ${LIBS} ${catkin_LIBRARIES} gtsam gtsam_unstable)

# add_library (${PROJECT_NAME} ${${PROJECT_NAME}_SRC})
# add_dependencies(${PROJECT_NAME}  ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME} ${LIBS} ${catkin_LIBRARIES} gtsam gtsam_unstable)

message(${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# add_subdirectory(src/tests_misc)




# cmake_minimum_required(VERSION 2.8)
# project(IMU_test_module)

# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

# list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# # OpenCV
# find_package(OpenCV REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIRS})

# # Ceres
# find_package(Ceres REQUIRED)
# include_directories(${CERES_INCLUDE_DIRS})

# # g2o

# # Eigen
# include_directories("/usr/include/eigen3")



# # add_executable(ceresCurveFitting src/ceresCurveFitting.cpp)
# # target_link_libraries(ceresCurveFitting ${OpenCV_LIBS} ${CERES_LIBRARIES})

