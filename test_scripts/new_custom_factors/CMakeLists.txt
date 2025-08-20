cmake_minimum_required (VERSION 3.0.0)

project(new_custom_factors)

# Boost Libraries
#find_package(Boost)
#if(Boost_FOUND)
#  find_package ( Boost COMPONENTS program_options filesystem system serialization chrono REQUIRED )
#  set(Boost_GENERAL ${Boost_LIBRARIES})
#elseif(NOT Boost_FOUND)
#  message("Boost not found!")
#endif()
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
#include_directories(${GTSAM_INCLUDE_DIR})
#set(GTSAM_LIBRARIES gtsam)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES new_gps_factor dvlFactor  # the default here is ${PROJECT_NAME}, but we have two libraries so we should enter both names
        CATKIN_DEPENDS roscpp rospy std_msgs common_utils
)

include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS} )

add_library(new_gps_factor STATIC
  src/newGPSFactor.cpp
        include/newGPSFactor.h
)

#target_include_directories(new_custom_factors
#  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
#)

target_link_libraries(new_gps_factor ${catkin_LIBRARIES} gtsam gtsam_unstable )

add_library(dvlFactor STATIC
        src/new_DVL_factor.cpp
        include/new_DVL_factor.h
        )
#
#target_include_directories(dvlFactor
#        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
#        )

target_link_libraries(dvlFactor ${catkin_LIBRARIES} gtsam gtsam_unstable )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
