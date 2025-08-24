cmake_minimum_required (VERSION 3.15)

project(MCApps VERSION 1.0.0 LANGUAGES CXX)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++17)
#add_compile_options(-fsanitize=leak)
#add_link_options(-fsanitize=leak)
#add_compile_options(-fsanitize-recover=address)
#add_link_options(-fsanitize-recover=address)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   -Wall -O2 ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -O2 ")
#
## fPIC flags, needed?
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
#add_definitions (-march=native) #This is what made openGV work for me.

## CUDA
#find_package(CUDA)
#message("---CUDA---")
#message("CUDA Libraries: " ${CUDA_LIBRARIES})
#message("CUDA Include Dirs: " ${CUDA_INCLUDE_DIRS})
#if(NOT CUDA_FOUND)
#  message("CUDA not found!")
#endif()
#
##set(Python_ADDITIONAL_VERSIONS "2.7")
#find_package(PythonLibs REQUIRED)
#message("---PYTHON--- ")
#message("Python Libraries: " ${PYTHON_LIBRARIES})
#message("Python Include Dirs: " ${PYTHON_INCLUDE_DIRS})
#if (NOT PYTHONLIBS_FOUND)
#  message("Python not found! Python components will not be built.")
#endif()
#
## Find OpenCV
#find_package(OpenCV 4 REQUIRED)
#message("---OpenCV----")
#message("OpenCV Libraries: " ${OpenCV_LIBS})
#message("OpenCV Include Dirs: " ${OpenCV_INCLUDE_DIRS})
#if(NOT OpenCV_FOUND)
#  message("OpenCV not found!")
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
## FIND REQUIRED PACKAGES
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")
##FIND EIGEN
#find_package(Eigen REQUIRED)
#message("EIGEN_INCLUDE_DIR :" ${EIGEN_INCLUDE_DIR})
#set(ADDITIONAL_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR}/unsupported)
# set(PYTHON_INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS} /usr/local/lib/python3/dist-packages/numpy/core/include/numpy)

#set(PYTHON_INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS} /usr/lib/python3/dist-packages/numpy/core/include/numpy)
#include_directories( ${PYTHON_INCLUDE_DIRS}  /usr/include/python3.10/)
#include_directories( ${OpenCV_INCLUDE_DIRS} )
#include_directories( ${Boost_INCLUDE_DIR} )
#include_directories( ${ADDITIONAL_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS} /usr/lib/x86_64-linux-gnu/libpython3.10.so)
#
#set (LIBS ${OpenCV_LIBS} ${CUDA_LIBRARIES}  ${Boost_GENERAL} gflags glog)



#catkin
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy rosbag message_filters
#   std_msgs
#   cv_bridge
#   image_transport
#   sensor_msgs
#   common_utils
#   MCDataUtils
#   MCSlam
# )

# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
#   CATKIN_DEPENDS roscpp rospy std_msgs common_utils MCDataUtils MCSlam
#   DEPENDS OpenCV
# )

# include_directories(include)
# include_directories( ${catkin_INCLUDE_DIRS} )

# FILE(GLOB ${PROJECT_NAME}_SRC src/ParseSettings.cpp src/LiveViewer.cpp)

add_library(MCSlam_Dependencies INTERFACE)

# Create the main SLAM application executable
add_executable(MCSlamapp  
    src/mc_slam_app.cpp 
    ${${PROJECT_NAME}_SRC}
)

# Link dependencies
target_link_libraries(MCSlamapp 
    PRIVATE
        MCSlam_Dependencies
        common_utils
        MCDataUtils
        MCSlam
)

# Set include directories for this target
target_include_directories(MCSlamapp
    PRIVATE 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Installation rules for the executable
install(TARGETS MCSlamapp
    RUNTIME DESTINATION bin
)

# Optional: Create a MonoSlamApp if the source exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/MonoSlamApp.cpp")
    add_executable(MonoSlamApp  
        src/MonoSlamApp.cpp 
        ${${PROJECT_NAME}_SRC}
    )
    
    target_link_libraries(MonoSlamApp 
        PRIVATE
            MCSlam_Dependencies
            common_utils
            MCDataUtils
            MCSlam
    )
    
    target_include_directories(MonoSlamApp
        PRIVATE 
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    
    install(TARGETS MonoSlamApp
        RUNTIME DESTINATION bin
    )
endif()

# Copy parameter files to build directory for easier testing
file(GLOB PARAM_FILES "params/*")
if(PARAM_FILES)
    file(COPY ${PARAM_FILES} 
         DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/params)
endif()

