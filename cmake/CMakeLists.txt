#####################################
## Add the content of this file to the
# ros workspace CMakeLists.txt before
# Building
###################################################################

## FIND REQUIRED PACKAGES
#add_subdirectory(${PROJECT_SOURCE_DIR}/TRI-SLAM/cmake/)

add_compile_options(-std=c++17)
add_compile_options(-fsanitize=leak)
add_link_options(-fsanitize=leak)
#add_compile_options(-fsanitize-recover=address)
#add_link_options(-fsanitize-recover=address)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O2 ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2 ")
#set(CMAKE_CXX_FLAGS "-msse3")

## FIND REQUIRED PACKAGES
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/TRI-SLAM/cmake/")
message(${PROJECT_SOURCE_DIR})
find_package(Eigen REQUIRED)
#set(EIGEN_INCLUDE_DIR "/usr/include/eigen3" CACHE PATH "Path to Eigen Directory")
message("EIGEN_INCLUDE_DIR :" ${EIGEN_INCLUDE_DIR})
set(ADDITIONAL_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR}/unsupported)


##FIND Python
##set(Python_ADDITIONAL_VERSIONS "2.7")
find_package(PythonLibs REQUIRED)
message("---PYTHON--- ")
message("Python Libraries: " ${PYTHON_LIBRARIES} )
message("Python Include Dirs: " ${PYTHON_INCLUDE_DIRS})
if (NOT PYTHONLIBS_FOUND)
  message("Python not found! Python components will not be built.")
endif()

# Find OpenCV
find_package(OpenCV 4.5.5 REQUIRED)
message("---OpenCV----")
message("opencv root: " ${OpenCV_DIR})
message("OpenCV Libraries: " ${OpenCV_LIBS})
message("OpenCV Include Dirs : " ${OpenCV_INCLUDE_DIRS})
if(NOT OpenCV_FOUND)
  message("OpenCV not found!")
endif()

#find package DBoW2
find_package(DBoW2 REQUIRED)
if(DBoW2_FOUND)
  message("DBoW2 library found, using it from the system")
  message(${DBoW2_DIR})
  message(${DBoW2_INCLUDE_DIRS})
else()
  message("DBoW2 not found!")
endif()
#
find_package(fbow REQUIRED)
if(fbow_FOUND)
  message("fbow library found, using it from the system")
  message(${fbow_DIR})
  message(${fbow_INCLUDE_DIRS})
  message(${fbow_LIBS})
  #  set(BoW_LIBS ${fbow_LIBS})
  #  set(BoW_INCLUDE_DIRS ${fbow_INCLUDE_DIRS})
  #  set(BoW_DIR ${fbow_DIR})
else()
  message("fbow not found!")
endif()
#
## message(${DLib_DIR})
#message(${DLib_DIR})
#
find_package(DLib REQUIRED)
if(DLib_FOUND)
  message("DLib library found, using it from the system")
  message("Dlib LIBS" ${DLib_LIBS})
  message("Dlib includes" ${DLib_INCLUDE_DIRS})

else()
  message("DLib not found!")
endif()
#
# DLoopDetector. Note: It does not have a cpp file (cause Templates) so no Libs to include.
message(${DLoopDetector_DIR})

find_package(DLoopDetector REQUIRED)
if(DLoopDetector_FOUND)
  message("DLoopDetector library found, using it from the system")
  message("DLoopDetector includes" ${DLoopDetector_INCLUDE_DIRS})
else()
  message("DLoopDetector not found!")
endif()


#find package opengv
find_package(opengv REQUIRED)
if(opengv_FOUND)
  message("opengv library found, using it from the system")
  message(${opengv_DIR})

else()
  message("opengv not found!")
endif()

find_package(Pangolin REQUIRED)
if(Pangolin_FOUND)
  message("Pangolin library found, using it from the system")
  message(${Pangolin_DIR})

else()
  message("Pangolin not found!")
endif()


# Boost Libraries
find_package (Boost)
if(Boost_FOUND)
  message("Boost library found, using it from the system")
  find_package (Boost COMPONENTS program_options filesystem system serialization json chrono REQUIRED)
  set(Boost_GENERAL ${Boost_LIBRARIES})
elseif(NOT Boost_FOUND)
  message("Boost not found!")
endif()

find_package(GTSAMCMakeTools)
# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
message("gtsam include : " ${GTSAM_UNSTABLE_DIR})
find_package(GTSAM_UNSTABLE REQUIRED)
message("gtsam include : " ${GTSAM_INCLUDE_DIR})
message("gtsam include : " ${GTSAM_UNSTABLE_INCLUDE_DIR})

set(GTSAM_LIBRARIES gtsam)


#include(GtsamMakeConfigFile)

find_package(GeographicLib REQUIRED)
if(GeographicLib_FOUND)
  message("GeographicLib library found, using it from the system")
  message(${GeographicLib_INCLUDE_DIRS})
elseif(NOT GeographicLib_FOUND)
  message("GeographicLib not found!")
endif()

find_package(TIFF REQUIRED)
if(NOT TIFF_FOUND)
  message("libtiff not found!")
endif()

#????????????
## CUDA
#find_package(CUDA)
#message("---CUDA---")
#message("CUDA Libraries: " ${CUDA_LIBRARIES})
#message("CUDA Include Dirs: " ${CUDA_INCLUDE_DIRS})
#if(NOT CUDA_FOUND)
#  message("CUDA not found!")
#endif()

#set(PYTHON_INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS} /usr/lib/python3/dist-packages/numpy/core/include/numpy)

include_directories(${GTSAM_INCLUDE_DIR})
include_directories(${GTSAM_UNSTABLE_INCLUDE_DIR})
include_directories( ${Boost_INCLUDE_DIR} )
include_directories( ${PYTHON_INCLUDE_DIRS}  /usr/include/python3.10/)
#include_directories( ${ADDITIONAL_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS} /usr/lib/x86_64-linux-gnu/libpython3.10.so)
include_directories( ${ADDITIONAL_INCLUDE_DIRS}  ) #This is for eigen and eigne unsupported${EIGEN_INC_DIR}
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${DLib_INCLUDE_DIRS} ${DBoW2_INCLUDE_DIRS} ${DLoopDetector_INCLUDE_DIRS})
include_directories( ${opengv_INC_DIR} )
include_directories( ${Pangolin_INCLUDE_DIRS} )
include_directories( ${GeographicLib_INCLUDE_DIRS})
include_directories(${fbow_INCLUDE_DIRS})
#
##set(fbow_lib_path "/usr/local/lib/libfbow.so")
##set(fbow_lib_path_two "/usr/local/lib/libfbow.so.0.0")
##set(fbow_lib_path_three "/usr/local/lib/libfbow.so.0.0.1")
#
#${CUDA_LIBRARIES} , /usr/lib/x86_64-linux-gnu/libpython3.10.so
set (LIBS ${OpenCV_LIBS} ${PYTHON_LIBRARIES} ${Boost_GENERAL} ${DLib_LIBS} ${DBoW2_LIBS} ${fbow_LIBS} ${Pangolin_LIBRARIES} opengv glog gflags ${GeographicLib_LIBRARIES} )
