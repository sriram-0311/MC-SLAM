cmake_minimum_required(VERSION 3.20)
project(MCSlam VERSION 2.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

option(MCSLAM_WITH_VIEWER "Build the Pangolin 3D viewer" ON)
option(MCSLAM_BUILD_TESTS "Build unit tests" ON)

# Emit DT_RPATH instead of DT_RUNPATH so the libraries we were linked against
# win over whatever happens to be on LD_LIBRARY_PATH (e.g. another GTSAM install).
add_link_options("LINKER:--disable-new-dtags")

#############################################
# Dependencies
#############################################
find_package(Threads REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs features2d calib3d highgui video)
find_package(Boost 1.70 REQUIRED COMPONENTS program_options filesystem serialization json)
find_package(GTSAM 4.3 REQUIRED)
find_package(GTSAM_UNSTABLE 4.3 REQUIRED)
find_package(DBoW2 REQUIRED)
find_package(DLib REQUIRED)
find_package(opengv REQUIRED)
list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/geographiclib)  # Debian/Ubuntu ships only a Find module
find_package(GeographicLib REQUIRED)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)
find_package(DLoopDetector REQUIRED)
find_package(fbow REQUIRED CONFIG)

if(MCSLAM_WITH_VIEWER)
  find_package(Pangolin 0.8 REQUIRED)
  # Pangolin's exported targets don't carry the libepoxy include path.
  find_path(EPOXY_INCLUDE_DIR epoxy/gl.h REQUIRED)
endif()

# DBoW2 / DLib / DLoopDetector export plain variables rather than targets; wrap them once.
add_library(mcslam_bow INTERFACE)
target_include_directories(mcslam_bow SYSTEM INTERFACE
  ${DBoW2_INCLUDE_DIRS} ${DBoW2_INCLUDE_DIR}/DBoW2
  ${DLib_INCLUDE_DIRS} ${DLoopDetector_INCLUDE_DIRS})
target_link_libraries(mcslam_bow INTERFACE ${DBoW2_LIBRARIES} ${DLib_LIBRARIES})

# fbow's config calls include_directories()/link_directories() globally and
# lists every OpenCV module in fbow_LIBS; link only the library itself.
add_library(mcslam_fbow INTERFACE)
target_include_directories(mcslam_fbow SYSTEM INTERFACE ${fbow_INCLUDE_DIRS})
target_link_libraries(mcslam_fbow INTERFACE ${fbow_LIB_DIR}/libfbow.so)

#############################################
# Targets
#############################################
add_subdirectory(common_utils)
add_subdirectory(MCDataUtils)
add_subdirectory(MCSlam)
add_subdirectory(MCApps)

if(MCSLAM_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
