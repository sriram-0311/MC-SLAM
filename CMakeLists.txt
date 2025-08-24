cmake_minimum_required(VERSION 3.15)
project(MC-SLAM-Workspace VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Windows/MSVC specific settings
if(WIN32)
    # Enable parallel compilation in MSVC
    add_compile_options(/MP)
    
    # Set runtime library to MultiThreaded DLL for release builds
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
    
    # Disable specific MSVC warnings
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
    add_compile_definitions(_SCL_SECURE_NO_WARNINGS)
    add_compile_definitions(NOMINMAX)
    add_compile_definitions(WIN32_LEAN_AND_MEAN)
    
    # Set warning level
    add_compile_options(/W3)
    
    # Optimization flags
    add_compile_options($<$<CONFIG:Release>:/O2>)
    add_compile_options($<$<CONFIG:Debug>:/Od>)
    
    # Suppress specific MSVC warnings
    add_compile_options(/wd4267 /wd4244 /wd4305 /wd4996)
else()
    # Linux/GCC specific settings
    add_compile_options(-Wall -O2)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()

# Set output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Add custom cmake modules path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules")
file(MAKE_DIRECTORY "${CMAKE_SOURCE_DIR}/cmake/modules")

#############################################
# DEPENDENCY MANAGEMENT
#############################################

# Find required packages
find_package(Threads REQUIRED)

# Eigen3
find_package(Eigen3 REQUIRED)
if(NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
endif()

# OpenCV (Required)
if(WIN32)
    # Windows-specific OpenCV handling
    find_package(OpenCV CONFIG REQUIRED)
    if(OpenCV_FOUND)
        message(STATUS "OpenCV found via CONFIG: ${OpenCV_VERSION}")
        message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
        message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
    else()
        message(FATAL_ERROR "OpenCV not found! Please install OpenCV via vcpkg: vcpkg install opencv4:x64-windows")
    endif()
else()
    # Linux OpenCV handling
    find_package(OpenCV REQUIRED)
    if(OpenCV_FOUND)
        message(STATUS "OpenCV found: ${OpenCV_VERSION}")
        message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
    else()
        message(FATAL_ERROR "OpenCV not found!")
    endif()
endif()

# Boost (Required)
find_package(Boost REQUIRED COMPONENTS 
    program_options 
    filesystem 
    system 
    serialization 
    chrono
)
if(Boost_VERSION VERSION_GREATER_EQUAL "1.70.0")
    find_package(Boost QUIET COMPONENTS json)
    if(TARGET Boost::json)
        set(BOOST_JSON_FOUND TRUE)
    endif()
endif()

# glog and gflags (Windows compatible)
if(WIN32)
    find_package(glog CONFIG QUIET)
    find_package(gflags CONFIG QUIET)
else()
    find_package(glog QUIET)
    find_package(gflags QUIET)
endif()

# Optional packages with graceful degradation
set(OPTIONAL_PACKAGES "")

# Python (optional)
find_package(Python3 COMPONENTS Interpreter Development QUIET)
if(Python3_FOUND)
    message(STATUS "Python3 found: ${Python3_VERSION}")
    list(APPEND OPTIONAL_PACKAGES "Python3")
else()
    message(WARNING "Python3 not found - some functionality may be limited")
endif()

# GTSAM (optional)
find_package(GTSAM QUIET)
if(GTSAM_FOUND)
    message(STATUS "GTSAM found")
    find_package(GTSAMCMakeTools QUIET)
    list(APPEND OPTIONAL_PACKAGES "GTSAM")
else()
    message(WARNING "GTSAM not found - advanced SLAM features will be disabled")
endif()

# DBoW2 (optional)
find_package(DBoW2 QUIET)
if(DBoW2_FOUND)
    message(STATUS "DBoW2 found")
    list(APPEND OPTIONAL_PACKAGES "DBoW2")
else()
    message(WARNING "DBoW2 not found - loop closure functionality will be disabled")
endif()

# OpenGV (optional)
find_package(opengv QUIET)
if(opengv_FOUND)
    message(STATUS "OpenGV found")
    list(APPEND OPTIONAL_PACKAGES "OpenGV")
else()
    message(WARNING "OpenGV not found - some geometric vision algorithms will be disabled")
endif()

# Pangolin (optional - for visualization)
find_package(Pangolin QUIET)
if(Pangolin_FOUND)
    message(STATUS "Pangolin found")
    list(APPEND OPTIONAL_PACKAGES "Pangolin")
else()
    message(WARNING "Pangolin not found - 3D visualization will be disabled")
endif()

# GeographicLib (optional)
find_package(GeographicLib QUIET)
if(GeographicLib_FOUND)
    message(STATUS "GeographicLib found")
    list(APPEND OPTIONAL_PACKAGES "GeographicLib")
else()
    message(WARNING "GeographicLib not found - GPS functionality may be limited")
endif()

# TIFF (optional)
find_package(TIFF QUIET)
if(TIFF_FOUND)
    message(STATUS "TIFF found")
    list(APPEND OPTIONAL_PACKAGES "TIFF")
endif()

#############################################
# ROS COMPATIBILITY OPTIONS
#############################################

option(BUILD_WITH_ROS "Build with ROS support (Linux only)" OFF)
option(BUILD_WINDOWS_NATIVE "Build Windows native version without ROS" ON)

if(WIN32)
    set(BUILD_WITH_ROS OFF CACHE BOOL "ROS not supported on Windows" FORCE)
    set(BUILD_WINDOWS_NATIVE ON CACHE BOOL "Windows native build" FORCE)
endif()

#############################################
# GLOBAL INCLUDES AND DEFINITIONS
#############################################

# Create a master include interface
add_library(MCSlam_Dependencies INTERFACE)

target_include_directories(MCSlam_Dependencies INTERFACE
    ${OpenCV_INCLUDE_DIRS}
    $<TARGET_PROPERTY:Eigen3::Eigen,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(MCSlam_Dependencies INTERFACE
    ${OpenCV_LIBS}
    Eigen3::Eigen
    Boost::program_options
    Boost::filesystem
    Boost::system
    Boost::serialization
    Boost::chrono
    Threads::Threads
)

# Add compile definitions
target_compile_definitions(MCSlam_Dependencies INTERFACE
    MCSLAM_WINDOWS_NATIVE=$<BOOL:${BUILD_WINDOWS_NATIVE}>
    MCSLAM_WITH_ROS=$<BOOL:${BUILD_WITH_ROS}>
)

# Add optional dependencies
if(Python3_FOUND)
    target_include_directories(MCSlam_Dependencies INTERFACE ${Python3_INCLUDE_DIRS})
    target_link_libraries(MCSlam_Dependencies INTERFACE ${Python3_LIBRARIES})
    target_compile_definitions(MCSlam_Dependencies INTERFACE MCSLAM_PYTHON_SUPPORT)
endif()

if(GTSAM_FOUND)
    target_link_libraries(MCSlam_Dependencies INTERFACE gtsam)
    if(TARGET gtsam_unstable)
        target_link_libraries(MCSlam_Dependencies INTERFACE gtsam_unstable)
    endif()
    target_compile_definitions(MCSlam_Dependencies INTERFACE MCSLAM_GTSAM_SUPPORT)
endif()

if(DBoW2_FOUND)
    target_include_directories(MCSlam_Dependencies INTERFACE ${DBoW2_INCLUDE_DIRS})
    target_link_libraries(MCSlam_Dependencies INTERFACE ${DBoW2_LIBS})
    target_compile_definitions(MCSlam_Dependencies INTERFACE MCSLAM_DBOW2_SUPPORT)
endif()

if(opengv_FOUND)
    target_link_libraries(MCSlam_Dependencies INTERFACE opengv)
    target_compile_definitions(MCSlam_Dependencies INTERFACE MCSLAM_OPENGV_SUPPORT)
endif()

if(Pangolin_FOUND)
    target_link_libraries(MCSlam_Dependencies INTERFACE ${Pangolin_LIBRARIES})
    target_compile_definitions(MCSlam_Dependencies INTERFACE MCSLAM_PANGOLIN_SUPPORT)
endif()

if(GeographicLib_FOUND)
    target_include_directories(MCSlam_Dependencies INTERFACE ${GeographicLib_INCLUDE_DIRS})
    target_link_libraries(MCSlam_Dependencies INTERFACE ${GeographicLib_LIBRARIES})
    target_compile_definitions(MCSlam_Dependencies INTERFACE MCSLAM_GEOGRAPHICLIB_SUPPORT)
endif()

if(glog_FOUND)
    if(WIN32)
        target_link_libraries(MCSlam_Dependencies INTERFACE glog::glog)
    else()
        target_link_libraries(MCSlam_Dependencies INTERFACE glog::glog)
    endif()
    target_compile_definitions(MCSlam_Dependencies INTERFACE MCSLAM_GLOG_SUPPORT)
endif()

if(gflags_FOUND)
    if(WIN32)
        target_link_libraries(MCSlam_Dependencies INTERFACE gflags::gflags)
    else()
        target_link_libraries(MCSlam_Dependencies INTERFACE gflags)
    endif()
    target_compile_definitions(MCSlam_Dependencies INTERFACE MCSLAM_GFLAGS_SUPPORT)
endif()

if(TIFF_FOUND)
    target_link_libraries(MCSlam_Dependencies INTERFACE ${TIFF_LIBRARIES})
    target_compile_definitions(MCSlam_Dependencies INTERFACE MCSLAM_TIFF_SUPPORT)
endif()

if(BOOST_JSON_FOUND)
    target_link_libraries(MCSlam_Dependencies INTERFACE Boost::json)
    target_compile_definitions(MCSlam_Dependencies INTERFACE MCSLAM_BOOST_JSON_SUPPORT)
endif()

#############################################
# ADD SUBDIRECTORIES
#############################################

# Add packages in dependency order
add_subdirectory(common_utils)
add_subdirectory(MCDataUtils)
add_subdirectory(MCSlam)
add_subdirectory(MCApps)

# Optional test modules
option(BUILD_TESTS "Build test modules" ON)
if(BUILD_TESTS)
    if(EXISTS "${CMAKE_SOURCE_DIR}/test_imu_modules/CMakeLists.txt")
        add_subdirectory(test_imu_modules)
    endif()
endif()

# Test scripts (optional)
option(BUILD_TEST_SCRIPTS "Build test scripts" OFF)
if(BUILD_TEST_SCRIPTS)
    if(EXISTS "${CMAKE_SOURCE_DIR}/test_scripts/gps_imu_fusion/CMakeLists.txt")
        add_subdirectory(test_scripts/gps_imu_fusion)
    endif()
    if(EXISTS "${CMAKE_SOURCE_DIR}/test_scripts/min_working_ex/CMakeLists.txt")
        add_subdirectory(test_scripts/min_working_ex)
    endif()
    if(EXISTS "${CMAKE_SOURCE_DIR}/test_scripts/new_custom_factors/CMakeLists.txt")
        add_subdirectory(test_scripts/new_custom_factors)
    endif()
    if(EXISTS "${CMAKE_SOURCE_DIR}/test_scripts/unit_tests/CMakeLists.txt")
        add_subdirectory(test_scripts/unit_tests)
    endif()
endif()

#############################################
# SUMMARY
#############################################

message(STATUS "=== MC-SLAM Build Configuration Summary ===")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "Platform: ${CMAKE_SYSTEM_NAME}")
message(STATUS "OpenCV Version: ${OpenCV_VERSION}")
message(STATUS "Windows Native Build: ${BUILD_WINDOWS_NATIVE}")
message(STATUS "ROS Support: ${BUILD_WITH_ROS}")
if(GTSAM_FOUND)
    message(STATUS "GTSAM: Found")
else()
    message(STATUS "GTSAM: Not Found")
endif()
message(STATUS "Optional packages found: ${OPTIONAL_PACKAGES}")
if(BUILD_TESTS)
    message(STATUS "Tests: Enabled")
else()
    message(STATUS "Tests: Disabled")
endif()
message(STATUS "============================================")