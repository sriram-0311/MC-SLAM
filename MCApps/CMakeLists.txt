cmake_minimum_required (VERSION 3.15)

project(MCApps VERSION 1.0.0 LANGUAGES CXX)

# Platform-specific build options
option(BUILD_ROS_VERSION "Build ROS-compatible version (Linux only)" OFF)
option(BUILD_WINDOWS_VERSION "Build Windows-compatible version" ON)

if(WIN32)
    set(BUILD_ROS_VERSION OFF CACHE BOOL "ROS not supported on Windows" FORCE)
    set(BUILD_WINDOWS_VERSION ON CACHE BOOL "Windows version enabled" FORCE)
endif()

# Find ROS if building ROS version
if(BUILD_ROS_VERSION AND NOT WIN32)
    find_package(catkin QUIET COMPONENTS
        roscpp
        rospy
        rosbag
        message_filters
        std_msgs
        cv_bridge
        image_transport
        sensor_msgs
    )
    
    if(catkin_FOUND)
        catkin_package(
            INCLUDE_DIRS include
            LIBRARIES ${PROJECT_NAME}
            CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport sensor_msgs
            DEPENDS OpenCV Eigen3
        )
        set(ROS_SUPPORT_AVAILABLE TRUE)
    else()
        message(WARNING "ROS requested but not found, building Windows version instead")
        set(BUILD_ROS_VERSION OFF)
        set(BUILD_WINDOWS_VERSION ON)
        set(ROS_SUPPORT_AVAILABLE FALSE)
    endif()
else()
    set(ROS_SUPPORT_AVAILABLE FALSE)
endif()

# Source files common to both versions
set(COMMON_SOURCES
    src/ParseSettings.cpp
)

# Platform-specific source files
if(BUILD_WINDOWS_VERSION)
    set(WINDOWS_SOURCES
        src/mc_slam_app_windows.cpp
        ${COMMON_SOURCES}
    )
endif()

if(BUILD_ROS_VERSION AND ROS_SUPPORT_AVAILABLE)
    set(ROS_SOURCES
        src/mc_slam_app.cpp
        src/LiveViewer.cpp
        ${COMMON_SOURCES}
    )
endif()

# Create Windows-compatible SLAM application
if(BUILD_WINDOWS_VERSION)
    add_executable(MCSlamapp_Windows ${WINDOWS_SOURCES})
    
    target_link_libraries(MCSlamapp_Windows 
        PRIVATE
            MCSlam_Dependencies
            common_utils
            MCDataUtils
            MCSlam
    )
    
    target_include_directories(MCSlamapp_Windows
        PRIVATE 
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    
    # Windows-specific compile definitions
    target_compile_definitions(MCSlamapp_Windows PRIVATE
        MCSLAM_WINDOWS_BUILD=1
        MCSLAM_ROS_BUILD=0
    )
    
    # Set output name
    set_target_properties(MCSlamapp_Windows PROPERTIES OUTPUT_NAME "MCSlamapp")
    
    install(TARGETS MCSlamapp_Windows
        RUNTIME DESTINATION bin
    )
    
    message(STATUS "Building Windows-compatible MC-SLAM application")
endif()

# Create ROS-compatible SLAM application (Linux only)
if(BUILD_ROS_VERSION AND ROS_SUPPORT_AVAILABLE)
    add_executable(MCSlamapp_ROS ${ROS_SOURCES})
    
    target_link_libraries(MCSlamapp_ROS 
        PRIVATE
            MCSlam_Dependencies
            common_utils
            MCDataUtils
            MCSlam
            ${catkin_LIBRARIES}
    )
    
    target_include_directories(MCSlamapp_ROS
        PRIVATE 
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            ${catkin_INCLUDE_DIRS}
    )
    
    # ROS-specific compile definitions
    target_compile_definitions(MCSlamapp_ROS PRIVATE
        MCSLAM_WINDOWS_BUILD=0
        MCSLAM_ROS_BUILD=1
    )
    
    # Set output name
    set_target_properties(MCSlamapp_ROS PROPERTIES OUTPUT_NAME "MCSlamapp_ROS")
    
    install(TARGETS MCSlamapp_ROS
        RUNTIME DESTINATION bin
    )
    
    message(STATUS "Building ROS-compatible MC-SLAM application")
endif()

# Legacy support - create symlink or copy for backward compatibility
if(BUILD_WINDOWS_VERSION AND NOT BUILD_ROS_VERSION)
    # On Windows, the main executable is the Windows version
    message(STATUS "Main executable: MCSlamapp (Windows version)")
elseif(BUILD_ROS_VERSION AND ROS_SUPPORT_AVAILABLE)
    # On Linux with ROS, create both versions
    message(STATUS "Available executables: MCSlamapp (Windows-style), MCSlamapp_ROS (ROS)")
endif()

# Copy parameter files to build directory for easier testing
file(GLOB PARAM_FILES "params/*")
if(PARAM_FILES)
    file(COPY ${PARAM_FILES} 
         DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/params)
endif()

# Create example configuration files for Windows
if(BUILD_WINDOWS_VERSION)
    # Create a sample video configuration file
    set(EXAMPLE_VIDEO_CONFIG "# MC-SLAM Video Configuration\n")
    set(EXAMPLE_VIDEO_CONFIG "${EXAMPLE_VIDEO_CONFIG}# List video files or camera indices (one per line)\n")
    set(EXAMPLE_VIDEO_CONFIG "${EXAMPLE_VIDEO_CONFIG}# Examples:\n")
    set(EXAMPLE_VIDEO_CONFIG "${EXAMPLE_VIDEO_CONFIG}# 0  # Use camera 0\n")
    set(EXAMPLE_VIDEO_CONFIG "${EXAMPLE_VIDEO_CONFIG}# 1  # Use camera 1\n")
    set(EXAMPLE_VIDEO_CONFIG "${EXAMPLE_VIDEO_CONFIG}# C:/videos/cam0.mp4\n")
    set(EXAMPLE_VIDEO_CONFIG "${EXAMPLE_VIDEO_CONFIG}# C:/videos/cam1.mp4\n")
    set(EXAMPLE_VIDEO_CONFIG "${EXAMPLE_VIDEO_CONFIG}0\n")
    
    file(WRITE "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/params/video_config_example.txt" 
         "${EXAMPLE_VIDEO_CONFIG}")
endif()

# Print build summary
message(STATUS "=== MCApps Build Summary ===")
if(BUILD_WINDOWS_VERSION)
    message(STATUS "Windows version: ENABLED")
endif()
if(BUILD_ROS_VERSION AND ROS_SUPPORT_AVAILABLE)
    message(STATUS "ROS version: ENABLED")
elseif(BUILD_ROS_VERSION)
    message(STATUS "ROS version: REQUESTED but ROS not found")
else()
    message(STATUS "ROS version: DISABLED")
endif()
message(STATUS "==============================")

