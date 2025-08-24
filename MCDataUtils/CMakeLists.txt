cmake_minimum_required(VERSION 3.15)
project(MCDataUtils VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard if not already set by parent
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
endif()

# Include directories
include_directories(include)

# Collect source files
file(GLOB ${PROJECT_NAME}_SRC "src/*.cpp")
file(GLOB ${PROJECT_NAME}_HEADERS "include/${PROJECT_NAME}/*.h" "include/${PROJECT_NAME}/*.hpp")

# Platform-specific source exclusions
if(WIN32 OR BUILD_WINDOWS_NATIVE)
    # Exclude ROS-specific files on Windows
    list(FILTER ${PROJECT_NAME}_SRC EXCLUDE REGEX ".*RosDataReader\\.cpp$")
    list(FILTER ${PROJECT_NAME}_SRC EXCLUDE REGEX ".*RosbagParser\\.cpp$")
    list(FILTER ${PROJECT_NAME}_HEADERS EXCLUDE REGEX ".*RosDataReader\\.h$")
    list(FILTER ${PROJECT_NAME}_HEADERS EXCLUDE REGEX ".*RosbagParser\\.h$")
    
    message(STATUS "MCDataUtils: Excluding ROS-specific files for Windows build")
endif()

# Check if ROS sources exist and should be included
if(NOT WIN32 AND BUILD_WITH_ROS)
    # Keep ROS sources for Linux ROS build
    message(STATUS "MCDataUtils: Including ROS support")
else()
    message(STATUS "MCDataUtils: Building without ROS support")
endif()

# Create the library
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRC} ${${PROJECT_NAME}_HEADERS})

# Link dependencies
target_link_libraries(${PROJECT_NAME} 
    PUBLIC 
        MCSlam_Dependencies
        common_utils
)

# Platform-specific linking
if(WIN32)
    # Windows-specific libraries
    target_link_libraries(${PROJECT_NAME} PRIVATE
        ws2_32  # For network operations if needed
        winmm   # For multimedia timer functions
    )
endif()

# Add GTSAM support if available
if(TARGET gtsam)
    target_link_libraries(${PROJECT_NAME} PUBLIC gtsam)
    target_compile_definitions(${PROJECT_NAME} PUBLIC MCSLAM_GTSAM_AVAILABLE)
endif()
if(TARGET gtsam_unstable)
    target_link_libraries(${PROJECT_NAME} PUBLIC gtsam_unstable)
endif()

# Set include directories for this target
target_include_directories(${PROJECT_NAME}
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Compile definitions
target_compile_definitions(${PROJECT_NAME} PRIVATE
    MCDATAUTILS_BUILDING_DLL
)

if(WIN32)
    target_compile_definitions(${PROJECT_NAME} PUBLIC
        MCDATAUTILS_WINDOWS_BUILD=1
        NOMINMAX
        WIN32_LEAN_AND_MEAN
    )
endif()

# Add platform-specific compile features
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE
        /W3                     # Warning level 3
        /wd4267 /wd4244 /wd4305 # Suppress common conversion warnings
        /wd4996                 # Suppress deprecated function warnings
    )
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${PROJECT_NAME} PRIVATE
        -Wall -Wextra
        -Wno-unused-parameter
    )
endif()

# Installation rules
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
    PATTERN "RosDataReader.h" EXCLUDE
    PATTERN "RosbagParser.h" EXCLUDE
)

# Export targets
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE MCSlam::
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# Print configuration summary
message(STATUS "=== MCDataUtils Configuration ===")
message(STATUS "Windows Build: ${WIN32}")
message(STATUS "ROS Support: ${BUILD_WITH_ROS}")
message(STATUS "Source files: ${${PROJECT_NAME}_SRC}")
message(STATUS "=================================")
