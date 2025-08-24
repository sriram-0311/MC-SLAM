cmake_minimum_required(VERSION 3.15)
project(MCSlam VERSION 1.0.0 LANGUAGES CXX)

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
file(GLOB ELAS_SRC "include/MCSlam/elas/*.cpp")
file(GLOB ${PROJECT_NAME}_HEADERS "include/${PROJECT_NAME}/*.h" "include/${PROJECT_NAME}/*.hpp")

# Create the library
add_library(${PROJECT_NAME} 
    ${${PROJECT_NAME}_SRC} 
    ${ELAS_SRC} 
    ${${PROJECT_NAME}_HEADERS}
    include/MCSlam/relocalization.h
    include/MCSlam/Tracking.h
    src/Tracking.cpp
    src/newGPSFactor.cpp
)

# Link dependencies
target_link_libraries(${PROJECT_NAME} 
    PUBLIC 
        MCSlam_Dependencies
        common_utils
        MCDataUtils
)

# Add GTSAM support if available
if(TARGET gtsam)
    target_link_libraries(${PROJECT_NAME} PUBLIC gtsam)
endif()
if(TARGET gtsam_unstable)
    target_link_libraries(${PROJECT_NAME} PUBLIC gtsam_unstable)
endif()

# Add GeographicLib if available
if(TARGET GeographicLib)
    target_link_libraries(${PROJECT_NAME} PUBLIC GeographicLib)
elseif(GeographicLib_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC ${GeographicLib_LIBRARIES})
endif()

# Set include directories for this target
target_include_directories(${PROJECT_NAME}
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Installation rules
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
)

# Export targets
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE MCSlam::
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# Optional: Add tests subdirectory if it exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/tests_misc/CMakeLists.txt" AND BUILD_TESTS)
    add_subdirectory(src/tests_misc)
endif()

