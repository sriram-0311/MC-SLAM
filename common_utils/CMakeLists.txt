cmake_minimum_required(VERSION 3.15)
project(common_utils VERSION 1.0.0 LANGUAGES CXX)

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

# Create the library
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRC} ${${PROJECT_NAME}_HEADERS})

# Link against the global dependencies interface
target_link_libraries(${PROJECT_NAME} 
    PUBLIC MCSlam_Dependencies
)

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
