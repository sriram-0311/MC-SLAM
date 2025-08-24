cmake_minimum_required(VERSION 3.15)
project(test_imu_modules VERSION 1.0.0 LANGUAGES CXX)

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
file(GLOB ${PROJECT_NAME}_HEADERS "include/*.h" "include/*.hpp")

# Create the library (if there are source files)
if(${PROJECT_NAME}_SRC)
    add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRC} ${${PROJECT_NAME}_HEADERS})
    
    # Link dependencies
    target_link_libraries(${PROJECT_NAME} 
        PUBLIC 
            MCSlam_Dependencies
            common_utils
            MCDataUtils
            MCSlam
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
    
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
        install(DIRECTORY include/
            DESTINATION include
        )
    endif()
endif()

# Look for test executables in src directory
file(GLOB TEST_SOURCES "src/*test*.cpp" "src/*Test*.cpp")
foreach(TEST_SOURCE ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    
    target_link_libraries(${TEST_NAME} 
        PRIVATE
            MCSlam_Dependencies
            common_utils
            MCDataUtils
            MCSlam
    )
    
    if(TARGET ${PROJECT_NAME})
        target_link_libraries(${TEST_NAME} PRIVATE ${PROJECT_NAME})
    endif()
    
    # Install test executables
    install(TARGETS ${TEST_NAME}
        RUNTIME DESTINATION bin/tests
    )
endforeach()

