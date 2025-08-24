# MC-SLAM Windows Build Guide

This project has been adapted to build on Windows with MSVC, removing the original ROS/catkin dependencies.

## Prerequisites

### Required Dependencies
1. **Visual Studio 2019/2022** with C++ development tools
2. **CMake 3.15+**
3. **OpenCV 4.x** - Can be installed via vcpkg or manually
4. **Eigen3** - Can be installed via vcpkg
5. **Boost** - Can be installed via vcpkg

### Optional Dependencies (Recommended)
- **GTSAM 4.x** - For advanced SLAM functionality
- **vcpkg** - For easy package management

### Dependencies via vcpkg (Recommended)

Install vcpkg and then install the following packages:

```bash
# Install vcpkg dependencies
vcpkg install opencv4:x64-windows
vcpkg install eigen3:x64-windows  
vcpkg install boost:x64-windows
vcpkg install glog:x64-windows
vcpkg install gflags:x64-windows

# Optional packages
vcpkg install pangolin:x64-windows  # For visualization
vcpkg install geographiclib:x64-windows  # For GPS functionality
```

## Building the Project

### Option 1: Visual Studio IDE

1. Open Visual Studio
2. Choose "Open a local folder" and select the MC-SLAM directory
3. Visual Studio will automatically detect the CMakeLists.txt
4. Configure the project (it will use CMakeSettings.json)
5. Build the project using Build > Build All

### Option 2: Command Line

```bash
# Create build directory
mkdir build
cd build

# Configure with CMake (adjust paths as needed)
cmake .. -G "Visual Studio 17 2022" -A x64 ^
  -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake ^
  -DCMAKE_PREFIX_PATH="C:/opencv/build"

# Build
cmake --build . --config Release
```

## Configuration

### CMakeSettings.json

Update the `CMAKE_PREFIX_PATH` in CMakeSettings.json to point to your dependency installations:

```json
"CMAKE_PREFIX_PATH": "C:/vcpkg/installed/x64-windows;C:/opencv/build;C:/Program Files/gtsam"
```

### Build Options

- `BUILD_TESTS`: Enable/disable test modules (default: ON for Debug, OFF for Release)
- `BUILD_TEST_SCRIPTS`: Enable/disable test scripts (default: OFF)

## Key Changes from Original Linux Version

1. **Removed ROS/catkin dependencies** - The project now builds as a standard CMake project
2. **Added Windows-specific compiler flags** - MSVC optimizations and warning suppressions
3. **Graceful dependency handling** - Optional packages are detected and the build continues without them
4. **Modern CMake structure** - Uses target-based linking and interface libraries
5. **Conditional compilation** - Features are enabled/disabled based on available dependencies

## Project Structure

After building, you'll have:

- `bin/MCSlamapp.exe` - Main SLAM application
- `lib/` - Static libraries for each module
- `bin/params/` - Parameter files copied for convenience

## Troubleshooting

### Common Issues

1. **OpenCV not found**: Ensure OpenCV is properly installed and CMAKE_PREFIX_PATH is set
2. **Boost not found**: Install via vcpkg or set BOOST_ROOT environment variable
3. **GTSAM not found**: This is optional - the build will continue with limited functionality
4. **Python not found**: Python support is optional but recommended for full functionality

### Missing Dependencies

The build system is designed to gracefully handle missing optional dependencies:

- Without GTSAM: Advanced SLAM features will be disabled
- Without DBoW2: Loop closure functionality will be disabled  
- Without Pangolin: Visualization will be disabled
- Without Python: Some data processing features will be limited

Check the CMake configuration output to see which optional features are enabled.

## Running the Application

The main application is `MCSlamapp.exe`. You'll need to:

1. Prepare configuration files (see original documentation)
2. Ensure all required DLLs are in the PATH or next to the executable
3. Run from the directory containing parameter files

Example:
```bash
cd build/bin
./MCSlamapp.exe --config_file params/slam_config.cfg
```