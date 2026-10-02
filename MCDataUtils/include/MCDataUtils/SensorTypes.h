#pragma once

#include <Eigen/Core>

namespace mcslam {

/// One inertial measurement. Timestamp is in seconds.
struct ImuSample {
    double timestamp = 0.0;
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();  // m/s^2, sensor frame
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();   // rad/s, sensor frame
};

/// One GNSS position fix (WGS84). Timestamp is in seconds.
struct GnssFix {
    double timestamp = 0.0;
    double latitude = 0.0;   // deg
    double longitude = 0.0;  // deg
    double altitude = 0.0;   // m
};

}  // namespace mcslam
