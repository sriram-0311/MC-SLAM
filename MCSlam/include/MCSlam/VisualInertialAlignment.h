#pragma once

#include <memory>
#include <string>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/PreintegrationParams.h>

#include "MCDataUtils/SensorTypes.h"

namespace mcslam {

/// Input of the stereo-inertial alignment: keyframe body (IMU) poses from stereo visual odometry,
/// expressed in an arbitrary metric world frame, and the IMU samples covering them.
struct ViAlignmentInput {
    std::vector<double> kf_times;          ///< keyframe timestamps [s], increasing
    std::vector<gtsam::Pose3> w_T_b;       ///< body poses in the VO world frame, one per keyframe
    std::vector<ImuSample> imu;            ///< IMU samples (body frame) spanning the keyframes
    std::shared_ptr<gtsam::PreintegrationParams> params;  ///< noise model; n_gravity sets |g|
};

struct ViAlignmentResult {
    bool success = false;
    std::string reason;                    ///< why the alignment was rejected
    gtsam::Vector3 gyro_bias = gtsam::Vector3::Zero();
    gtsam::Rot3 g_R_w;                     ///< rotates the VO world so that gravity points along -z
    std::vector<gtsam::Vector3> velocities;  ///< keyframe velocities in the gravity-aligned frame
    double gravity_norm_unconstrained = 0.0;  ///< |g| before the magnitude is imposed (sanity check)
    double rotation_rms_deg = 0.0;         ///< RMS of visual vs. preintegrated rotation after the bias fit
    double position_rms = 0.0;             ///< RMS [m] of the position equations of the final fit
};

struct ViAlignmentOptions {
    double min_duration = 1.5;             ///< [s] window length needed for a well-posed solve
    double max_gravity_norm_error = 0.1;   ///< relative error of the unconstrained |g| to accept
    double max_rotation_rms_deg = 0.35;    ///< reject windows whose rotations disagree (VO glitch)
    double max_position_rms = 0.03;        ///< [m] reject windows the motion model cannot explain
    int gyro_bias_iterations = 3;
    int gravity_refine_iterations = 3;
};

/// Stereo-inertial initialization in the spirit of VINS-Mono / ORB-SLAM3, simplified by the metric
/// scale stereo provides: (1) gyroscope bias from preintegrated vs. visual rotations, (2) keyframe
/// velocities and gravity from a linear least-squares problem, (3) gravity refined on its sphere of
/// known magnitude. The accelerometer bias is left at zero for the factor graph to estimate.
ViAlignmentResult alignVisualInertial(const ViAlignmentInput& in, const ViAlignmentOptions& opt = {});

}  // namespace mcslam
