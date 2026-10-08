#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "MCSlam/VisualInertialAlignment.h"

using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Vector3;

namespace {

// Smooth 3D trajectory in a gravity-aligned world (gravity = -z): a circle with a height wave and
// yaw/pitch/roll oscillations, so every rotation axis and the gravity direction are excited.
struct Trajectory {
    double r = 1.5, w = 0.8;
    Point3 p(double t) const { return {r * std::cos(w * t), r * std::sin(w * t), 0.3 * std::sin(2 * w * t)}; }
    Vector3 v(double t) const {
        return {-r * w * std::sin(w * t), r * w * std::cos(w * t), 0.6 * w * std::cos(2 * w * t)};
    }
    Vector3 a(double t) const {
        return {-r * w * w * std::cos(w * t), -r * w * w * std::sin(w * t), -1.2 * w * w * std::sin(2 * w * t)};
    }
    Rot3 R(double t) const {
        return Rot3::Ypr(w * t + 0.3, 0.2 * std::sin(1.3 * t), 0.15 * std::cos(0.9 * t));
    }
    Vector3 omega_body(double t) const {
        const double h = 1e-6;
        return Rot3::Logmap(R(t - h).between(R(t + h))) / (2 * h);
    }
};

struct Synthetic {
    mcslam::ViAlignmentInput in;
    std::vector<Vector3> v_true;  // world (gravity-aligned) velocities at keyframes
    Rot3 vo_R_true;               // the unknown rotation between the true world and the VO frame
};

Synthetic makeSynthetic(const Vector3& gyro_bias, double gyro_noise = 0.0, unsigned seed = 1) {
    const Trajectory traj;
    const double g = 9.81, imu_rate = 1000.0, kf_period = 0.2, duration = 2.4;
    Synthetic s;
    s.in.params = gtsam::PreintegrationParams::MakeSharedU(g);
    s.in.params->setAccelerometerCovariance(gtsam::I_3x3 * 1e-4);
    s.in.params->setGyroscopeCovariance(gtsam::I_3x3 * 1e-6);
    s.in.params->setIntegrationCovariance(gtsam::I_3x3 * 1e-8);
    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, 1.0);
    const Vector3 g_world(0, 0, -g);
    for (int k = 0; k <= static_cast<int>(duration * imu_rate) + 10; ++k) {
        // sample at the middle of its hold interval so zero-order-hold integration is 2nd-order accurate
        const double t = k / imu_rate;
        const double tm = t + 0.5 / imu_rate;
        const Rot3 R = traj.R(tm);
        mcslam::ImuSample m;
        m.timestamp = t;
        m.accel = R.unrotate(traj.a(tm) - g_world);
        m.gyro = traj.omega_body(tm) + gyro_bias + gyro_noise * Vector3(noise(rng), noise(rng), noise(rng));
        s.in.imu.push_back(m);
    }
    s.vo_R_true = Rot3::Ypr(0.7, -0.4, 1.1);  // VO world = arbitrary rotation of the gravity frame
    for (double t = 0.1; t <= duration + 1e-9; t += kf_period) {
        s.in.kf_times.push_back(t);
        const Pose3 w_T_b(traj.R(t), traj.p(t));
        s.in.w_T_b.push_back(Pose3(s.vo_R_true, Point3(0.2, -0.1, 0.4)) * w_T_b);
        s.v_true.push_back(traj.v(t));
    }
    return s;
}

}  // namespace

TEST(VisualInertialAlignment, RecoversGyroBiasGravityAndVelocities) {
    const Vector3 bg(0.02, -0.015, 0.01);
    const Synthetic s = makeSynthetic(bg);
    const mcslam::ViAlignmentResult res = mcslam::alignVisualInertial(s.in);
    ASSERT_TRUE(res.success) << res.reason;

    EXPECT_LT((res.gyro_bias - bg).norm(), 1e-3);
    EXPECT_NEAR(res.gravity_norm_unconstrained, 9.81, 0.05);

    // g_R_w * vo_R_true must map the true world's -z to -z, i.e. be a pure yaw rotation.
    const Rot3 yaw_only = res.g_R_w * s.vo_R_true;
    const Vector3 z = yaw_only * Vector3::UnitZ();
    EXPECT_LT(std::acos(std::min(1.0, z.z())) * 180.0 / M_PI, 0.2);  // gravity direction error [deg]

    for (size_t i = 0; i < s.v_true.size(); ++i)
        EXPECT_LT((res.velocities[i] - yaw_only * s.v_true[i]).norm(), 0.02) << "keyframe " << i;
}

TEST(VisualInertialAlignment, ToleratesGyroNoise) {
    const Vector3 bg(-0.01, 0.02, 0.005);
    const Synthetic s = makeSynthetic(bg, /*gyro_noise=*/0.01);
    const mcslam::ViAlignmentResult res = mcslam::alignVisualInertial(s.in);
    ASSERT_TRUE(res.success) << res.reason;
    EXPECT_LT((res.gyro_bias - bg).norm(), 3e-3);
}

TEST(VisualInertialAlignment, RejectsShortWindow) {
    Synthetic s = makeSynthetic(Vector3::Zero());
    s.in.kf_times.resize(3);
    s.in.w_T_b.resize(3);
    const mcslam::ViAlignmentResult res = mcslam::alignVisualInertial(s.in);
    EXPECT_FALSE(res.success);
}

TEST(VisualInertialAlignment, RejectsWindowNotCoveredByImu) {
    Synthetic s = makeSynthetic(Vector3::Zero());
    s.in.imu.resize(s.in.imu.size() / 2);  // IMU stream ends mid-window
    const mcslam::ViAlignmentResult res = mcslam::alignVisualInertial(s.in);
    EXPECT_FALSE(res.success);
}
