#include "MCSlam/VisualInertialAlignment.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>
#include <gtsam/navigation/ImuFactor.h>

namespace mcslam {
namespace {

using gtsam::Matrix3;
using gtsam::Rot3;
using gtsam::Vector3;
using gtsam::imuBias::ConstantBias;

/// Integrates the samples overlapping [t0, t1) with a zero-order hold: each sample is applied until
/// the next one (or the interval end). The sample preceding t0 covers the gap up to the first one.
gtsam::PreintegratedImuMeasurements integrate(const std::vector<ImuSample>& imu, double t0, double t1,
                                              const std::shared_ptr<gtsam::PreintegrationParams>& params) {
    gtsam::PreintegratedImuMeasurements pim(params, ConstantBias());
    for (size_t k = 0; k < imu.size(); ++k) {
        const double start = std::max(imu[k].timestamp, t0);
        const double end = std::min(k + 1 < imu.size() ? imu[k + 1].timestamp : t1, t1);
        if (end <= start)
            continue;
        pim.integrateMeasurement(imu[k].accel, imu[k].gyro, end - start);
    }
    return pim;
}

/// Orthonormal basis of the plane orthogonal to the unit vector n.
Eigen::Matrix<double, 3, 2> tangentBasis(const Vector3& n) {
    const Vector3 a = std::abs(n.z()) < 0.9 ? Vector3::UnitZ() : Vector3::UnitX();
    const Vector3 b1 = n.cross(a).normalized();
    const Vector3 b2 = n.cross(b1);
    Eigen::Matrix<double, 3, 2> B;
    B << b1, b2;
    return B;
}

}  // namespace

ViAlignmentResult alignVisualInertial(const ViAlignmentInput& in, const ViAlignmentOptions& opt) {
    ViAlignmentResult res;
    const size_t n = in.kf_times.size();
    if (n < 3 || in.w_T_b.size() != n || !in.params) {
        res.reason = "need at least 3 keyframes with poses and IMU parameters";
        return res;
    }
    if (in.kf_times.back() - in.kf_times.front() < opt.min_duration) {
        res.reason = "window too short";
        return res;
    }
    if (in.imu.empty() || in.imu.front().timestamp > in.kf_times.front() ||
        in.imu.back().timestamp < in.kf_times.back()) {
        res.reason = "IMU samples do not cover the keyframe window";
        return res;
    }

    std::vector<gtsam::PreintegratedImuMeasurements> pims;
    std::vector<double> dts;
    for (size_t i = 0; i + 1 < n; ++i) {
        pims.push_back(integrate(in.imu, in.kf_times[i], in.kf_times[i + 1], in.params));
        dts.push_back(in.kf_times[i + 1] - in.kf_times[i]);
    }

    // 1. Gyroscope bias: Gauss-Newton on Log(dR(bg)^T * R_i^T R_j) using the bias Jacobian of the
    //    preintegrated rotation (first-order bias correction, valid for small biases).
    Vector3 bg = Vector3::Zero();
    for (int it = 0; it < opt.gyro_bias_iterations; ++it) {
        Matrix3 A = Matrix3::Zero();
        Vector3 b = Vector3::Zero();
        for (size_t i = 0; i + 1 < n; ++i) {
            gtsam::Matrix96 H;
            const gtsam::Vector9 delta = pims[i].biasCorrectedDelta(ConstantBias(Vector3::Zero(), bg), H);
            const Rot3 dR_imu = Rot3::Expmap(delta.head<3>());
            const Rot3 dR_vo = in.w_T_b[i].rotation().between(in.w_T_b[i + 1].rotation());
            const Vector3 r = Rot3::Logmap(dR_imu.between(dR_vo));
            // r(bg + d) ~ r - H_theta_bg * d
            const Matrix3 J = H.block<3, 3>(0, 3);
            A += J.transpose() * J;
            b += J.transpose() * r;
        }
        bg += A.ldlt().solve(b);
    }
    res.gyro_bias = bg;
    const ConstantBias bias(Vector3::Zero(), bg);
    {
        double sq = 0.0;
        for (size_t i = 0; i + 1 < n; ++i) {
            const Rot3 dR_imu = Rot3::Expmap(pims[i].biasCorrectedDelta(bias).head<3>());
            const Rot3 dR_vo = in.w_T_b[i].rotation().between(in.w_T_b[i + 1].rotation());
            sq += Rot3::Logmap(dR_imu.between(dR_vo)).squaredNorm();
        }
        res.rotation_rms_deg = std::sqrt(sq / static_cast<double>(n - 1)) * 180.0 / M_PI;
    }
    if (res.rotation_rms_deg > opt.max_rotation_rms_deg) {
        res.reason = "visual and inertial rotations disagree: RMS " + std::to_string(res.rotation_rms_deg) + " deg";
        return res;
    }

    // Bias-corrected preintegrated position/velocity deltas (body frame of keyframe i, no gravity).
    std::vector<Vector3> dp(n - 1), dv(n - 1);
    for (size_t i = 0; i + 1 < n; ++i) {
        const gtsam::Vector9 delta = pims[i].biasCorrectedDelta(bias);
        dp[i] = delta.segment<3>(3);
        dv[i] = delta.segment<3>(6);
    }

    // 2./3. Velocities and gravity in the VO world frame. Per keyframe pair (i, j = i+1):
    //   v_i dt + 1/2 g dt^2       = p_j - p_i - R_i dp_ij
    //   -v_i + v_j - g dt         = R_i dv_ij
    // First with g free (3 unknowns), then with g = |g| * (g_dir + B * dtheta) on the sphere.
    const double g_mag = in.params->n_gravity.norm();
    auto solve = [&](const Vector3* g_dir, Vector3& g_out, std::vector<Vector3>& v_out) {
        const int ng = g_dir ? 2 : 3;
        const int cols = 3 * static_cast<int>(n) + ng;
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6 * (n - 1), cols);
        Eigen::VectorXd b = Eigen::VectorXd::Zero(6 * (n - 1));
        Eigen::Matrix<double, 3, 2> B;
        if (g_dir)
            B = tangentBasis(*g_dir);
        for (size_t i = 0; i + 1 < n; ++i) {
            const int r = 6 * static_cast<int>(i);
            const double dt = dts[i];
            const Matrix3 R_i = in.w_T_b[i].rotation().matrix();
            const Vector3 p_i = in.w_T_b[i].translation(), p_j = in.w_T_b[i + 1].translation();
            Vector3 rhs_p = p_j - p_i - R_i * dp[i];
            Vector3 rhs_v = R_i * dv[i];
            A.block<3, 3>(r, 3 * i) = Matrix3::Identity() * dt;
            A.block<3, 3>(r + 3, 3 * i) = -Matrix3::Identity();
            A.block<3, 3>(r + 3, 3 * (i + 1)) = Matrix3::Identity();
            if (g_dir) {
                const Vector3 g0 = g_mag * *g_dir;
                rhs_p -= 0.5 * dt * dt * g0;
                rhs_v += dt * g0;
                A.block<3, 2>(r, 3 * n) = 0.5 * dt * dt * g_mag * B;
                A.block<3, 2>(r + 3, 3 * n) = -dt * g_mag * B;
            } else {
                A.block<3, 3>(r, 3 * n) = 0.5 * dt * dt * Matrix3::Identity();
                A.block<3, 3>(r + 3, 3 * n) = -dt * Matrix3::Identity();
            }
            b.segment<3>(r) = rhs_p;
            b.segment<3>(r + 3) = rhs_v;
        }
        const Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);
        const Eigen::VectorXd e = A * x - b;
        double sq = 0.0;
        for (size_t i = 0; i + 1 < n; ++i)
            sq += e.segment<3>(6 * i).squaredNorm();
        res.position_rms = std::sqrt(sq / static_cast<double>(n - 1));
        v_out.resize(n);
        for (size_t i = 0; i < n; ++i)
            v_out[i] = x.segment<3>(3 * i);
        g_out = g_dir ? Vector3(g_mag * (*g_dir + B * x.tail<2>())) : Vector3(x.tail<3>());
    };

    Vector3 g;
    std::vector<Vector3> v;
    solve(nullptr, g, v);
    res.gravity_norm_unconstrained = g.norm();
    if (std::abs(g.norm() - g_mag) > opt.max_gravity_norm_error * g_mag) {
        res.reason = "gravity magnitude inconsistent: |g| = " + std::to_string(g.norm());
        return res;
    }
    Vector3 g_dir = g.normalized();
    for (int it = 0; it < opt.gravity_refine_iterations; ++it) {
        solve(&g_dir, g, v);
        g_dir = g.normalized();
    }

    if (res.position_rms > opt.max_position_rms) {
        res.reason = "motion model does not fit the visual positions: RMS " + std::to_string(res.position_rms) + " m";
        return res;
    }

    // Rotation taking the estimated gravity direction to -z (yaw stays that of the VO frame).
    const Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(g_dir, -Vector3::UnitZ());
    res.g_R_w = Rot3(q.toRotationMatrix());
    res.velocities.resize(n);
    for (size_t i = 0; i < n; ++i)
        res.velocities[i] = res.g_R_w * v[i];
    res.success = true;
    return res;
}

}  // namespace mcslam
