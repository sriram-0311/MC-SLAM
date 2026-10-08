// Offline check of the stereo-inertial alignment on EuRoC: runs alignVisualInertial on a window of a
// stereo-VO keyframe trajectory (body frame, TUM format) with the raw IMU, and compares the gyroscope
// bias, gravity direction and speeds against the EuRoC ground truth.
//
//   vi_align_check <traj_body.txt> <mav0_dir> [start_offset_s=0] [window_s=2.0]

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <gtsam/navigation/PreintegrationParams.h>

#include "MCSlam/VisualInertialAlignment.h"

using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Vector3;

namespace {

std::vector<std::vector<double>> readCsv(const std::string& path) {
    std::vector<std::vector<double>> rows;
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#')
            continue;
        std::stringstream ss(line);
        std::string tok;
        std::vector<double> row;
        while (std::getline(ss, tok, ','))
            row.push_back(std::stod(tok));
        rows.push_back(row);
    }
    return rows;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: vi_align_check <traj_body.txt> <mav0_dir> [start_offset_s] [window_s]\n";
        return 1;
    }
    const std::string mav0 = argv[2];
    const double offset = argc > 3 ? std::stod(argv[3]) : 0.0;
    const double window = argc > 4 ? std::stod(argv[4]) : 2.0;

    struct Kf { double t; Pose3 T; };
    std::vector<Kf> traj;
    {
        std::ifstream f(argv[1]);
        double t, x, y, z, qx, qy, qz, qw;
        while (f >> t >> x >> y >> z >> qx >> qy >> qz >> qw)
            traj.push_back({t, Pose3(Rot3::Quaternion(qw, qx, qy, qz), gtsam::Point3(x, y, z))});
    }
    if (traj.empty()) {
        std::cerr << "empty trajectory\n";
        return 1;
    }

    mcslam::ViAlignmentInput in;
    const double t0 = traj.front().t + offset;
    for (const auto& k : traj) {
        if (k.t >= t0 && k.t <= t0 + window) {
            in.kf_times.push_back(k.t);
            in.w_T_b.push_back(k.T);
        }
    }
    for (const auto& r : readCsv(mav0 + "/imu0/data.csv")) {
        mcslam::ImuSample m;
        m.timestamp = r[0] * 1e-9;
        m.gyro = Vector3(r[1], r[2], r[3]);
        m.accel = Vector3(r[4], r[5], r[6]);
        if (m.timestamp >= t0 - 0.1 && m.timestamp <= t0 + window + 0.1)
            in.imu.push_back(m);
    }
    // EuRoC ADIS16448 noise densities (imu0/sensor.yaml)
    in.params = gtsam::PreintegrationParams::MakeSharedU(9.81);
    in.params->setAccelerometerCovariance(gtsam::I_3x3 * std::pow(2.0e-3, 2));
    in.params->setGyroscopeCovariance(gtsam::I_3x3 * std::pow(1.6968e-4, 2));
    in.params->setIntegrationCovariance(gtsam::I_3x3 * 1e-8);

    // diagnostic: report the residuals with the quality gates disabled
    mcslam::ViAlignmentOptions opt;
    opt.max_rotation_rms_deg = 1e9;
    opt.max_position_rms = 1e9;
    const mcslam::ViAlignmentResult res = mcslam::alignVisualInertial(in, opt);
    std::printf("window %.2f s from +%.2f s: %zu keyframes, %zu IMU samples\n",
                in.kf_times.empty() ? 0.0 : in.kf_times.back() - in.kf_times.front(), offset,
                in.kf_times.size(), in.imu.size());
    std::printf("unconstrained |g| = %.3f  rot_rms %.3f deg  pos_rms %.4f m\n", res.gravity_norm_unconstrained,
                res.rotation_rms_deg, res.position_rms);
    if (!res.success) {
        std::printf("REJECTED: %s\n", res.reason.c_str());
        return 2;
    }

    // Ground truth at the keyframes (nearest sample): body rotation, velocity and gyro bias.
    const auto gt = readCsv(mav0 + "/state_groundtruth_estimate0/data.csv");
    Vector3 g_dir_true = Vector3::Zero(), bg_true = Vector3::Zero();
    double speed_err = 0.0;
    size_t j = 0;
    for (size_t i = 0; i < in.kf_times.size(); ++i) {
        const double t = in.kf_times[i];
        while (j + 1 < gt.size() && gt[j + 1][0] * 1e-9 <= t)
            ++j;
        const auto& r = gt[j];
        const Rot3 W_R_b = Rot3::Quaternion(r[4], r[5], r[6], r[7]);
        // gravity (-z in the GT world) seen in the body frame, then expressed in the VO world
        g_dir_true += in.w_T_b[i].rotation() * W_R_b.unrotate(-Vector3::UnitZ());
        bg_true += Vector3(r[11], r[12], r[13]);
        speed_err += std::abs(res.velocities[i].norm() - Vector3(r[8], r[9], r[10]).norm());
    }
    const size_t n = in.kf_times.size();
    g_dir_true.normalize();
    bg_true /= static_cast<double>(n);
    const Vector3 g_dir_est = res.g_R_w.unrotate(-Vector3::UnitZ());
    const double g_err_deg = std::acos(std::min(1.0, g_dir_est.dot(g_dir_true))) * 180.0 / M_PI;
    std::printf("gyro bias est  [%.4f %.4f %.4f]\n", res.gyro_bias.x(), res.gyro_bias.y(), res.gyro_bias.z());
    std::printf("gyro bias true [%.4f %.4f %.4f]  error %.4f rad/s\n", bg_true.x(), bg_true.y(), bg_true.z(),
                (res.gyro_bias - bg_true).norm());
    std::printf("gravity direction error %.2f deg\n", g_err_deg);
    std::printf("mean |speed| error %.3f m/s\n", speed_err / static_cast<double>(n));
    return 0;
}
