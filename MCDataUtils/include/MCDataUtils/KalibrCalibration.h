#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace mcslam {

/// Intrinsics and extrinsics of a camera rig, as consumed by CamArrayConfig.
struct CameraRigCalibration {
    int num_cams = 0;
    cv::Size image_size;
    std::vector<cv::Mat> K;         // 3x3
    std::vector<cv::Mat> dist;      // 1x5 (k1 k2 p1 p2 k3) for radtan
    std::vector<cv::Mat> R_kalibr;  // T_cn_cnm1 rotation (cam_i from cam_{i-1}); identity for cam0
    std::vector<cv::Mat> t_kalibr;  // T_cn_cnm1 translation
    std::vector<cv::Mat> R;         // cam_i from cam0
    std::vector<cv::Mat> t;         // cam_i from cam0
    std::vector<cv::Mat> P;         // K [R | t]
};

/**
 * Load the camera chain from a Kalibr-style YAML (camchain / camchain-imucam).
 *
 * Only top-level nodes named cam0, cam1, ... are treated as cameras, so the
 * same file may also carry `imu:` / `gps:` sections. All cameras must share
 * one resolution and use the pinhole model with radtan or equidistant
 * distortion. Throws std::runtime_error on malformed input.
 */
CameraRigCalibration loadKalibrCameraChain(const std::string& path);

}  // namespace mcslam
