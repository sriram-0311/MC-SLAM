#include "MCDataUtils/KalibrCalibration.h"

#include <stdexcept>

#include <glog/logging.h>

namespace mcslam {

namespace {

[[noreturn]] void fail(const std::string& path, const std::string& what) {
    throw std::runtime_error("Calibration " + path + ": " + what);
}

// Reads the top 3x4 block of a 4x4 row-major transform given as a list of rows.
void readTransform(const cv::FileNode& node, cv::Mat& R, cv::Mat& t, const std::string& path) {
    R = cv::Mat::eye(3, 3, CV_64F);
    t = cv::Mat::zeros(3, 1, CV_64F);
    if (node.empty()) return;
    if (node.size() != 4) fail(path, "T_cn_cnm1 must have 4 rows");
    for (int r = 0; r < 3; ++r) {
        const cv::FileNode row = node[r];
        if (row.size() != 4) fail(path, "T_cn_cnm1 rows must have 4 entries");
        for (int c = 0; c < 3; ++c) R.at<double>(r, c) = static_cast<double>(row[c]);
        t.at<double>(r, 0) = static_cast<double>(row[3]);
    }
}

}  // namespace

CameraRigCalibration loadKalibrCameraChain(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) fail(path, "cannot open file");

    CameraRigCalibration calib;
    for (int i = 0;; ++i) {
        const cv::FileNode cam = fs["cam" + std::to_string(i)];
        if (cam.empty()) break;

        std::string model, dist_model;
        cam["camera_model"] >> model;
        cam["distortion_model"] >> dist_model;
        if (model != "pinhole") fail(path, "cam" + std::to_string(i) + ": unsupported camera_model '" + model + "'");

        std::vector<double> intr, dc;
        std::vector<int> res;
        cam["intrinsics"] >> intr;
        cam["distortion_coeffs"] >> dc;
        cam["resolution"] >> res;
        if (intr.size() != 4 || res.size() != 2) fail(path, "cam" + std::to_string(i) + ": bad intrinsics/resolution");

        const cv::Size size(res[0], res[1]);
        if (i == 0) calib.image_size = size;
        else if (size != calib.image_size) fail(path, "all cameras must share one resolution");

        cv::Mat K = (cv::Mat_<double>(3, 3) << intr[0], 0, intr[2], 0, intr[1], intr[3], 0, 0, 1);

        // OpenCV order (k1 k2 p1 p2 k3). Equidistant (k1..k4) keeps the legacy
        // mapping used by the original reader.
        cv::Mat dist = cv::Mat::zeros(1, 5, CV_64F);
        if (dist_model == "radtan") {
            if (dc.size() > 5) fail(path, "too many radtan coefficients");
            for (size_t j = 0; j < dc.size(); ++j) dist.at<double>(0, j) = dc[j];
        } else if (dist_model == "equidistant") {
            for (size_t j = 0; j < std::min<size_t>(dc.size(), 3); ++j)
                dist.at<double>(0, j < 2 ? j : j + 2) = dc[j];
        } else {
            fail(path, "unsupported distortion_model '" + dist_model + "'");
        }

        cv::Mat Rk, tk;
        readTransform(cam["T_cn_cnm1"], Rk, tk, path);

        // Chain T_cn_cnm1 to express every camera relative to cam0.
        cv::Mat R = Rk.clone(), t = tk.clone();
        if (i > 0) {
            R = Rk * calib.R.back();
            t = Rk * calib.t.back() + tk;
        }
        cv::Mat Rt;
        cv::hconcat(R, t, Rt);

        calib.K.push_back(K);
        calib.dist.push_back(dist);
        calib.R_kalibr.push_back(Rk);
        calib.t_kalibr.push_back(tk);
        calib.R.push_back(R);
        calib.t.push_back(t);
        calib.P.push_back(K * Rt);
    }

    calib.num_cams = static_cast<int>(calib.K.size());
    if (calib.num_cams == 0) fail(path, "no cam0 node found");
    LOG(INFO) << "Loaded " << calib.num_cams << " camera(s) from " << path << " at "
              << calib.image_size.width << "x" << calib.image_size.height;
    return calib;
}

}  // namespace mcslam
