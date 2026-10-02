#include "MCDataUtils/EurocReader.h"

#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

#include <glog/logging.h>
#include <opencv2/imgcodecs.hpp>

#include "MCDataUtils/KalibrCalibration.h"

namespace fs = std::filesystem;

namespace mcslam {

namespace {

constexpr double kNsToSec = 1e-9;

// Splits "a,b,c" into fields, trimming surrounding whitespace and '\r'.
std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, ',')) {
        const auto b = field.find_first_not_of(" \t\r");
        const auto e = field.find_last_not_of(" \t\r");
        out.push_back(b == std::string::npos ? "" : field.substr(b, e - b + 1));
    }
    return out;
}

// Reads <cam_dir>/data.csv -> {timestamp_ns: absolute image path}.
std::map<int64_t, std::string> readCameraIndex(const fs::path& cam_dir) {
    const fs::path csv = cam_dir / "data.csv";
    std::ifstream in(csv);
    if (!in) throw std::runtime_error("EuRoC: cannot open " + csv.string());
    std::map<int64_t, std::string> index;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        const auto f = splitCsv(line);
        if (f.size() < 2) throw std::runtime_error("EuRoC: malformed line in " + csv.string() + ": " + line);
        index.emplace(std::stoll(f[0]), (cam_dir / "data" / f[1]).string());
    }
    return index;
}

}  // namespace

void EurocReader::initialize(MCDataUtilSettings s) {
    settings = s;

    fs::path root(settings.data_path);
    if (fs::exists(root / "mav0")) root /= "mav0";
    if (!fs::exists(root / "cam0" / "data.csv"))
        throw std::runtime_error("EuRoC: no cam0/data.csv under " + root.string());
    mav0_ = root.string();

    // Calibration.
    const CameraRigCalibration calib = loadKalibrCameraChain(settings.calib_file_path);
    num_cams_ = calib.num_cams;
    img_size_ = calib_img_size_ = calib.image_size;
    K_mats_ = calib.K;
    dist_coeffs_ = calib.dist;
    R_mats_ = calib.R;
    t_vecs_ = calib.t;
    R_mats_kalibr = calib.R_kalibr;
    t_vecs_kalibr = calib.t_kalibr;
    P_mats_ = calib.P;

    // Frames: keep only timestamps present in every camera stream.
    std::vector<std::map<int64_t, std::string>> cams;
    for (int c = 0; c < num_cams_; ++c) cams.push_back(readCameraIndex(root / ("cam" + std::to_string(c))));
    size_t dropped = 0;
    for (const auto& [t_ns, file0] : cams[0]) {
        std::vector<std::string> files{file0};
        for (int c = 1; c < num_cams_; ++c) {
            const auto it = cams[c].find(t_ns);
            if (it == cams[c].end()) break;
            files.push_back(it->second);
        }
        if (static_cast<int>(files.size()) != num_cams_) {
            ++dropped;
            continue;
        }
        frame_stamps_ns_.push_back(t_ns);
        frame_files_.push_back(std::move(files));
    }
    if (frame_stamps_ns_.empty()) throw std::runtime_error("EuRoC: no synchronized frames in " + mav0_);

    // IMU (optional): timestamp, w_x, w_y, w_z [rad/s], a_x, a_y, a_z [m/s^2].
    const fs::path imu_csv = root / "imu0" / "data.csv";
    if (std::ifstream in{imu_csv}) {
        std::string line;
        while (std::getline(in, line)) {
            if (line.empty() || line[0] == '#') continue;
            const auto f = splitCsv(line);
            if (f.size() < 7) throw std::runtime_error("EuRoC: malformed IMU line: " + line);
            ImuRecord r;
            r.t_ns = std::stoll(f[0]);
            r.sample.timestamp = static_cast<double>(r.t_ns) * kNsToSec;
            r.sample.gyro = {std::stod(f[1]), std::stod(f[2]), std::stod(f[3])};
            r.sample.accel = {std::stod(f[4]), std::stod(f[5]), std::stod(f[6])};
            imu_.push_back(r);
        }
    } else if (settings.imu) {
        throw std::runtime_error("EuRoC: imu enabled but " + imu_csv.string() + " is missing");
    }

    LOG(INFO) << "EuRoC " << mav0_ << ": " << frame_stamps_ns_.size() << " frames x " << num_cams_
              << " cams (" << dropped << " unsynchronized dropped), " << imu_.size() << " IMU samples";
}

bool EurocReader::readFrame(std::vector<cv::Mat>& imgs, int64_t& t_ns) {
    imgs.clear();
    if (next_frame_ >= frame_stamps_ns_.size()) return false;
    for (const auto& file : frame_files_[next_frame_]) {
        cv::Mat img = cv::imread(file, cv::IMREAD_GRAYSCALE);
        if (img.empty()) throw std::runtime_error("EuRoC: failed to read " + file);
        imgs.push_back(img);
    }
    t_ns = frame_stamps_ns_[next_frame_++];
    return true;
}

void EurocReader::collectImu(int64_t up_to_ns, std::deque<ImuSample>& out) {
    while (next_imu_ < imu_.size() && imu_[next_imu_].t_ns <= up_to_ns) out.push_back(imu_[next_imu_++].sample);
}

void EurocReader::loadNext(std::vector<cv::Mat>& imgs) {
    int64_t t_ns;
    readFrame(imgs, t_ns);
}

void EurocReader::getNext(std::vector<cv::Mat>& imgs, double& timeStamp) {
    int64_t t_ns = 0;
    if (readFrame(imgs, t_ns)) timeStamp = static_cast<double>(t_ns) * kNsToSec;
}

void EurocReader::getNext(std::vector<cv::Mat>& imgs, std::vector<std::string>& segmaskImgs, double& timeStamp) {
    segmaskImgs.clear();  // EuRoC has no segmentation masks
    getNext(imgs, timeStamp);
}

void EurocReader::getNext(std::vector<cv::Mat>& imgs, double& timeStamp, std::deque<ImuSample>& imu) {
    int64_t t_ns = 0;
    if (!readFrame(imgs, t_ns)) return;
    timeStamp = static_cast<double>(t_ns) * kNsToSec;
    collectImu(t_ns, imu);
}

void EurocReader::getNext(std::vector<cv::Mat>& imgs, double& timeStamp, std::deque<ImuSample>& imu,
                          std::deque<GnssFix>& gps) {
    gps.clear();  // EuRoC has no GNSS
    getNext(imgs, timeStamp, imu);
}

}  // namespace mcslam
