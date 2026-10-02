#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <random>

#include <opencv2/imgcodecs.hpp>

#include "MCDataUtils/EurocReader.h"
#include "MCDataUtils/KalibrCalibration.h"

namespace fs = std::filesystem;

namespace {

constexpr char kCalib[] = R"(%YAML:1.0
cam0:
  camera_model: pinhole
  distortion_model: radtan
  intrinsics: [458.654, 457.296, 367.215, 248.375]
  distortion_coeffs: [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05]
  resolution: [8, 6]
cam1:
  T_cn_cnm1:
    - [1.0, 0.0, 0.0, -0.11]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  camera_model: pinhole
  distortion_model: radtan
  intrinsics: [457.587, 456.134, 379.999, 255.238]
  distortion_coeffs: [-0.28368365, 0.07451284, -0.00010473, -3.55590700e-05]
  resolution: [8, 6]
imu:
  acc_noise: 2.0e-3
  gyr_noise: 1.6968e-4
  acc_walk: 3.0e-3
  gyr_walk: 1.9393e-5
  g_norm: 9.81
)";

// Builds <tmp>/mav0 with 3 cam0 frames, 2 matching cam1 frames, and 200 Hz IMU.
class EurocFixture : public ::testing::Test {
protected:
    void SetUp() override {
        root_ = fs::temp_directory_path() / ("euroc_test_" + std::to_string(std::random_device{}()));
        const fs::path mav0 = root_ / "mav0";
        for (const char* cam : {"cam0", "cam1"}) fs::create_directories(mav0 / cam / "data");
        fs::create_directories(mav0 / "imu0");

        const cv::Mat img(6, 8, CV_8UC1, cv::Scalar(128));
        const std::vector<int64_t> cam0_stamps{1000000000, 1050000000, 1100000000};
        const std::vector<int64_t> cam1_stamps{1000000000, 1100000000};  // middle frame missing
        writeCam(mav0 / "cam0", cam0_stamps, img);
        writeCam(mav0 / "cam1", cam1_stamps, img);

        std::ofstream imu(mav0 / "imu0" / "data.csv");
        imu << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
               "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\r\n";
        for (int64_t t = 950000000; t <= 1150000000; t += 5000000)  // 200 Hz
            imu << t << ",0.1,0.2,0.3,9.0,0.5,-0.5\r\n";

        calib_ = root_ / "calib.yaml";
        std::ofstream(calib_) << kCalib;
    }

    void TearDown() override { fs::remove_all(root_); }

    static void writeCam(const fs::path& dir, const std::vector<int64_t>& stamps, const cv::Mat& img) {
        std::ofstream csv(dir / "data.csv");
        csv << "#timestamp [ns],filename\n";
        for (int64_t t : stamps) {
            const std::string name = std::to_string(t) + ".png";
            cv::imwrite((dir / "data" / name).string(), img);
            csv << t << "," << name << "\n";
        }
    }

    MCDataUtilSettings settings(bool imu) const {
        MCDataUtilSettings s;
        s.data_path = root_.string();
        s.calib_file_path = calib_.string();
        s.imu = imu;
        return s;
    }

    fs::path root_, calib_;
};

}  // namespace

TEST_F(EurocFixture, CalibrationIgnoresNonCameraNodesAndChainsExtrinsics) {
    const auto c = mcslam::loadKalibrCameraChain(calib_.string());
    ASSERT_EQ(c.num_cams, 2);  // the imu: node must not become a camera
    EXPECT_EQ(c.image_size, cv::Size(8, 6));
    EXPECT_DOUBLE_EQ(c.K[0].at<double>(0, 0), 458.654);
    EXPECT_DOUBLE_EQ(c.dist[1].at<double>(0, 3), -3.55590700e-05);
    EXPECT_DOUBLE_EQ(c.t[1].at<double>(0, 0), -0.11);
    EXPECT_DOUBLE_EQ(cv::norm(c.R[0] - cv::Mat::eye(3, 3, CV_64F)), 0.0);
}

TEST_F(EurocFixture, CalibrationRejectsMissingFile) {
    EXPECT_THROW(mcslam::loadKalibrCameraChain((root_ / "nope.yaml").string()), std::runtime_error);
}

TEST_F(EurocFixture, EmitsOnlySynchronizedFramesWithImuSlices) {
    mcslam::EurocReader reader;
    reader.initialize(settings(true));
    EXPECT_EQ(reader.num_cams_, 2);
    EXPECT_EQ(reader.numFrames(), 2u);  // 1.05 s exists only in cam0
    EXPECT_EQ(reader.numImuSamples(), 41u);

    std::vector<cv::Mat> imgs;
    std::deque<mcslam::ImuSample> imu;
    double t = 0.0;

    reader.getNext(imgs, t, imu);
    ASSERT_EQ(imgs.size(), 2u);
    EXPECT_EQ(imgs[0].type(), CV_8UC1);
    EXPECT_DOUBLE_EQ(t, 1.0);
    EXPECT_EQ(imu.size(), 11u);  // 0.95 .. 1.00 inclusive
    EXPECT_DOUBLE_EQ(imu.back().timestamp, 1.0);
    EXPECT_DOUBLE_EQ(imu.front().gyro.y(), 0.2);
    EXPECT_DOUBLE_EQ(imu.front().accel.x(), 9.0);

    imu.clear();
    reader.getNext(imgs, t, imu);
    EXPECT_DOUBLE_EQ(t, 1.1);
    EXPECT_EQ(imu.size(), 20u);  // (1.00, 1.10]
    EXPECT_GT(imu.front().timestamp, 1.0);

    reader.getNext(imgs, t, imu);
    EXPECT_TRUE(imgs.empty());  // end of sequence
}

TEST_F(EurocFixture, ThrowsWhenImuRequestedButMissing) {
    fs::remove(root_ / "mav0" / "imu0" / "data.csv");
    mcslam::EurocReader reader;
    EXPECT_THROW(reader.initialize(settings(true)), std::runtime_error);
}
