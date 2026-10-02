#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "MCDataUtils/DatasetReaderBase.h"
#include "MCDataUtils/SensorTypes.h"

namespace mcslam {

/**
 * Reader for the EuRoC MAV dataset in ASL format:
 *   <data_path>/mav0/cam{0,1}/data.csv + data/<ns>.png, <data_path>/mav0/imu0/data.csv
 *
 * Camera intrinsics/extrinsics come from the Kalibr-style file at
 * settings.calib_file_path (see params/euroc/). Frames are emitted only for
 * timestamps present in every camera stream. Each getNext() that takes an IMU
 * queue returns the IMU samples in (previous frame time, current frame time];
 * the first call returns every sample up to the first frame.
 */
class EurocReader : public DatasetReaderBase {
public:
    void initialize(MCDataUtilSettings settings) override;

    void loadNext(std::vector<cv::Mat>& imgs) override;
    void getNext(std::vector<cv::Mat>& imgs, double& timeStamp) override;
    void getNext(std::vector<cv::Mat>& imgs, std::vector<std::string>& segmaskImgs, double& timeStamp) override;
    void getNext(std::vector<cv::Mat>& imgs, double& timeStamp, std::deque<ImuSample>& imu) override;
    void getNext(std::vector<cv::Mat>& imgs, double& timeStamp, std::deque<ImuSample>& imu,
                 std::deque<GnssFix>& gps) override;

    size_t numFrames() const { return frame_stamps_ns_.size(); }
    size_t numImuSamples() const { return imu_.size(); }

private:
    struct ImuRecord {
        int64_t t_ns;
        ImuSample sample;
    };

    bool readFrame(std::vector<cv::Mat>& imgs, int64_t& t_ns);
    void collectImu(int64_t up_to_ns, std::deque<ImuSample>& out);

    std::string mav0_;
    std::vector<int64_t> frame_stamps_ns_;   // synchronized across cameras
    std::vector<std::vector<std::string>> frame_files_;  // [frame][cam]
    std::vector<ImuRecord> imu_;
    size_t next_frame_ = 0;
    size_t next_imu_ = 0;
};

}  // namespace mcslam
