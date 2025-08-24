//
// VideoStreamReader.h - Windows-compatible video stream reader for MC-SLAM
// Replaces ROS dependencies with OpenCV video capture
//

#pragma once

#include "DatasetReaderBase.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <chrono>

// Simple IMU data structure to replace sensor_msgs::Imu
struct ImuData {
    double timestamp;
    double linear_acceleration[3];  // x, y, z
    double angular_velocity[3];     // x, y, z
    double orientation[4];          // quaternion w, x, y, z
};

// Simple GPS data structure to replace sensor_msgs::NavSatFix
struct GpsData {
    double timestamp;
    double latitude;
    double longitude;
    double altitude;
    double covariance[9];
};

class VideoStreamReader : public DatasetReaderBase {
public:
    VideoStreamReader();
    ~VideoStreamReader();

    // Inherited from DatasetReaderBase
    void initialize(MCDataUtilSettings settings) override;
    void getNext(std::vector<cv::Mat>& imgs, double& timeStamp) override;
    void getNext(std::vector<cv::Mat>& imgs, std::vector<std::string>& segmaskNames, double& timeStamp) override;
    void getNext(std::vector<cv::Mat>& imgs, double& timeStamp, std::deque<ImuData>& imu_msgs);
    void getNext(std::vector<cv::Mat>& imgs, double& timeStamp, std::deque<ImuData>& imu_msgs, std::deque<GpsData>& gps_msgs);

    // Video stream specific methods
    bool openVideoStreams(const std::vector<std::string>& video_paths);
    bool openCameras(const std::vector<int>& camera_indices);
    void startCapture();
    void stopCapture();

private:
    // Video capture objects
    std::vector<cv::VideoCapture> video_captures_;
    std::vector<std::string> video_paths_;
    std::vector<int> camera_indices_;
    
    // Threading for continuous capture
    std::vector<std::thread> capture_threads_;
    std::vector<cv::Mat> current_frames_;
    std::vector<bool> frame_ready_;
    std::vector<std::mutex> frame_mutexes_;
    std::condition_variable frame_condition_;
    std::mutex global_mutex_;
    
    // Timing
    std::chrono::high_resolution_clock::time_point start_time_;
    double current_timestamp_;
    
    // IMU and GPS simulation/reading
    std::deque<ImuData> imu_data_;
    std::deque<GpsData> gps_data_;
    std::mutex imu_mutex_;
    std::mutex gps_mutex_;
    
    // Control flags
    bool is_capturing_;
    bool use_cameras_;  // true for live cameras, false for video files
    
    // Helper methods
    void captureThreadFunction(int camera_index);
    void loadCalibrationData(const std::string& calib_path);
    void simulateImuData(double timestamp);
    void simulateGpsData(double timestamp);
    double getCurrentTimestamp();
    void preprocessFrame(cv::Mat& frame);
    
    // Configuration
    bool simulate_imu_;
    bool simulate_gps_;
    int target_fps_;
    cv::Size target_size_;
};

// Helper functions for time conversion
double getTimestampSeconds();
std::chrono::high_resolution_clock::time_point getTimePoint();