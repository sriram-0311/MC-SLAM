//
// VideoStreamReader.cpp - Implementation of Windows-compatible video stream reader
//

#include "MCDataUtils/VideoStreamReader.h"
#include "MCDataUtils/MCDataUtilParams.h"
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iostream>

#ifdef MCSLAM_GLOG_SUPPORT
    #include <glog/logging.h>
    #define LOG_INFO LOG(INFO)
    #define LOG_ERROR LOG(ERROR)
    #define VLOG_INFO(x) VLOG(x)
#else
    #define LOG_INFO std::cout
    #define LOG_ERROR std::cerr
    #define VLOG_INFO(x) if(x <= 1) std::cout
#endif

VideoStreamReader::VideoStreamReader() 
    : is_capturing_(false)
    , use_cameras_(false)
    , simulate_imu_(false)
    , simulate_gps_(false)
    , target_fps_(30)
    , target_size_(640, 480)
    , current_timestamp_(0.0)
{
    start_time_ = std::chrono::high_resolution_clock::now();
}

VideoStreamReader::~VideoStreamReader() {
    stopCapture();
}

void VideoStreamReader::initialize(MCDataUtilSettings settings) {
    settings_ = settings;
    
    LOG_INFO << "Initializing VideoStreamReader..." << std::endl;
    
    // Load calibration data
    loadCalibrationData(settings.calib_file_path);
    
    // Determine input type from settings
    if (!settings.images_path.empty()) {
        // Check if images_path contains video files or camera indices
        std::ifstream config_file(settings.images_path);
        if (config_file.good()) {
            // Read video file paths or camera indices from config file
            std::string line;
            std::vector<std::string> video_files;
            std::vector<int> cam_indices;
            
            while (std::getline(config_file, line)) {
                if (line.empty() || line[0] == '#') continue;
                
                // Try to parse as integer (camera index)
                try {
                    int cam_id = std::stoi(line);
                    cam_indices.push_back(cam_id);
                    use_cameras_ = true;
                } catch (...) {
                    // Not an integer, treat as video file path
                    video_files.push_back(line);
                    use_cameras_ = false;
                }
            }
            
            if (use_cameras_ && !cam_indices.empty()) {
                openCameras(cam_indices);
            } else if (!video_files.empty()) {
                openVideoStreams(video_files);
            }
        } else {
            // Single video file or camera index
            try {
                int cam_id = std::stoi(settings.images_path);
                openCameras({cam_id});
                use_cameras_ = true;
            } catch (...) {
                openVideoStreams({settings.images_path});
                use_cameras_ = false;
            }
        }
    }
    
    // Setup IMU and GPS simulation if needed
    simulate_imu_ = settings.imu;
    simulate_gps_ = settings.gps;
    
    // Initialize frame storage
    current_frames_.resize(num_cams_);
    frame_ready_.resize(num_cams_, false);
    frame_mutexes_.resize(num_cams_);
    
    startCapture();
    
    LOG_INFO << "VideoStreamReader initialized with " << num_cams_ << " cameras" << std::endl;
}

bool VideoStreamReader::openVideoStreams(const std::vector<std::string>& video_paths) {
    video_paths_ = video_paths;
    video_captures_.resize(video_paths.size());
    
    for (size_t i = 0; i < video_paths.size(); ++i) {
        if (!video_captures_[i].open(video_paths[i])) {
            LOG_ERROR << "Failed to open video file: " << video_paths[i] << std::endl;
            return false;
        }
        
        // Get video properties
        if (i == 0) {
            target_fps_ = static_cast<int>(video_captures_[i].get(cv::CAP_PROP_FPS));
            int width = static_cast<int>(video_captures_[i].get(cv::CAP_PROP_FRAME_WIDTH));
            int height = static_cast<int>(video_captures_[i].get(cv::CAP_PROP_FRAME_HEIGHT));
            target_size_ = cv::Size(width, height);
            img_size_ = target_size_;
        }
        
        LOG_INFO << "Opened video stream " << i << ": " << video_paths[i] << std::endl;
    }
    
    num_cams_ = static_cast<int>(video_paths.size());
    return true;
}

bool VideoStreamReader::openCameras(const std::vector<int>& camera_indices) {
    camera_indices_ = camera_indices;
    video_captures_.resize(camera_indices.size());
    
    for (size_t i = 0; i < camera_indices.size(); ++i) {
        if (!video_captures_[i].open(camera_indices[i])) {
            LOG_ERROR << "Failed to open camera " << camera_indices[i] << std::endl;
            return false;
        }
        
        // Set camera properties
        video_captures_[i].set(cv::CAP_PROP_FRAME_WIDTH, target_size_.width);
        video_captures_[i].set(cv::CAP_PROP_FRAME_HEIGHT, target_size_.height);
        video_captures_[i].set(cv::CAP_PROP_FPS, target_fps_);
        
        // Get actual properties
        if (i == 0) {
            int width = static_cast<int>(video_captures_[i].get(cv::CAP_PROP_FRAME_WIDTH));
            int height = static_cast<int>(video_captures_[i].get(cv::CAP_PROP_FRAME_HEIGHT));
            img_size_ = cv::Size(width, height);
        }
        
        LOG_INFO << "Opened camera " << camera_indices[i] << std::endl;
    }
    
    num_cams_ = static_cast<int>(camera_indices.size());
    return true;
}

void VideoStreamReader::startCapture() {
    if (is_capturing_) return;
    
    is_capturing_ = true;
    capture_threads_.clear();
    
    // Start capture threads for each camera
    for (int i = 0; i < num_cams_; ++i) {
        capture_threads_.emplace_back(&VideoStreamReader::captureThreadFunction, this, i);
    }
}

void VideoStreamReader::stopCapture() {
    if (!is_capturing_) return;
    
    is_capturing_ = false;
    
    // Wait for all threads to finish
    for (auto& thread : capture_threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    
    // Release video captures
    for (auto& cap : video_captures_) {
        if (cap.isOpened()) {
            cap.release();
        }
    }
}

void VideoStreamReader::captureThreadFunction(int camera_index) {
    while (is_capturing_) {
        cv::Mat frame;
        
        if (video_captures_[camera_index].read(frame)) {
            // Preprocess frame
            preprocessFrame(frame);
            
            // Store frame
            {
                std::lock_guard<std::mutex> lock(frame_mutexes_[camera_index]);
                current_frames_[camera_index] = frame.clone();
                frame_ready_[camera_index] = true;
            }
            
            // Notify that frame is ready
            frame_condition_.notify_all();
        } else {
            // End of video or camera error
            if (!use_cameras_) {
                // For video files, stop when reaching end
                is_capturing_ = false;
                break;
            } else {
                // For cameras, try to reconnect
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        
        // Control frame rate for cameras
        if (use_cameras_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / target_fps_));
        }
    }
}

void VideoStreamReader::getNext(std::vector<cv::Mat>& imgs, double& timeStamp) {
    imgs.clear();
    
    // Wait for all frames to be ready
    std::unique_lock<std::mutex> lock(global_mutex_);
    frame_condition_.wait(lock, [this] {
        for (int i = 0; i < num_cams_; ++i) {
            if (!frame_ready_[i]) return false;
        }
        return true;
    });
    
    // Copy frames
    for (int i = 0; i < num_cams_; ++i) {
        std::lock_guard<std::mutex> frame_lock(frame_mutexes_[i]);
        if (frame_ready_[i]) {
            imgs.push_back(current_frames_[i].clone());
            frame_ready_[i] = false;
        }
    }
    
    // Update timestamp
    timeStamp = getCurrentTimestamp();
    current_timestamp_ = timeStamp;
    
    img_counter_++;
}

void VideoStreamReader::getNext(std::vector<cv::Mat>& imgs, std::vector<std::string>& segmaskNames, double& timeStamp) {
    getNext(imgs, timeStamp);
    
    // Generate segmentation mask names if needed
    segmaskNames.clear();
    if (settings_.read_segmask) {
        for (int i = 0; i < num_cams_; ++i) {
            std::string mask_name = settings_.segmasks_path + "/cam" + std::to_string(i) + 
                                  "/" + std::to_string(static_cast<long long>(timeStamp * 1000)) + 
                                  "." + settings_.mask_type;
            segmaskNames.push_back(mask_name);
        }
    }
}

void VideoStreamReader::getNext(std::vector<cv::Mat>& imgs, double& timeStamp, std::deque<ImuData>& imu_msgs) {
    getNext(imgs, timeStamp);
    
    if (simulate_imu_) {
        simulateImuData(timeStamp);
    }
    
    // Return IMU data up to current timestamp
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_msgs.clear();
    while (!imu_data_.empty() && imu_data_.front().timestamp <= timeStamp) {
        imu_msgs.push_back(imu_data_.front());
        imu_data_.pop_front();
    }
}

void VideoStreamReader::getNext(std::vector<cv::Mat>& imgs, double& timeStamp, 
                               std::deque<ImuData>& imu_msgs, std::deque<GpsData>& gps_msgs) {
    getNext(imgs, timeStamp, imu_msgs);
    
    if (simulate_gps_) {
        simulateGpsData(timeStamp);
    }
    
    // Return GPS data up to current timestamp
    std::lock_guard<std::mutex> lock(gps_mutex_);
    gps_msgs.clear();
    while (!gps_data_.empty() && gps_data_.front().timestamp <= timeStamp) {
        gps_msgs.push_back(gps_data_.front());
        gps_data_.pop_front();
    }
}

void VideoStreamReader::loadCalibrationData(const std::string& calib_path) {
    cv::FileStorage fs(calib_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        LOG_ERROR << "Cannot open calibration file: " << calib_path << std::endl;
        return;
    }
    
    LOG_INFO << "Reading calibration data from: " << calib_path << std::endl;
    
    cv::FileNode fn = fs.root();
    cv::FileNodeIterator fi = fn.begin(), fi_end = fn.end();
    
    int cam_count = 0;
    for (; fi != fi_end; ++fi, cam_count++) {
        cv::FileNode f = *fi;
        if (f.name().find("cam") == std::string::npos) break;
        
        // Read distortion coefficients
        std::vector<double> dc;
        cv::Mat_<double> dist_coeff = cv::Mat_<double>::zeros(1, 5);
        f["distortion_coeffs"] >> dc;
        
        if (settings_.radtan) {
            for (size_t j = 0; j < dc.size() && j < 5; j++) {
                dist_coeff(0, j) = dc[j];
            }
        } else {
            // Equidistant model
            for (size_t j = 0; j < 3 && j < dc.size(); j++) {
                if (j < 2) {
                    dist_coeff(0, j) = dc[j];
                } else {
                    dist_coeff(0, j + 2) = dc[j];
                }
            }
        }
        
        // Read resolution
        std::vector<int> resolution;
        f["resolution"] >> resolution;
        if (cam_count == 0) {
            calib_img_size_ = cv::Size(resolution[0], resolution[1]);
        }
        
        // Read intrinsics
        std::vector<double> intrinsics;
        f["intrinsics"] >> intrinsics;
        cv::Mat_<double> K_mat = cv::Mat_<double>::zeros(3, 3);
        K_mat(0, 0) = intrinsics[0]; K_mat(1, 1) = intrinsics[1];
        K_mat(0, 2) = intrinsics[2]; K_mat(1, 2) = intrinsics[3];
        K_mat(2, 2) = 1.0;
        
        // Read extrinsics
        cv::Mat_<double> R = cv::Mat_<double>::eye(3, 3);
        cv::Mat_<double> t = cv::Mat_<double>::zeros(3, 1);
        
        cv::FileNode tn = f["T_cn_cnm1"];
        if (!tn.empty()) {
            cv::FileNodeIterator fi2 = tn.begin(), fi2_end = tn.end();
            int r = 0;
            for (; fi2 != fi2_end && r < 3; ++fi2, r++) {
                cv::FileNode f2 = *fi2;
                R(r, 0) = f2[0]; R(r, 1) = f2[1]; R(r, 2) = f2[2];
                t(r, 0) = f2[3];
            }
        }
        
        // Store calibration data
        R_mats_kalibr.push_back(R.clone());
        t_vecs_kalibr.push_back(t.clone());
        
        // Convert to world coordinates
        if (cam_count > 0) {
            cv::Mat R3 = R * R_mats_[cam_count - 1];
            cv::Mat t3 = R * t_vecs_[cam_count - 1] + t;
            R = R3; t = t3;
        }
        
        cv::Mat Rt = build_Rt(R, t);
        cv::Mat P = K_mat * Rt;
        
        R_mats_.push_back(R);
        t_vecs_.push_back(t);
        dist_coeffs_.push_back(dist_coeff);
        K_mats_.push_back(K_mat);
        P_mats_.push_back(P);
    }
    
    img_size_ = calib_img_size_;
    num_cams_ = cam_count;
    
    LOG_INFO << "Loaded calibration for " << num_cams_ << " cameras" << std::endl;
}

void VideoStreamReader::simulateImuData(double timestamp) {
    // Simple IMU simulation - replace with actual IMU reading if available
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    ImuData imu;
    imu.timestamp = timestamp;
    
    // Simulate some basic IMU data (could be replaced with actual sensor reading)
    imu.linear_acceleration[0] = 0.0;
    imu.linear_acceleration[1] = 0.0;
    imu.linear_acceleration[2] = -9.81; // gravity
    
    imu.angular_velocity[0] = 0.0;
    imu.angular_velocity[1] = 0.0;
    imu.angular_velocity[2] = 0.0;
    
    imu.orientation[0] = 1.0; // w
    imu.orientation[1] = 0.0; // x
    imu.orientation[2] = 0.0; // y
    imu.orientation[3] = 0.0; // z
    
    imu_data_.push_back(imu);
}

void VideoStreamReader::simulateGpsData(double timestamp) {
    // Simple GPS simulation - replace with actual GPS reading if available
    std::lock_guard<std::mutex> lock(gps_mutex_);
    
    GpsData gps;
    gps.timestamp = timestamp;
    gps.latitude = 42.3601; // Example: Boston
    gps.longitude = -71.0589;
    gps.altitude = 0.0;
    
    // Simple covariance
    for (int i = 0; i < 9; ++i) {
        gps.covariance[i] = (i % 4 == 0) ? 1.0 : 0.0;
    }
    
    gps_data_.push_back(gps);
}

double VideoStreamReader::getCurrentTimestamp() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time_);
    return duration.count() / 1000000.0; // Convert to seconds
}

void VideoStreamReader::preprocessFrame(cv::Mat& frame) {
    if (frame.empty()) return;
    
    // Resize if needed
    if (settings_.resize_images && settings_.rf != 1.0) {
        cv::Size new_size(static_cast<int>(frame.cols * settings_.rf), 
                         static_cast<int>(frame.rows * settings_.rf));
        cv::resize(frame, frame, new_size);
    }
    
    // Convert to grayscale if color processing is not enabled
    if (!settings_.color_imgs && frame.channels() == 3) {
        cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    }
    
    // Convert to float and normalize
    frame.convertTo(frame, CV_32F);
    frame /= 255.0f;
}

// Helper functions
double getTimestampSeconds() {
    auto now = std::chrono::high_resolution_clock::now();
    auto epoch = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::microseconds>(epoch);
    return seconds.count() / 1000000.0;
}

std::chrono::high_resolution_clock::time_point getTimePoint() {
    return std::chrono::high_resolution_clock::now();
}