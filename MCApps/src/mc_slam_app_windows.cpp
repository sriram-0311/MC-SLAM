//
// mc_slam_app_windows.cpp - Windows-compatible MC-SLAM application
// Removes ROS dependencies and uses VideoStreamReader for input
//

#include "MCSlam/FrontEnd.h"
#include "MCSlam/Backend.h"

#ifdef MCSLAM_PANGOLIN_SUPPORT
#include "MCSlam/OpenGlViewer.h"
#endif

#include "ParseSettings.h"
#include "MCDataUtils/CamArrayConfig.h"
#include "MCDataUtils/MCDataUtilParams.h"
#include "MCDataUtils/DatasetReader.h"
#include "MCDataUtils/VideoStreamReader.h"

#include <chrono>
#include <thread>
#include <iostream>

#ifdef MCSLAM_OPENGV_SUPPORT
#include <Eigen/Eigen>
#include <opengv/types.hpp>
#include <memory>
#include <opengv/relative_pose/NoncentralRelativeAdapter.hpp>
#include <opengv/sac_problems/relative_pose/NoncentralRelativePoseSacProblem.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/relative_pose/methods.hpp>
#endif

#include "MCSlam/time_measurement.hpp"
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>

#ifdef MCSLAM_GFLAGS_SUPPORT
    #include <gflags/gflags.h>
    DEFINE_bool(fhelp, false, "show config file options");
    DEFINE_string(config_file, "", "config file path");
    DEFINE_string(log_file, "pose_stats.txt", "log_file file path");
    DEFINE_string(traj_file, "Tum_trajectory.txt", "trajectory file file path");
#else
    // Simple command line parsing fallback
    struct {
        bool fhelp = false;
        std::string config_file = "";
        std::string log_file = "pose_stats.txt";
        std::string traj_file = "Tum_trajectory.txt";
    } FLAGS;
    
    void parseCommandLine(int argc, char** argv) {
        for (int i = 1; i < argc; i++) {
            std::string arg = argv[i];
            if (arg == "--help" || arg == "-h") {
                FLAGS.fhelp = true;
            } else if (arg == "--config_file" && i + 1 < argc) {
                FLAGS.config_file = argv[++i];
            } else if (arg == "--log_file" && i + 1 < argc) {
                FLAGS.log_file = argv[++i];
            } else if (arg == "--traj_file" && i + 1 < argc) {
                FLAGS.traj_file = argv[++i];
            }
        }
    }
#endif

#ifdef MCSLAM_GLOG_SUPPORT
    #include <glog/logging.h>
    #define LOG_INFO LOG(INFO)
    #define VLOG_INFO(x) VLOG(x)
#else
    #define LOG_INFO std::cout
    #define VLOG_INFO(x) if(x <= 1) std::cout
#endif

void handleKeyboardInput(FrontEnd& frontend, Backend& backend, DatasetReaderBase &dataReader
#ifdef MCSLAM_PANGOLIN_SUPPORT
    , OpenGlViewer &glViewer
#endif
);

void process_frames(FrontEnd& frontend, Backend& backend);
bool updateData(FrontEnd& frontend, DatasetReaderBase &dataReader);

using namespace cv;
using namespace std;
#ifdef MCSLAM_OPENGV_SUPPORT
using namespace opengv;
#endif
using namespace std::chrono;

float tracking_time = 0.0;
float optim_time = 0.0;
float feat_xtract_time = 0.0;
int frame_counter = 0;
int optim_cnt = 0;

int main(int argc, char** argv) {

#ifdef MCSLAM_GFLAGS_SUPPORT
    google::ParseCommandLineFlags(&argc, &argv, true);
#else
    parseCommandLine(argc, argv);
#endif

#ifdef MCSLAM_GLOG_SUPPORT
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;
#endif

    if (FLAGS.fhelp) {
        std::cout << "MC-SLAM Windows Application\n";
        std::cout << "Usage: " << argv[0] << " --config_file <path> [options]\n";
        std::cout << "Options:\n";
        std::cout << "  --config_file <path>  Configuration file path (required)\n";
        std::cout << "  --log_file <path>     Log file path (default: pose_stats.txt)\n";
        std::cout << "  --traj_file <path>    Trajectory file path (default: Tum_trajectory.txt)\n";
        std::cout << "  --help, -h            Show this help message\n";
        return 0;
    }

    if (FLAGS.config_file.empty()) {
        std::cerr << "Error: config_file is required. Use --help for usage information.\n";
        return -1;
    }

    // Parse the settings in config file
    MCDataUtilSettings settings;
    parse_settings(FLAGS.config_file, settings, FLAGS.fhelp);

    // Initialize data reader - use VideoStreamReader for Windows
    DatasetReaderBase* datareader = nullptr;
    
    if (settings.is_ros) {
        std::cerr << "Warning: ROS support is not available in Windows build. Using VideoStreamReader instead.\n";
        settings.is_ros = false;
    }
    
    // Always use VideoStreamReader or DatasetReader for Windows
    if (!settings.images_path.empty()) {
        // Check if this looks like a video file or camera index
        std::string path = settings.images_path;
        if (path.find(".mp4") != std::string::npos || 
            path.find(".avi") != std::string::npos || 
            path.find(".mov") != std::string::npos ||
            std::isdigit(path[0])) {
            // Video file or camera index
            datareader = new VideoStreamReader();
        } else {
            // Image sequence
            datareader = new DatasetReader();
        }
    } else {
        datareader = new DatasetReader();
    }
    
    datareader->initialize(settings);

    // Create camera configuration object
    auto* cam_cfg = new CamArrayConfig(
        datareader->getKMats(), datareader->getDistMats(), datareader->getRMats(),
        datareader->getTMats(), datareader->getKalibrRMats(), datareader->getKalibrTMats(),
        datareader->img_size_, datareader->num_cams_, false);
    
    VLOG_INFO(0) << "datareader initialized" << std::endl;

    // Create the SLAM front end object
    FrontEnd *slam_frontend = new FrontEnd(
        settings.frontend_params_file, *cam_cfg, settings.debug_mode, settings.calib_file_path,
        settings.imu, settings.gps, settings.relocalization, settings.customVocabFile,
        settings.dataBase, settings.graphLogs, settings.mapLogs, settings.navability, settings.ImuMapFrame);
    
    VLOG_INFO(0) << "slam_frontend initialized" << std::endl;

    // Create the SLAM backend object
    Backend *slam_backend = new Backend(settings.backend_params_file, *cam_cfg, slam_frontend);
    VLOG_INFO(0) << "slam_backend initialized" << std::endl;

#ifdef MCSLAM_PANGOLIN_SUPPORT
    // Create OpenGL viewer object if Pangolin is available
    OpenGlViewer* glViewer = new OpenGlViewer(settings.frontend_params_file, slam_frontend);
    
    std::thread* viewerThread = nullptr;
    if (!settings.relocalization) {
        viewerThread = new std::thread(&OpenGlViewer::goLive, glViewer);
    } else {
        viewerThread = new std::thread([&]() {
            glViewer->goLiveFastTracking(slam_frontend);
        });
    }
#endif

    // Main processing loop
    LOG_INFO << "Starting MC-SLAM processing..." << std::endl;
    
#ifdef MCSLAM_PANGOLIN_SUPPORT
    handleKeyboardInput(*slam_frontend, *slam_backend, *datareader, *glViewer);
#else
    handleKeyboardInput(*slam_frontend, *slam_backend, *datareader);
#endif

    // Cleanup and save results
    slam_frontend->logFile_.close();
    slam_frontend->logFile2_.close();

    if (slam_frontend->useGPS) {
        slam_frontend->writeTrajectoryToFile(FLAGS.traj_file, true);
    } else {
        slam_frontend->writeTrajectoryToFile(FLAGS.traj_file, false);
    }

    if (!slam_frontend->relocal) {
        slam_frontend->saveORBDatabase();
    }

#ifdef MCSLAM_PANGOLIN_SUPPORT
    if (viewerThread && viewerThread->joinable()) {
        glViewer->requestFinish();
        viewerThread->join();
        delete viewerThread;
        delete glViewer;
    }
#endif

    delete slam_backend;
    delete slam_frontend;
    delete cam_cfg;
    delete datareader;

    LOG_INFO << "MC-SLAM processing completed." << std::endl;
    return 0;
}

bool updateData(FrontEnd& frontend, DatasetReaderBase &dataReader) {
    std::vector<Mat> imgs, segMasks;
    std::vector<std::string> segmaskNames;
    double timeStamp;

    // For Windows build, we use simplified data structures instead of ROS messages
    std::deque<ImuData> imu_msgs;
    std::deque<GpsData> gps_msgs;

    try {
        if (dataReader.settings.gps) {
            // Try to cast to VideoStreamReader for GPS support
            VideoStreamReader* videoReader = dynamic_cast<VideoStreamReader*>(&dataReader);
            if (videoReader) {
                videoReader->getNext(imgs, timeStamp, imu_msgs, gps_msgs);
                // Convert GPS data to frontend format if needed
                // frontend.gps_msgs_combined.insert(frontend.gps_msgs_combined.end(), gps_msgs.begin(), gps_msgs.end());
            } else {
                dataReader.getNext(imgs, timeStamp);
            }
        } else if (dataReader.settings.imu) {
            VideoStreamReader* videoReader = dynamic_cast<VideoStreamReader*>(&dataReader);
            if (videoReader) {
                videoReader->getNext(imgs, timeStamp, imu_msgs);
                // Convert IMU data to frontend format if needed
                // frontend.imu_msgs_combined.insert(frontend.imu_msgs_combined.end(), imu_msgs.begin(), imu_msgs.end());
            } else {
                dataReader.getNext(imgs, timeStamp);
            }
        } else {
            if (dataReader.settings.read_segmask) {
                dataReader.getNext(imgs, segmaskNames, timeStamp);
            } else {
                dataReader.getNext(imgs, timeStamp);
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error reading data: " << e.what() << std::endl;
        return false;
    }

    if (imgs.empty()) {
        VLOG_INFO(0) << "Reached the end of images" << std::endl;
        return false;
    }

    // Create segmentation masks
    for (int i = 0; i < dataReader.num_cams_; i++) {
        segMasks.push_back(Mat::zeros(dataReader.img_size_, CV_32FC1));
    }

    // Copy images and masks
    std::vector<cv::Mat> copiedImgs;
    for (const auto& img : imgs) {
        copiedImgs.push_back(img.clone());
    }

    std::vector<cv::Mat> copiedSegMasks;
    for (const auto& mask : segMasks) {
        copiedSegMasks.push_back(mask.clone());
    }

    frontend.createFrame(copiedImgs, copiedSegMasks, timeStamp);
    return true;
}

void process_frames(FrontEnd& frontend, Backend& backend) {
    frame_counter++;
    
    auto startF = high_resolution_clock::now();
    
    // Skip IMU initialization check for Windows build
    // if (frontend.useIMU) {
    //     if (!frontend.imu_initialized || frontend.imu_msgs_combined.empty()) {
    //         return;
    //     }
    // }

    // Process the multi-camera frame
    auto startT = high_resolution_clock::now();
    frontend.processFrame();
    auto stopT = high_resolution_clock::now();
    
    auto duration = duration_cast<milliseconds>(stopT - startT);
    feat_xtract_time += duration.count();
    VLOG_INFO(1) << "Feature extraction time: " << duration.count() << " ms | Average: " 
                 << (feat_xtract_time / frame_counter) << " ms" << std::endl;

    // Track the current frame
    startT = high_resolution_clock::now();
    bool new_kf = frontend.trackFrame();
    stopT = high_resolution_clock::now();
    
    auto stopF = high_resolution_clock::now();
    duration = duration_cast<milliseconds>(stopT - startT);
    tracking_time += duration.count();
    
    VLOG_INFO(1) << "Tracking time: " << duration.count() << " ms | Average: " 
                 << (tracking_time / frame_counter) << " ms" << std::endl;
    VLOG_INFO(1) << "Total frontend time: " << duration_cast<milliseconds>(stopF - startF).count() 
                 << " ms | Average: " << (duration_cast<milliseconds>(stopF - startF).count() / frame_counter) 
                 << " ms" << std::endl;

    // Backend processing
    startT = high_resolution_clock::now();
    bool optimize = false;
    
    if (frontend.initialized_ == NOT_INITIALIZED) {
        frontend.reset();
        return;
    }

    if (frontend.initialized_ == INITIALIZED && new_kf) {
        if (backend.backendType == MULTI) {
            auto startT1 = high_resolution_clock::now();
            bool loop_closed = false;
            
#ifdef MCSLAM_DBOW2_SUPPORT
            if (frontend.result.detection()) {
                loop_closed = true;
            }
#endif

            optimize = backend.SmartFactor_backend(loop_closed);
            auto stopT1 = high_resolution_clock::now();
            auto duration1 = duration_cast<milliseconds>(stopT1 - startT1);
            VLOG_INFO(0) << "Backend graph construction time: " << duration1.count() << " ms" << std::endl;

            if (optimize) {
                optim_cnt++;
                auto startT2 = high_resolution_clock::now();
                bool update = backend.optimizePosesLandmarks();
                auto stopT2 = high_resolution_clock::now();
                auto duration2 = duration_cast<milliseconds>(stopT2 - startT2);
                VLOG_INFO(0) << "Backend optimization time: " << duration2.count() << " ms" << std::endl;
                
                if (update) {
                    auto startT3 = high_resolution_clock::now();
                    backend.UpdateVariables_SmartFactors();
                    auto stopT3 = high_resolution_clock::now();
                    auto duration3 = duration_cast<milliseconds>(stopT3 - startT3);
                    VLOG_INFO(0) << "Variable update time: " << duration3.count() << " ms" << std::endl;
                }
            }

#ifdef MCSLAM_DBOW2_SUPPORT
            // Handle loop closure
            if (frontend.result.detection()) {
                backend.lvar.loop_candidate_ = true;
                backend.lvar.rel_pose_ = frontend.result.relative_pose;
                backend.lvar.best_match_idx_ = static_cast<int>(frontend.lfFrames[frontend.result.match]->frameId);
                backend.lvar.curr_frame_idx_ = static_cast<int>(frontend.currentFrame->frameId);
                frontend.appendLogs(true, backend.prevState_, backend.prevBias_);
            } else {
                frontend.appendLogs(false, backend.prevState_, backend.prevBias_);
            }
#endif
        }
    }

    stopT = high_resolution_clock::now();
    duration = duration_cast<milliseconds>(stopT - startT);
    optim_time += duration.count();
    
    if (optimize) {
        VLOG_INFO(1) << "Total backend time: " << duration.count() << " ms | Average: " 
                     << (optim_time / optim_cnt) << " ms" << std::endl;
    }

    frontend.reset();
}

void handleKeyboardInput(FrontEnd& frontend, Backend& backend, DatasetReaderBase &dataReader
#ifdef MCSLAM_PANGOLIN_SUPPORT
    , OpenGlViewer &glViewer
#endif
) {
    int seq = 1;
    float time_per_frame = 0.0;

    std::cout << "MC-SLAM Controls:" << std::endl;
    std::cout << "  Press '.' (period) to process next frame manually" << std::endl;
    std::cout << "  Press ESC to exit" << std::endl;
    std::cout << "  Press any other key for continuous processing" << std::endl;

    while (true) {
        int key = waitKey(1);
        if ((key & 255) != 255) {
            int condition = (key & 255);
            switch (condition) {
                case 46: { // '.' key for manual frame processing
                    bool success = updateData(frontend, dataReader);
                    if (success) {
                        process_frames(frontend, backend);
                    } else {
                        std::cout << "No more frames to process." << std::endl;
                        goto exit_loop;
                    }
                    break;
                }
                case 27: { // ESC key for exit
                    std::cout << "Exiting MC-SLAM..." << std::endl;
                    goto exit_loop;
                }
                default: {
                    // Any other key starts continuous processing
                    std::cout << "Starting continuous processing..." << std::endl;
                    goto continuous_processing;
                }
            }
        }
    }

continuous_processing:
    while (true) {
        // Check for ESC key to exit
        int key = waitKey(1);
        if ((key & 255) == 27) {
            std::cout << "Exiting MC-SLAM..." << std::endl;
            break;
        }

        auto startT = high_resolution_clock::now();
        bool success = updateData(frontend, dataReader);

        if (success) {
            process_frames(frontend, backend);
        } else {
            std::cout << "Reached end of data sequence." << std::endl;
            break;
        }

        auto stopT = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(stopT - startT);
        time_per_frame += duration.count();
        
        VLOG_INFO(1) << "Frame " << seq << " processing time: " << duration.count() 
                     << " ms | Average: " << (time_per_frame / seq) << " ms" << std::endl;
        
        auto avg_fps = 1000.0 / (time_per_frame / seq);
        if (seq % 10 == 0) {
            std::cout << "Processed " << seq << " frames, Average FPS: " << avg_fps << std::endl;
        }
        
        seq++;
    }

exit_loop:
    cv::destroyAllWindows();
    
#ifdef MCSLAM_PANGOLIN_SUPPORT
    glViewer.requestFinish();
    while (!glViewer.isFinished()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
#endif
}