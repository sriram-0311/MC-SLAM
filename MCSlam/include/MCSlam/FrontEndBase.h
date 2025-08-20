//
// Created by Pushyami Kaveti on 1/24/22.
//

#ifndef SRC_FRONTENDBASE_H
#define SRC_FRONTENDBASE_H

#include "common_utils/tools.h"
#include "DUtils.h"
#include <mutex>
#include "MCSlam/MultiCameraFrame.h"

using namespace std;

enum INIT_STATE{
    NOT_INITIALIZED =0,
    INITIALIZED = 1,
    REINITIALIZING = 2
};

class FrontEndBase {
public:
    FrontEndBase(){}
    ~FrontEndBase(){}

    virtual  cv::Mat getPose()=0;
    virtual vector<cv::Mat> getAllPoses()=0;
    virtual vector<cv::Mat> getAllPoses1()=0;
    virtual vector<cv::Mat> getAllPoses_gp3p()=0;
    virtual cv::Mat getPose_gp3p()=0;
    virtual vector<cv::Mat> getAllPoses_dummy() = 0;
    virtual void getMapPoints(vector<Point3f>& mapPoints)=0;


    cv::Mat currentFramePose;
    vector<cv::Mat> allPoses;
    vector<cv::Mat> allPoses1, allPoses2;
    vector<cv::Mat> allPoses_dummy;
    vector<cv::Mat> combined_poses;
    vector<double> combined_tStamps;
    ///Vector of sparse reconstruction for viz
    vector<vector<Point3f>> sparse_recon;
    vector<vector<Point3f>> sparse_recon_color;

    std::mutex mMutexPose, mMutexPoints, mMutexImu;

    /// Log variables
    int log_num_intramatches_ = 0;
    int log_num_matches_ = 0 ;
    int log_num_lm_matches_ = 0;
    int log_num_tracked_lms_ =0;
    int log_num_new_matches_ = 0;
    int log_num_triangulated_ = 0;
    int log_KF_=0;


};


#endif //SRC_FRONTENDBASE_H
