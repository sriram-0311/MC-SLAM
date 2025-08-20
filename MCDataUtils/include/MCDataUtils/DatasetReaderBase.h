//
// Created by Pushyami Kaveti on 9/16/19.
//

#pragma once

#include <string>
#include <vector>
#include <glog/logging.h>
#include <dirent.h>
#include <common_utils/tools.h>

#include "MCDataUtilParams.h"
#include "CamArrayConfig.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include <sensor_msgs/NavSatFix.h>

using namespace std;

class DatasetReaderBase {
public:
    DatasetReaderBase(){}
    ~DatasetReaderBase(){}

    virtual void initialize(MCDataUtilSettings refocus_set) = 0;
    virtual void loadNext(vector<cv::Mat>& imgs) = 0;
    virtual void getNext(vector<cv::Mat>& imgs, double& timeStamp) = 0;
    virtual void getNext(vector<cv::Mat>& imgs, vector<string>& segmaskImgs, double& timeStamp) = 0;
    virtual void getNext(vector<cv::Mat>& imgs, double& timeStamp, std::deque<sensor_msgs::Imu>& imu_msgs) = 0;
    virtual void getNext(vector<cv::Mat>& imgs, double& timeStamp, std::deque<sensor_msgs::Imu>& imu_msgs, std::deque<sensor_msgs::NavSatFix>& gps_msgs) = 0;


    vector<Mat> getKMats(){ return K_mats_ ; }
    vector<Mat> getRectMats(){ return Hrects ; }
    vector<float> getBaselines(){ return baselines ; }
    vector<Mat> getRMats(){ return R_mats_ ; }
    vector<Mat> getTMats(){ return t_vecs_ ; }
    vector<Mat> getKalibrRMats(){ return R_mats_kalibr ; }
    vector<Mat> getKalibrTMats(){ return t_vecs_kalibr ; }
    vector<Mat> getPMats(){ return P_mats_ ; }
    vector<Mat> getDistMats(){ return dist_coeffs_ ; }


    MCDataUtilSettings settings;
    Size calib_img_size_;
    Size img_size_;

    vector<float> baselines;
    vector<Mat> K_mats_;
    vector<Mat> Hrects;
    vector<Mat> R_mats_ , R_mats_kalibr;
    vector<Mat> t_vecs_ , t_vecs_kalibr;
    vector<Mat> dist_coeffs_;
    vector<Mat> P_mats_;

    vector<Mat> img_set;
    //vector<cuda::GpuMat> gpu_img_set;
    cuda::GpuMat temp;

    int num_cams_;
    int UNDISTORT_IMAGES;
    int RADTAN_FLAG;
    int GPU_FLAG;
    string MASK_TYPE;
    int img_counter;



};


