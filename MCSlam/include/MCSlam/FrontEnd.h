//
// Created by auv on 6/6/20.
//

#ifndef LIGHT_FIELDS_ROS_FRONTEND_H
#define LIGHT_FIELDS_ROS_FRONTEND_H
#pragma once
#include <boost/json.hpp>
#include <boost/assign.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/filesystem.hpp>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "MCDataUtils/CamArrayConfig.h"
#include "MCSlam/ORBVocabulary.h"
#include "MCSlam/ORBextractor.h"
#include "MCSlam/FrontEndBase.h"
#include "MCSlam/GlobalMap.h"
#include "common_utils/StringEnumerator.hpp"
#include "common_utils/matplotlibcpp.h"
#include "common_utils/utilities.h"
#include "MCSlam/LoopCloser.h"
#include "MCSlam/relocalization.h"
#include "MCSlam/Tracking.h"
#include "gtsam/geometry/triangulation.h"
#include "MCSlam/GtsamFactorHelpers.h"
#include "MCSlam/utils.h"
#include "MCSlam/time_measurement.hpp"
#include <tuple>
#include <memory>
#include <chrono>
#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <opengv/types.hpp>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_unstable/slam/ProjectionFactorPPP.h>
#include <gtsam_unstable/slam/ProjectionFactorPPPC.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <yaml-cpp/yaml.h>

#include "MCSlam/utils.h"
#include <boost/assign.hpp>
#include <boost/assign/std/vector.hpp>
#include "MCSlam/newGPSFactor.h"
#include <boost/json.hpp>

/// OPENGV ///
#include <Eigen/Eigen>
#include <opengv/types.hpp>
#include <memory>
#include <opengv/relative_pose/NoncentralRelativeAdapter.hpp>
#include <opengv/sac_problems/relative_pose/NoncentralRelativePoseSacProblem.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/NoncentralAbsoluteAdapter.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>
#include <opengv/point_cloud/methods.hpp>
#include <opengv/point_cloud/PointCloudAdapter.hpp>
#include <opencv2/core/eigen.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>



using namespace std;
using namespace cv;
using namespace opengv;

//#include "QueryResults.h"
typedef fbow::Vocabulary ORBVocabularyfbow;
typedef fbow::fBow2 bowFeatureVector;
typedef fbow::fBow bowVector;
typedef unsigned int NodeId;
typedef unsigned int EntryId;
//typedef QueryResults QueryResults;

typedef gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3_S2>> Cameras;


enum INIT_COND{
    MIN_FEATS =0,
    RANSAC_FILTER=1
};
enum POSEST_ALGO{
    PC_ALIGN=0,
    SEVENTEEN_PT=1,
    G_P3P=2
};
enum INTER_MATCH{
    BF_MATCH=0,
    BoW_MATCH=1
};

class FrontEnd: public FrontEndBase {
public:
    // TEST MODULE :
    struct LOOPER_PAYLOAD
    {
        bool isLoop = false;

    };

    FrontEnd(const string &strSettingsFile, CamArrayConfig &camconfig, bool debug, const string &calibFilePath,
             bool useIMU, bool useGPS, bool relocalization, string& customVoc, string& dataBase, string& graphLogs,
             string& mapLogs, bool navability, int ImuMapFrame)
            : DEBUG_MODE(debug), useIMU(useIMU), relocal(relocalization), camconfig_(camconfig), frontend_config_file(strSettingsFile),
              calib_file_path(calibFilePath), mapFile(mapLogs), dbFile(dataBase), customVocFile(customVoc),
              graphLogsFile(graphLogs), useGPS(useGPS), navability(navability), ImuMapFrame(ImuMapFrame) {
        //Check and load settings file
        cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);
        cv::FileStorage calibSettings(calibFilePath, cv::FileStorage::READ);
        if(!fSettings.isOpened())
        {
            cerr << "Failed to open settings file at: " << strSettingsFile << endl;
            exit(-1);
        }


        //Load ORB Vocabulary
        VLOG(0) <<  "Loading ORB Vocabulary. This could take a while..." << endl;
        string voc_file = fSettings["Vocabulary"];
        // read the vocabulary and create the voc object
        orb_vocabulary = new ORBVocabulary();
        bool bVocLoad = orb_vocabulary->loadFromTextFile(voc_file);

        string fbow_voc_file = fSettings["FBOWVocabulary"];
        orb_vocabulary_fbow = new ORBVocabularyfbow();
        orb_vocabulary_fbow->readFromFile(fbow_voc_file);

        //TODO: Get this from the config file in the future.
        database_vocab = new ORBVocabulary();
        VLOG(0) << "Loading the database vocab ..." << endl;
        database_vocab->load(customVocFile);

        if(!bVocLoad)
        {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Failed to open at: " << voc_file << endl;
            exit(-1);
        }
        VLOG(0) << "Vocabulary loaded!" << endl;

        //init_cond_ = (INIT_COND)fSettings["InitCondition"];
        //posestAlgo_ = (POSEST_ALGO)fSettings["PoseEstimation"];
        init_cond_=  static_cast<INIT_COND>((int)fSettings["InitCondition"]);
        posestAlgo_=  static_cast<POSEST_ALGO>((int)fSettings["PoseEstimation"]);
        interMatch_ = static_cast<INTER_MATCH>((int)fSettings["InterMatch"]);

        kf_translation_threshold = (double)fSettings["KFBaselineThresholdTranslation"];
        // cout<<"kf threshold T: "<<kf_translation_threshold;

        kf_rotation_threshold = (double)fSettings["KFBaselineThresholdRotation"];
        // cout<<"kf threshold R: "<<kf_rotation_threshold;

        string logDir = fSettings["LogDir"];
        boost::filesystem::path dir(logDir.c_str());
        if(boost::filesystem::create_directory(dir))
        {
            std::cerr<< "Directory Created: "<<logDir<<std::endl;
        }
        dir /= logFileName_;
        logFile_.open (dir.string());
        boost::filesystem::path dir2(logDir.c_str());
        dir2 /= logFileName2_;
        //logFile2_.open (dir2.string());
        //logFile_ << "Writing poses and landmarks to a file.\n";

        // Load ORB parameters
        int nFeatures = fSettings["ORBextractor.nFeatures"];
        float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
        int nLevels = fSettings["ORBextractor.nLevels"];
        int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
        int fMinThFAST = fSettings["ORBextractor.minThFAST"];

        // create the object to compute ORB features
        orBextractor = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
        for(int i =0; i <camconfig.num_cams_ ; i++){
            orBextractors.push_back(new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST));
        }

        //create the ORBDatabase object to parse and store images into Bag of words vector
        orb_database = new ORBDatabase(*orb_vocabulary, true , 2);

        //compute the rectification homography
        currentFramePose = cv::Mat::eye(3, 4, CV_64F);
        //currentFramePose1 = cv::Mat::eye(3, 4, CV_64F);
        currentFramePose2 = cv::Mat::eye(3, 4, CV_64F);
        current_frameId = 0;
        currentKFID = 0;
        //initialized_ = false;
        initialized_ = NOT_INITIALIZED;
        initializationTrials = 0;
        //map object
        map = new GlobalMap();

        initialized_mono_ = false;
        currentFramePose_mono = cv::Mat::eye(3, 4, CV_64F);
        map_mono = new GlobalMap();

        if(relocal)
        {
            // TODO: Change the db files.
            // Initialize Relocalization params.
            VLOG(0) << "Initializing Relocalization Modules ..." << std::endl;
            // Initialize the functions.
            std::string db_file = dbFile;
            std::string map_file = mapFile;
            json_file_value = loadMap(map_file);
            relook = new Relocalization(orb_vocabulary, database_vocab, orBextractor, db_file, json_file_value);
            relook->K_mat = camconfig_.K_mats_;
            tracer = std::make_unique<Tracking>();
            tracer->loadKDTree(json_file_value);
        }
        else
        {
            // Mapping case, search for loops.
            initializeLoopClosure();
            prev_point = -1;
            // Change the log file location here:
            logFile2_.open(graphLogsFile, std::ofstream::out | std::ofstream::trunc);
            logFile2_.close();
            logFile2_.open(graphLogsFile, std::ios_base::app);
            EntryCount = 0;
        }

        prev_point = -1;
        // Change the log file location here:

        logFile2_.open(graph_logs , std::ofstream::out | std::ofstream::trunc);
        logFile2_.close();
        logFile2_.open(graph_logs, std::ios_base::app);

        EntryCount = 0;
//        lcLF_ = new LoopCloser(orb_vocabulary);
//        lcMono_ = new LoopCloser(orb_vocabulary);
//        depthProc = new DepthReconstructor(1, camconfig_.iqm_size_, true, false);
//        depthProc->init(camconfig_.K_mats_[0] , camconfig_.dist_coeffs_[0],  camconfig_.K_mats_[1] , camconfig_.dist_coeffs_[1],camconfig_.R_mats_[1], camconfig_.t_mats_[1]);

        convertCamConfig_CV2GTSAM(camconfig, RT_Mats);
        convertCamConfig_CV2GTSAM(camconfig, RT_Mats_init);

        clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        sparse_recon.reserve(500);


        // imu
        initialized_imu = NOT_INITIALIZED;

        if(useIMU){
            if(!calibSettings.isOpened())
            {
            cerr << "Failed to open calib Settings file at: " << calibFilePath << endl;
            exit(-1);
            }
            VLOG(0) << " calib file "  << calibFilePath << endl;

            // Read IMU calibration parameters
            FileStorage fs(calibFilePath, FileStorage::READ);

            if (fs["imu"].isNone())
            {
                cerr << "IMU calibration parameters not found in " << calibFilePath << endl;
                exit(-1);
            }
            else {

                FileNode imuNode = fs["imu"];
                imuNode["acc_noise"] >> acc_n;
                imuNode["acc_walk"] >> acc_w;
                imuNode["gyr_noise"] >> gyr_n;
                imuNode["gyr_walk"] >> gyr_w;
                imuNode["g_norm"] >> g_norm;
                Mat_<double> R = Mat_<double>::zeros(3,3);
                Mat_<double> t = Mat_<double>::zeros(3,1);
                FileNode tn = imuNode["Tbc"];
                if (tn.empty()) {
                    R(0,0) = 1.0; R(1,1) = 1.0; R(2,2) = 1.0;
                    t(0,0) = 0.0; t(1,0) = 0.0; t(2,0) = 0.0;
                } else {
                    FileNodeIterator fi2 = tn.begin(), fi2_end = tn.end();
                    int r = 0;
                    for (; fi2 != fi2_end; ++fi2, r++) {
                        if (r==3)
                            continue;
                        FileNode f2 = *fi2;
                        R(r,0) = (double)f2[0]; R(r,1) = (double)f2[1]; R(r,2) = (double)f2[2];
                        t(r,0) = (double)f2[3];  // * 1000; //. 1000 for real rig we donot have a scale yet.. not sure what metrics this will be
                    }
                }
                Eigen::Matrix3d R_eig;
                cv::cv2eigen(R, R_eig);
                Eigen::Vector3d t_eig;
                cv::cv2eigen(t, t_eig);
                Tbc_cv = Mat::eye(4, 4, CV_64F);
                R.copyTo(Tbc_cv(cv::Range(0, 3), cv::Range(0, 3)));
                t.copyTo(Tbc_cv(cv::Range(0, 3), cv::Range(3, 4)));
                t.copyTo(Tbc_Relocalization(cv::Range(0, 3), cv::Range(3, 4)));

                Tbc.block<3,3>(0,0) = R_eig;
                Tbc.block<3,1>(0,3) = t_eig;

                VLOG(0) << endl << "Tbc - \n" << Tbc << endl;
            }
        }
        /// Set the camera rig for the tracking module.
        if(relocal and useIMU)
        {
//            //TODO: Clean this up.
            VLOG(0) << "Creating the camera rig for the tracking module ..." << std::endl;
            // Convert the K_mat to gtsam format.
            std::vector<gtsam::Matrix> K_mats_gtsam;
            for(const auto& cam : camconfig_.K_mats_)
            {
                gtsam::Matrix K_gtsam(3,3);
                cv::cv2eigen(cam, K_gtsam);
                K_mats_gtsam.push_back(K_gtsam);
            }
            // Combine the R and T matrices.
            std::vector<gtsam::Pose3> R_T_mats_gtsam;
            for(int i = 0; i < camconfig_.R_mats_.size(); i++)
            {
                gtsam::Matrix R_gtsam(3,3);
                cv::cv2eigen(camconfig_.R_mats_[i], R_gtsam);
                gtsam::Point3 T_gtsam;
                cv::cv2eigen(camconfig_.t_mats_[i], T_gtsam);
                gtsam::Rot3 R_gtsam_rot(R_gtsam);
                gtsam::Pose3 R_T_gtsam(R_gtsam_rot, T_gtsam);
                R_T_mats_gtsam.push_back(R_T_gtsam);
            }
            // Convert the Tbc to gtsam format.
            gtsam::Pose3 Tbc_gtsam;
            gtsam::Matrix R_gtsam(3,3);
            cv::cv2eigen(Tbc_cv(cv::Range(0, 3), cv::Range(0, 3)), R_gtsam);
            gtsam::Point3 T_gtsam;
            cv::cv2eigen(Tbc_cv(cv::Range(0, 3), cv::Range(3, 4)), T_gtsam);
            gtsam::Rot3 R_gtsam_rot(R_gtsam);
            Tbc_gtsam = gtsam::Pose3(R_gtsam_rot, T_gtsam);
            tracer->createCameraRig(K_mats_gtsam, R_T_mats_gtsam, Tbc_gtsam);
            // tracer->createCameraRig(camconfig_.K_mats_, camconfig_.R_mats_, camconfig_.t_mats_, Tbc_cv);
        }


        if(useGPS){

            if (!calibSettings.isOpened()) {
                cerr << "Failed to open calib Settings file at: " << calibFilePath << endl;
                exit(-1);
            }
            // Read GPS calibration parameters
            FileStorage fs(calibFilePath, FileStorage::READ);

            if (fs["gps"].isNone())
            {
                cerr << "GPS calibration parameters not found in " << calibFilePath << endl;
                exit(-1);
            }
            else {

                FileNode gpsNode = fs["gps"];
                FileNode tn = gpsNode["Tbg"];
                Mat_<double> R = Mat_<double>::zeros(3,3);
                Mat_<double> t = Mat_<double>::zeros(3,1);
                if (tn.empty()) {
                    VLOG(0) << "empty";
                    R(0,0) = 1.0; R(1,1) = 1.0; R(2,2) = 1.0;
                    t(0,0) = 0.0; t(1,0) = 0.0; t(2,0) = 0.0;
                } else {

                    VLOG(0) << "found";
                    FileNodeIterator fi2 = tn.begin(), fi2_end = tn.end();
                    int r = 0;
                    for (; fi2 != fi2_end; ++fi2, r++) {
                        if (r==3)
                            continue;
                        FileNode f2 = *fi2;
                        R(r,0) = (double)f2[0]; 
                        R(r,1) = (double)f2[1]; 
                        R(r,2) = (double)f2[2];
                        t(r,0) = (double)f2[3];  
                    }
                }

                Eigen::Matrix3d R_eig;
                cv::cv2eigen(R, R_eig);
                Eigen::Vector3d t_eig;
                cv::cv2eigen(t, t_eig);
                Tbg = Mat::eye(4, 4, CV_64F);
                R.copyTo(Tbg(cv::Range(0, 3), cv::Range(0, 3)));
                t.copyTo(Tbg(cv::Range(0, 3), cv::Range(3, 4)));

                VLOG(0) << endl << "Tbg" << Tbg << endl;
        }
        // compute_overlap();
    }

}
    ~FrontEnd(){

    }

    void saveORBDatabase() const;


    void compute_overlap();
    //MOno
    bool initialization_mono(vector<DMatch> &matches_mono);
    void estimatePose_Mono();
    void findMatchesMono(MultiCameraFrame* lf_prev, MultiCameraFrame* lf_cur, int cam_ind, std::vector<DMatch>& matches);
    void insertKeyFrame_Mono();
    void normalizeKps(vector<cv::Point2f>& kps, vector<cv::Point2f>& kps_n, Mat& T);

    void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);
    void Triangulate(const cv::Point2f &kp1, const cv::Point2f &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);
    void getDataForGTSAMTriangulation(MultiCameraFrame *lf, int iMatchInd, Cameras &cameras,
                                      gtsam::Point2Vector &measurements, vector<int> &compCamInds,
                                      vector<int> &octaves);
    int triangulateMatches(MultiCameraFrame* prev_kf, MultiCameraFrame* curFrame, vector<DMatch> inter_matches, vector<bool>& inliers, vector<Mat>& triangulatedPts, vector<double>& depthVec);
    double getSceneDepthStats(MultiCameraFrame* kf);
//    int triangulateNeighbors(std::map<int, MultiCameraFrame*> kfMap,  vector<double>& depthVec);
    int triangulateNeighbors(std::map<int, MultiCameraFrame*>& kfMap, std::map<int, vector<DMatch>>& interMatches, vector<double>& depthVec);
    int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::Point2f>& kps1,
                const vector<cv::Point2f>& kps2, vector<bool> &inliers,
                const cv::Mat &K, vector<cv::Point3f> &P3D, float th2,
                vector<bool> &good, float &parallax, const cv::Mat &R1, const cv::Mat &t1);
    bool checkTriangulation(vector<Mat>& PJs, vector<Point2f>& kps, cv::Mat& P3D, float &parallax);
    bool ReconstructF(const vector<cv::Point2f>& kps1, const vector<cv::Point2f>& kps2,
                      vector<bool> &inliers, cv::Mat &F, cv::Mat &K,float sigma,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D,
                      vector<bool> &vbTriangulated, float minParallax, int minTriangulated);
    cv::Mat solveF_8point(const vector<cv::Point2f> &kps1, const vector<cv::Point2f> &kps2);
    void generateRandomIndices(int max_ind, vector<vector<int>>& indices_set, int num_sets);
    float CheckFundamental(const cv::Mat &F, vector<cv::Point2f>& kps1,
                           vector<cv::Point2f>& kps2, vector<bool> &inliers, float sigma);
    void FindFundamentalMatrix(vector<Point2f> kps1, vector<Point2f> kps2, Mat& F, vector<bool> &inliers);

    cv::Mat getPose_Mono();
    vector<cv::Mat> getAllPoses_Mono();


    void createFrame(vector<Mat> img_set, vector<Mat> segmap_set, double timeStamp);
    void computeRefocusedImage();
    void filterIntraMatches( std::vector<IntraMatch>& matches_map, MultiCameraFrame* currentFrame,
                             std::vector<IntraMatch>& filtered_intra_matches,
                             vector<DBoW2::NodeId >& words_, set<DBoW2::NodeId >& filtered_words);

    void bundleAdjustIntraMatches(std::vector<IntraMatch>& tmp_intra_matches);

    void obtainLfFeatures( std::vector<IntraMatch>& matches_map, MultiCameraFrame* currentFrame,
                           std::vector<IntraMatch>& filtered_intra_matches,
                           vector<DBoW2::NodeId >& words_, set<DBoW2::NodeId >& filtered_words);

    void processFrame();
    void processFrameNon();
    bool trackFrame();
    bool initialization();
    bool initialization_non_overlapping(vector<DMatch> &mono_matches);
    void insertKeyFrame();
    void deleteCurrentFrame();

    //void BruteForceMatching(Mat img1, Mat img2, vector<Mat> descs1, vector<Mat> descs2, vector<KeyPoint> kps1, vector<KeyPoint> kps2);
    void findInterMatches(MultiCameraFrame* lf_prev, MultiCameraFrame* lf_cur, std::vector<DMatch>& matches_z_filtered,
                          std::vector<DMatch>& matches_mono_z_filtered, bool viz);
    void findInterMatches(MultiCameraFrame* lf_prev, MultiCameraFrame* lf_cur, std::vector<DMatch>& matches_z_filtered,
                          bool viz);
    void findInterMatchesBow(MultiCameraFrame *lf_prev, MultiCameraFrame *lf_cur, std::vector<DMatch> &matches,
                             std::vector<DMatch> &mono_matches, bool viz);
    void findInterMatchesBow(MultiCameraFrame *lf_prev, MultiCameraFrame *lf_cur, std::vector<DMatch> &matches,
                             bool viz);
    void InterMatchingBow( DBoW2::FeatureVector lfFeatVec1, DBoW2::FeatureVector lfFeatVec2, std::vector<cv::Mat> img_desc1,
                           std::vector<cv::Mat> img_desc2, vector<unsigned int>& indices_1,
                           vector<unsigned int>& indices_2);
    void InterMatchingFBow( bowFeatureVector lfFeatVec1, bowFeatureVector lfFeatVec2, vector<Mat> img_desc1,
                           vector<Mat> img_desc2, vector<unsigned int>& indices_1,
                           vector<unsigned int>& indices_2);
    void InterMatchingBow(MultiCameraFrame* lf_prev, MultiCameraFrame* lf_cur, vector<unsigned int>& indices_1,
                          vector<unsigned int>& indices_2,set<DBoW2::NodeId>& words );
    void get3D_2DCorrs(MultiCameraFrame *lf, int featInd, Mat pose, vector<Mat> &projmats, vector<Mat> &PJs, std::vector<Mat_<double>> &xs,
                       vector<int> &view_inds, vector<Point2f> &kps, vector<int> &octaves);
    int estimatePoseLF(MultiCameraFrame* lf_prev, MultiCameraFrame* lf_cur,
                                 std::vector<DMatch>& matches, vector<bool>& inliers, Mat& T, bool SAC);
    int poseFromPCAlignment (MultiCameraFrame* lf_prev, MultiCameraFrame* lf_cur,
    std::vector<DMatch>& matches, vector<bool>& inliers, Mat& T,  bool SAC);
    int poseFromSeventeenPt(MultiCameraFrame* lf_prev, MultiCameraFrame* lf_cur,
                            std::vector<DMatch>& matches, vector<bool>& inliers, Mat& T,  bool SAC);
    int absolutePoseFromGP3P(MultiCameraFrame* lf_prev, MultiCameraFrame* lf_cur,
                            std::vector<DMatch>& matches, vector<bool>& inliers, Mat& T,  bool SAC);
    void OptimizePose(vector<DMatch> matches, vector<int> lids, vector<bool>& inliers,
                      transformation_t nonlinear_transformation, Mat& poseOut);


    cv::Mat getPose();
    cv::Mat getPose_seventeen();
    cv::Mat getPose_gp3p();  //Relative pose
    vector<cv::Mat> getAllPoses_gp3p();
    vector<cv::Mat> getAllPoses();
    vector<cv::Mat> getAllPosesRelocalization();
    vector<cv::Mat> getAllPoses1(); // Absolute pose
    vector<cv::Mat> getAllPoses_dummy(); //
    void getMapPoints(vector<Point3f>& mapPoints); // for dummy frames added using gps + imu

    void reset();
    void drawMatchesArrows(cv::Mat& img, vector<KeyPoint>& kps_prev, vector<KeyPoint>& kps_cur,  std::vector<DMatch> matches, Mat mask, cv::Scalar color);
    void drawInterMatches(MultiCameraFrame *lf_prev, MultiCameraFrame *lf_cur, vector<DMatch> matches,
                          vector<bool> inliers, string windowTitle);
    void drawInterMatch(MultiCameraFrame *lf_prev, MultiCameraFrame *lf_cur, IntraMatch im1,IntraMatch im2, string windowTitle);
    void check_tri_sp(int num_views, vector<int> view_inds, IntraMatch& matches,  MultiCameraFrame* lf);

    void writeTrajectoryToFile(const string &filename,  bool mono);

    void featureStats();

    void drawInterMatch(MultiCameraFrame* lf_prev, MultiCameraFrame* lf_cur, int prev_ind, int cur_ind);
    void prepareOpengvData(vector<Point2f>& kps_1, vector<Point2f>& kps_2, int cam1, int cam2);
    void tracksForHist();
//    void searchLocalMap2(MultiCameraFrame *prevKF, vector<DMatch> inter_matches_with_landmarks, vector<bool> inliers,
//                        vector<DMatch> &inlierAllMapMatches, Mat &refinedPose, vector<int> &alllids, std::map<int, MultiCameraFrame*>& neighbors);
    void searchLocalMap2(MultiCameraFrame *prevKF, vector<DMatch> inter_matches_with_landmarks, vector<bool> inliers,
                        vector<DMatch> &inlierAllMapMatches, Mat &refinedPose, vector<int> &alllids, std::map<int, MultiCameraFrame*>& neighbors, std::map<int, vector<DMatch>>& interMatches);
    void searchLocalMap(MultiCameraFrame *prevKF, vector<DMatch> inter_matches_with_landmarks, vector<bool> inliers,
                        vector<DMatch> &inlierAllMapMatches, Mat &refinedPose, vector<int> &alllids);
    void writeLogs( string stats_file);
    void project3dPoint(const Mat &pt3d, int camInd,Mat Twb, double& exp_x, double& exp_y);

    std::string matToString(const cv::Mat& mat);


    /**
     * Returns bool as to if relocalization was successful.
     * @return
     */
    bool checkGlobalRelocalization();

    /**
     * Function to initialize VPR system parameters during mapping.
     */
    void initializeLoopClosure();


    /**
     * Converts the vectors to JSON array format.
     * @param vec
     * @return
     */
    boost::json::array vecToJson(const std::vector<std::vector<double>>& vec);

    /**
     * Converts cv::MAT to JSON array format.
     * @param mat
     * @return
     */
    boost::json::array matToJson(const std::vector<cv::Mat>& vec);

    /**
     * Function to load the json object with the landmarks file.
     * @param fileName
     * @return
     */
    static boost::json::value loadMap(const string &fileName);

    /**
     * Appends the logs to json format.
     */
    void appendLogsJSONformat(const std::string& entryId, const std::vector<int>& landmarks,
                              const std::vector<std::vector<double>>& points,
                              std::vector<cv::Mat>& descriptor, double timestamp, const std::vector<double>& position,
                              const boost::json::array& poseArray);

    /**
     *  Serialize the json file and save to disk.
     */
    void serializeJSONObject() const;

    /**
     * Append the data to file name.
     * @param file_nam
     * @param loop_detection
     */
    void appendLogs(bool loop_detection, gtsam::NavState prevState_, gtsam::imuBias::ConstantBias prevBias_);
    std::vector<std::vector<double>> matToVec(const cv::Mat& mat);

    /**
     * IMU message to be appended to the log file.
     * @param message
     */
    void appendIMUValue(sensor_msgs::Imu *message);


    bool startTrackingModule(gtsam::NavState state);
    void refinePose();

    bool visionTrackingModule();


    //only for mono-comparison
    GlobalMap* map_mono;
    bool initialized_mono_;
    vector<MultiCameraFrame*> lfFrames_mono;
    vector<cv::Mat> allPoses_mono;
    vector<Mat> neighborPoses_;
    vector<double> poseTimeStamps_mono;
    cv::Mat currentFramePose_mono;
    int initializationTrials =0;
    Mat prev_rvec, prev_tvec;
    bool trackingStatus = false;

    bool DEBUG_MODE;
    INIT_STATE initialized_ = NOT_INITIALIZED;
    INIT_COND init_cond_= RANSAC_FILTER;
    POSEST_ALGO posestAlgo_ = SEVENTEEN_PT;
    INTER_MATCH interMatch_ = BoW_MATCH;
    bool tracking_failure_= false;
    int num_frames_since_tracking_failure = 0;
    int num_trials_to_track = 2;// hardcoded, should be a parameter


    // imu
    INIT_STATE initialized_imu = NOT_INITIALIZED;
    bool useIMU = false;
    int ImuMapFrame = 0;
    bool relocal = false;  // Bool to trigger relocalization modules.
    bool globallyRelocalized = false;
    bool navability = false;

    string graph_logs = "";

    double kf_translation_threshold;
    double kf_rotation_threshold;

    int current_frameId, currentKFID;
    MultiCameraFrame* currentFrame;
    vector<MultiCameraFrame*> lfFrames;
    CamArrayConfig& camconfig_;
    string frontend_config_file;
    string calib_file_path;
    string logFileName_ = "poses_landmark.txt";
    string logFileName2_ = "poses_stats.txt";
    boost::json::object json_file_obj;
    boost::json::value json_file_value;
    int EntryCount;
    int prev_point;
    ofstream logFile_, logFile2_;
    ORBextractor* orBextractor;
    vector<ORBextractor*> orBextractors;
    ORBVocabulary* orb_vocabulary;
    ORBVocabularyfbow* orb_vocabulary_fbow;
    ORBVocabulary* database_vocab;
    ORBDatabase* orb_database;
    GlobalMap* map;
    //MulticamElas* reconstructor;
    DepthReconstructor* depthProc;

    // Loop Closure functionalities.
    LoopCloser* looper;
    LoopCloser::Detection_Frame result;

    Relocalization* relook;
    std::unique_ptr<Tracking> tracer;
    std::string mapFile;
    std::string dbFile;
    std::string customVocFile;
    std::string graphLogsFile;



    // LoopCloser* lcMono_;
    // LoopCloser* lcLF_;
    // LoopCloser* lcMasked_;

    vector<Mat> overlap_masks;
    vector<float> non_overlap_fraction;
    //cv::Mat currentFramePose;
    //EXtra variables for pose visualization
    cv::Mat currentFramePose1, currentFramePose2, currentFramePose3;
    cv::Mat Tbc_cv = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat Tbc_Relocalization = cv::Mat::eye(4, 4, CV_64F);

    float mapping_time=0.0;
    vector<double> poseTimeStamps;
    vector<cv::Mat> keyFramePoses;

    cv::Ptr<cv::CLAHE> clahe;

    // opengv pose estimation stuff
    vector<int> correspondences_1, correspondences_2;
    opengv::bearingVectors_t bearings_1, bearings_2;
    opengv::rotations_t rotations;
    opengv::translations_t translations;
    vector<gtsam::Pose3> RT_Mats, RT_Mats_init;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // IMU PReintegration related
    float acc_n, gyr_n, acc_w, gyr_w;
    float g_norm;
    Eigen::Matrix4d Tbc = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d Tbw;
    Eigen::Quaterniond world_imu_frame(std::deque<sensor_msgs::Imu>& imu_messages);
    Eigen::MatrixXd kabsch(Eigen::MatrixXd A, Eigen::MatrixXd B);
    void imu_initialize_noise(gtsam::imuBias::ConstantBias prior_imu_bias);
    void imu_initialize(std::deque<sensor_msgs::Imu> imu_msgs, double image_time);
    void imu_values_removal(double image_time);
    void imu_preintegration(double image_time);
    bool imu_initialized = false;
    gtsam::PreintegratedCombinedMeasurements *imu_integrator_comb;
    // gtsam::PreintegratedImuMeasurements *imu_integrator_opt;
    double last_imu_time = -1;
    gtsam::imuBias::ConstantBias prior_imu_bias;
    Eigen::Matrix3d world_imu_rotation = Eigen::Matrix3d::Zero();
    bool done_rotation_opt = false;
    std::deque<sensor_msgs::Imu> imu_msgs_initialized;
    bool first_keyframe = true;
    double prev_image_time = 0;
    std::deque<sensor_msgs::Imu> imu_msgs_combined;
    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr bias_noise_model;

    Eigen::Vector3d bias_acc;
    Eigen::Vector3d bias_gyr;
    gtsam::NavState lastState_;
    gtsam::imuBias::ConstantBias bias_prior, lastBias_;
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> params;

    void
    TriangulateNewLandmarks(const MultiCameraFrame *lf_cur, MultiCameraFrame *prev_KF,
                            const vector<DMatch> &new_inter_matches, int &num_triangulated, vector<int> &n_rays,
                            vector<double> &depthVec, double &avg_depth);

//    int mapping(const MultiCameraFrame *lf_cur, MultiCameraFrame *prev_KF, const vector<DMatch> &new_inter_matches,
//               std::map<int, MultiCameraFrame *> &neighbors, int num_triangulated, vector<int> &n_rays);
    int mapping(const MultiCameraFrame *lf_cur, MultiCameraFrame *prev_KF, const vector<DMatch> &new_inter_matches,
               std::map<int, MultiCameraFrame *> &neighbors, std::map<int, vector<DMatch>>& interMatches, int num_triangulated, vector<int> &n_rays);


    // IMU related
    // to debug
    bool imu_message_empty = false; // if gps and vision come very close such that they dont have any imu messages, then add a between factor based on this flag

    // GPS related

    std::deque<sensor_msgs::NavSatFix> gps_msgs_combined;
    bool gps_initialized = false;
    std::deque<sensor_msgs::NavSatFix> gps_msgs_initialize;
    std::deque<gtsam::Pose3> vins_poses_initialize;
    void gps_initialize_kabsch(std::deque<sensor_msgs::NavSatFix>  gps_msgs_combined, MultiCameraFrame* prev_kf);
    void gps_ENU_reference_params(sensor_msgs::NavSatFix gps_msg);
    bool gps_init_data_Stored = false;

    bool useGPS = false;            // for now do it in a basic way
    tuple<double, double, double> geodetic_to_enu(sensor_msgs::NavSatFix gps_msg);

    double refLat = 42.3384;
    double refLon = -71.0855;
    double refAlt = 10.0;


    tuple <double, double, double> gps_enu_origin;
    tuple<double, double, double> enu_wrt_origin(tuple<double, double, double> gps_message);


    std::deque<tuple<double, double, double>> gps_msgs_enu;


    cv::Mat T_body_gps = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat Tbg = cv::Mat::eye(4, 4, CV_64F);


    struct gps_message{
        int curr_gps_kf = -1;
        tuple<double, double, double> curr_gps_message_enu;
        tuple<double, double, double> prev_gps_message_enu;
        double tStamp;
    };
    gps_message gps_data_;
    int last_gps_kf = -1;
    bool validGPSmessage(gps_message gps_data_);
    double gps_euclidean(tuple<double, double, double> gps_msg1, tuple<double, double, double> gps_msg2);

    //debug variables
    struct gps_data
    {
        int pose_id;
        bool gps_factor_added = false;
        tuple<double, double, double> gps_message_enu;
        double timestamp;
    };
    gps_data gps_data_stored;
    bool gps_transform_stored = false;

    struct gps_umeyama_data
    {
        sensor_msgs::NavSatFix  gps_msg;
        MultiCameraFrame* prev_keyframe;
        MultiCameraFrame* next_keyframe;
    };
    bool gps_vins_next = false; // bool to store a particular kf as the next vins pose
    vector<gps_umeyama_data> gps_umeyama_data_vec;

    void interpolation_vins_GPS(vector<gps_umeyama_data> gps_umeyama_data_vec, Eigen::MatrixXd &gps_eigen , Eigen::MatrixXd &vins_eigen);
    
    //for dummy kfs
    void insertKeyFrame(gtsam::NavState prevState_, gtsam::imuBias::ConstantBias prevBias_, double timestamp);
    MultiCameraFrame* currentFrameDummy;
    void deleteCurrentDummyFrame();
    void appendGPSValue(gps_message gps_data, int gps_kf);


    // debug
    int dummy_frame_tot_count = 0;
    int dummy_frame_count = 0;
    vector<MultiCameraFrame*> lfFrames_debug;
    bool init_mode = true;
    // final results of relocalization
    std::map<int, std::vector<cv::KeyPoint>> reprojectedKeypoints;
    std::map<int, std::vector<cv::Mat>> reprojectedLandmarks;
    std::map<int, std::vector<int>> reprojectedLandmarkIds;


};


#endif //LIGHT_FIELDS_ROS_FRONTEND_H
