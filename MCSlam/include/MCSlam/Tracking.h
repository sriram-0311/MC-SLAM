//
// Created by aryaman on 11/3/23.
//
#ifndef SRC_TRACKING_H
#define SRC_TRACKING_H

#include <boost/json.hpp>
#include <opencv2/flann.hpp>
#include <unordered_map>
#include <mutex>
#include <thread>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/CameraSet.h>
#include <gtsam/slam/SmartProjectionRigFactor.h>
#include "MCSlam/MultiCameraFrame.h"

using namespace gtsam;

class Tracking
{
public:
    /**
     * Constructor
     */
    Tracking();
    ~Tracking();

    struct CameraRig
    {
        std::vector<cv::Mat> K_mats;
        std::vector<cv::Mat> extrinsics;
        gtsam::Pose3 b_T_c0;
        gtsam::Pose3 c0_T_w;
        gtsam::Pose3 w_T_b;
        double timestamp;
    };

    /**
     * This function takes in the json file and loads up the positions into a KD tree.
     * @param fileName
     */
    void loadKDTree(const boost::json::value& json_File_value);

    /**
     * Camera rig using the GTSAM's Camera set.
     * @param cameraParameters
     * @param R_T_mats
     * @param b_T_c
     */
    void createCameraRig(std::vector<gtsam::Matrix>& cameraParameters, std::vector<gtsam::Pose3>& R_T_mats,
                         gtsam::Pose3& b_T_c);

    /**
     * This function returns the points closer to the given query position from the tree.
     * @param queryPoint
     */
    void queryPoints(const cv::Mat& queryPoint, cv::Mat& results, cv::Mat& dists);

    /**
     * This function retrieves the union of the landmarks from the frames.
     * @param viewPoints
     */
    void retrieveLandmarks(cv::Mat& viewPoints, std::map<int, cv::Mat>& descriptors, std::unordered_map<int, cv::Mat>& landmarksMap);


    std::map<int, std::vector<cv::Mat>> projectLandmarks(std::unordered_map<int, cv::Mat>& lid_points_pair,
                                                         std::map<unsigned int, std::vector<cv::KeyPoint>> &keypoints,
                                                         std::map<int, cv::Mat>& descriptors,
                                                         std::map<unsigned int, std::vector<cv::Mat>>& descriptors_map,
                                                         std::map<int, std::vector<int>>& landmark_ids);

    /**
     * This function returns all the poses from the json file
     * @return allPoses
     */
    std::vector<cv::Mat> getAllPoses(cv::Mat Tbc);

    /**
     * This function queries the images in current frame and return the best matches for each keypoint reprojected
     * @return cameraRig
     */
    std::map<int, std::vector<cv::KeyPoint>> queryCurrentFrame(std::map<unsigned int, std::vector<cv::KeyPoint>>& keyPoints,
                                                               std::map<int, cv::Mat>& descriptors,
                                                               std::map<unsigned int, std::vector<cv::Mat>>& descriptors_map,
                                                               std::map<int, std::vector<cv::Mat>>& projectedLandmarks,
                                                                std::map<int, std::vector<int>>& landmark_ids,
                                                               MultiCameraFrame* currentFrame, ORBextractor* orbextractor);
    /**
     * This function takes the bestMatches map from queryCUrretnFrame and visulizes the matches keypoints on the images
     * @param bestMatches
     * @param currentFrame
     */
     static void visualizeMatches(std::map<int, std::vector<cv::KeyPoint>>& bestMatches, MultiCameraFrame* currentFrame);

    void visualizeMatches(MultiCameraFrame* currentFrame);

    /**
     * This function returns a map between l_id 3d point pair corresponding to the given ID.
     * @param ID
     * @param lid_points_pair
     */
    void processLandmarks(int ID, std::unordered_map<int, cv::Mat>& lid_points_pair, std::map<int, cv::Mat>& descriptors);

    /**
    * Function to load the positions from the json file.
    */
    void loadPositions();

    CameraRig cameraRig;
    int imgCols, imgRows;
    std::vector<cv::Mat> posesFromMap;
    boost::shared_ptr<gtsam::CameraSet<PinholePose<Cal3_S2>>> multi_cam_rig;

private:
/**
     * This function loads the pose from the json file.
     * @param jsonPoseValue
     */
    cv::Mat loadPoseFromJson(const boost::json::value& jsonPoseValue);

    /**
     * Calulates the inverse of the given transformation matrix.
     * @param a_T_b
     * @param b_T_a
     */
    static void inverse(cv::Mat& a_T_b, cv::Mat& b_T_a);

    /**
     * Using GTSAM's camera rig to project the landmarks to the image coordinates.
     * @param lid_points_pair
     * @param cam_ind
     * @param keypoint
     */
    std::map<int, std::vector<cv::Mat>> project_(std::unordered_map<int, cv::Mat>& lid_points_pair,
                  std::map<unsigned int, std::vector<cv::KeyPoint>>& keypoint, std::map<int, cv::Mat>& descriptors,
                  std::map<unsigned int, std::vector<cv::Mat>>& descriptors_map, std::map<int, std::vector<int>>& landmark_ids);

    /**
     * Helper function to visualize the keypoints.
     * @param keyPoints
     * @param timestamp
     */
    static void visualizeKeyPoints(std::map<int, std::vector<cv::KeyPoint>>& keyPoints, double timestamp, int num_cams);

    /**
     *
     * @param filePath
     * @param partialName
     */
    static std::string findMatchingFile(std::string& filePath, std::string& partialName);

    void querryEachFrame(int cameraIndex, std::vector<cv::KeyPoint>& keyPoints, std::vector<cv::Mat>& descriptors,
                         std::map<int, std::vector<cv::KeyPoint>>& bestMatches,
                         std::map<int, std::vector<cv::Mat>>& bestMatchLandmarks,
                         std::map<int, std::vector<cv::Mat>>& projectedLandmarks,
                         std::map<int, std::vector<int>>& bestMatchLandmarkIds,
                         std::map<int, std::vector<int>>& landmark_ids,
                         MultiCameraFrame* currentFrame, ORBextractor* orBextractor); //

    /**
    * Helper function to convert back the json array to the vector.
    * @param jsonArray
    * @param out_data
    */
    template <typename T>
    void arrayToMat(const boost::json::array& jsonArray, std::vector<cv::Mat>& out_data);


    int Knn = 5;
    std::unique_ptr<boost::json::value> jsonParser;
    cv::flann::GenericIndex<cv::flann::L2<double>>* kdTree;
    cv::flann::GenericIndex<cv::flann::L2<double>>* index;
    std::mutex landmarkMapMutex;
    std::mutex currentFrameKeyPointMutex;
    cv::Mat trainData;
    MultiCameraFrame* mcurrentFrame;
    ORBextractor* morbextractor;


};





#endif //SRC_TRACKING_H
