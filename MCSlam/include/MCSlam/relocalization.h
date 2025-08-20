//
// Created by aryaman on 10/4/23.
//

#include <boost/json.hpp>
#include "MCSlam/LoopCloser.h"
#include "MCSlam/MultiCameraFrame.h"
#include "MCSlam/utils.h"


#ifndef SRC_RELOCALIZATION_H
#define SRC_RELOCALIZATION_H

// Type definition to keep things short.
typedef DLoopDetector::TemplatedLoopDetector<DBoW2::FORB::TDescriptor, DBoW2::FORB> DorianLoopDetector;

class Relocalization : public DorianLoopDetector
{
public:

    /**
     * Constructor for Relocalization framework.
     */
    Relocalization(ORBVocabulary* voc, ORBVocabulary* map_vocab, ORBextractor* extract, const std::string& db_file_name,
                   const boost::json::value& json_File_value);

    /**
     * Activates the caller function to check for relocalization.
     */
    bool checkRelocalization(MultiCameraFrame* currentFrame);

    bool checkRelocalizationNavability(MultiCameraFrame *currentFrame);

    /**
     * A helper function to get the size of the orb database.
     * @return
     */
    int getORBDatabaseSize();

    std::vector<cv::Mat> K_mat;
    // Result containing the relocalization point.
    LoopCloser::Detection_Frame result;

    ~Relocalization() override;

private:

    void newVisualizeMatches(vector<gtsam::Point2> currentImagePoints, vector<gtsam::Point2> oldImagePoints, cv::Mat currentImage, cv::Mat oldImage);

    gtsam::Point2 project3DTo2D(const std::vector<double>& pt3d, const DBoW2::EntryId &old_Id);

    gtsam::Point2 project3DTo2D(const std::vector<double> &pt3d, Mat &Pose);

    /**
     * Module to perform geometric verification in the relocalization frames.
     * @param old_entry
     * @param result
     * @param currFeatVec
     * @return
     */

    bool geometricVerification(const EntryId& old_entry, LoopCloser::Detection_Frame& result,
                               DBoW2::FeatureVector& currFeatVec);

    /**
     * Helper function to convert back the json array to the vector.
     * @param jsonArray
     * @param out_data
     */
    template <typename T>
    void arrayToMat(const boost::json::array& jsonArray, std::vector<cv::Mat>& out_data);

    /**
     * Finds the matches between the two frames.
     * @param curr_featVec
     * @param best_featVec
     * @param indices_1
     * @param indices_2
     * @param matched_desc
     */
    void featureMatchesBow(const DBoW2::FeatureVector& curr_featVec,const DBoW2::FeatureVector& best_featVec,
                           vector<unsigned int> &indices_1, vector<unsigned int> &indices_2, std::vector<cv::Mat>& matched_desc);

    /**
     *  This is a function to perform GP3P and compute the pose given the matches and 3D 2D correspondences.
     * @param old_Id
     * @param matches
     * @param result
     * @return
     */
    bool checkAbsolutePose(const DBoW2::EntryId &old_Id, std::vector<cv::DMatch> &matches,
                           LoopCloser::Detection_Frame &result);

    bool checkAbsolutePose(const DBoW2::EntryId &old_Id, std::vector<cv::DMatch> &matches,
                      LoopCloser::Detection_Frame &result, std::map<int, std::vector<double>> &lid_to_point_map,
                      vector<int> &lIds, string &timestamp, int cameraIndex, Mat &Pose);
    /**
     * Gets the corresponding landmarks and l_ids from the json file for a particular landmark id.
     * @param old_Id
     * @param lid_to_point_map
     * @param lids
     */
    void processLandmarks(const DBoW2::EntryId &old_Id, std::map<int, std::vector<double>>& lid_to_point_map, std::vector<int>& lids);

    /**
     * This function looks into the graph_log file and gets the gt pose.
     * @param filename
     * @param Id
     * @param gt_pose
     * @return
     */
    static cv::Mat parseFileAndSaveGroundTruth(const std::string& filename, int Id,
                                        cv::Mat& gt_pose);

    /**
     * This just to check the error between the relative difference between the GT pose and the computed pose.
     * @param pose
     * @param groundTruthPose
     * @return
     */
    static cv::Mat computeRelativePoseError(const cv::Mat& pose, const cv::Mat& groundTruthPose);

    /**
     * Visualize the matches
     * @param matched_Id
     */
    void visualizeMatches(DBoW2::EntryId &matched_Id, std::vector<cv::DMatch> &matches);

    /**
     * Functionality to draw the matches of the matched images.
     * @param current_images
     * @param best_match_images
     * @param matches
     * @param best_match
     */
    void drawMatches(std::vector<cv::Mat> current_images, const std::vector<cv::Mat>& best_match_images,
                       std::vector<cv::DMatch>& matches, DBoW2::EntryId& best_match);

    void bruteForceMatching(Mat &desc1, Mat &desc2, std::vector<cv::DMatch> &matches);

    bool geometricVerificationNavability(const DBoW2::EntryId &old_entry, LoopCloser::Detection_Frame &result,
                                    DBoW2::FeatureVector& currFeatVec, int cameraIndex);

    void getLandmarkDescriptors(const DBoW2::EntryId &old_entry, std::vector<cv::Mat> &vecDesc,
                                map<int, vector<double>> &lid_to_point_map, vector<int> &lids, string &timestamp, Mat &Pose);

    LoopCloser* looper;
    MultiCameraFrame* r_frame{};
    ORBVocabulary* orb_vocabulary;
    ORBextractor* feat_extract;
    std::unique_ptr<boost::json::value> jsonParser;
    boost::json::object navabilityJsonParser;
    boost::json::object navabilityJsonPoseParser;
    std::vector<std::vector<double>> pointCloud;
    double minInlierRatio;

};


#endif //SRC_RELOCALIZATION_H
