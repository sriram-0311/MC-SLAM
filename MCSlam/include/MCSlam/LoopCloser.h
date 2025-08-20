//
// Created by Pushyami Kaveti on 4/18/21.
//

#ifndef SRC_LOOPCLOSER_H
#define SRC_LOOPCLOSER_H

#include <atomic>
#include <thread>
#include <mutex>
#include <filesystem>


#include "MCSlam/ORBVocabulary.h"
#include "MCSlam/MultiCameraFrame.h"
#include "MCSlam/GlobalMap.h"
#include "MCSlam/utils.h"
#include "DUtils/DUtils.h"
#include "DVision/DVision.h"
#include "TemplatedLoopDetector.h"
#include "TemplatedVocabulary.h"
#include "TemplatedDatabase.h"
#include "FORB.h"


//GTSAM
#include <gtsam/geometry/Pose3.h>

// OpenGV
#include <opengv/relative_pose/NoncentralRelativeAdapter.hpp>
#include <opengv/sac_problems/relative_pose/NoncentralRelativePoseSacProblem.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/NoncentralAbsoluteAdapter.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>
#include <opengv/point_cloud/methods.hpp>
#include <opengv/point_cloud/PointCloudAdapter.hpp>

class Relocalization;
// Type definition to keep things short.
typedef DLoopDetector::TemplatedLoopDetector<DBoW2::FORB::TDescriptor, DBoW2::FORB> DorianLoopDetector;

class LoopCloser : public DorianLoopDetector
{
public:
    /**
     * Solver type for the relative pose.
     */
    enum Solver
    {
        SEVENTEENPT,
        ABSOLUTE_POSE
    };
    Solver type;
    struct Detection_Frame
    {
        DLoopDetector::DetectionStatus status;
        EntryId query;
        EntryId match;
        double timeStamp;
        [[nodiscard]] inline bool detection() const
        {
            return status == DLoopDetector::LOOP_DETECTED;
        }
        gtsam::Pose3 relative_pose;
        std::pair<cv::Mat, cv::Mat> pose_to_check; // FOR VISUALIZATION.
        std::vector<int> lIds;
        std::vector<std::vector<std::tuple<int, cv::Point2f>>> measurements;
    };

    /**
     * DLoopDetector Parameters.
     */
    DorianLoopDetector::Parameters dloop_param;

    /**
     * Multi-Camera Parameters.
     */
    std::vector<cv::Mat> K_mat;

    LoopCloser()= default;
    /**
     * LoopCloser constructor (sets the input parameters)
     * @param voc : The vocabulary file.
     * @param params : The parameters to set.
     */
    LoopCloser(ORBVocabulary* voc, ORBextractor* extract);

    /**
     * The loop detector algorithm.
     * @param frame : The current frame of type MultiCameraFrame.
     * @return True or False along with the results of the
     */
    bool callerDetectLoop(MultiCameraFrame* frame, GlobalMap* map, std::vector<MultiCameraFrame*> &keyframes,
                          Detection_Frame& result);

    /**
     * Function to set the input parameters.
     * @param params
     */
    void setParams(DorianLoopDetector::Parameters& params);

    /**
     * A helper function clear the database.
     */
    void clearDataBase();

    /**
     * A helper function to save the orb database to the file.
     * @param fileName
     */
    void saveDatabase(const std::string& fileName);

    /**
     * A helper function to load the ORB Database from the file location
     * @param fileName
     */
    void loadDatabase(const std::string& fileName);

    /**
     * Get the size of the orb db.
     * @return
     */
    int getORBDatabaseSize();

    /**
     * Sets the feature ectractor
     * @param extract
     */
    void setFeatureExtractor(ORBextractor* extract);


    ~LoopCloser() override {
        delete orb_database;
    }

private:
    /**
     * Compute the matches between the best and the current frames for Geometric Verification.
     * @param curr_featVec
     * @param best_featVec
     * @param indices_1
     * @param indices_2
     * @param words
     */
    void featureMatchesBow(const DBoW2::FeatureVector& curr_featVec,const DBoW2::FeatureVector& best_featVec,
                           vector<unsigned int>& indices_1, vector<unsigned int>& indices_2, EntryId best_entry);
    /**
     * Computed the Essential Matrix given the best match and the current image for geometric verification, uses SEVENTEENPT.
     * @param old_Id
     * @param matches
     * @return
     */
    bool checkEssentialMatrix(const EntryId& old_Id, std::vector<cv::DMatch>& matches, Detection_Frame &result);

    /**
     * Geometric Verification using the GP3P algorithm.
     * @param old_Id
     * @param match
     * @param result
     * @return
     */
    bool checkAbsolutePose(const EntryId& old_Id, std::vector<cv::DMatch>& match, Detection_Frame &result);

    /**
     * Function to perform Geometric verification using the current and best matched frames.
     * Uses OpenGV functionalities to match and calculate the Fundamental Matrix
     * @param old_entry
     * @param curr_FeatVec
     * @return
     */
    bool geometricVerification(const EntryId& old_entry, Detection_Frame &result);

    /**
     * Helper function : Visualizer for the matches of the loop closure.
     * @param current_id
     * @param best_match
     */
    void visualize(DBoW2::EntryId current_id, DBoW2::EntryId best_match,
                   std::vector<cv::DMatch>& matches);

    /**
     * Helper function: Turns double values to string type.
     * @param value
     * @return
     */
    static std::string convertDoubleToString(double value);

    /**
     * Helper function : Finds the images for visualization.
     * @param filePrefix
     * @return
     */
    std::string findMatchingFile(std::string &filePath, std::string &partialName);

    /**
     * Helper function : Display the matches in a parallel thread.
     * @param current_images
     * @param best_match_images
     * @param inliers
     */
    void displayImages(const std::vector<cv::Mat>& current_images, const std::vector<cv::Mat>& best_match_images);

    /**
     * Helper function : Draws up the inlier points on the two images.
     * @param current_images
     * @param best_match_images
     * @param matches
     * @param best_match
     */
    void drawMatches(std::vector<cv::Mat> current_images, std::vector<cv::Mat> best_match_images,
                     std::vector<cv::DMatch>& matches, DBoW2::EntryId& best_match);

    /**
     * Helper function: Finds the relative pose between the best match and the current match absolute poses.
     * @param curr_frame_pose
     * @param best_match_pose
     * @return GTSAM Pose3
     */
    static cv::Mat getRelativePose(cv::Mat curr_frame_pose, cv::Mat& best_match_pose);

    /**
    * A helper function to get the lid and the measurements of the matched keyframes.
    */
    void getLIds(std::vector<cv::DMatch> &matches, const DBoW2::EntryId &best_match,
                 std::vector<int>& matched_Lid, std::vector<std::vector<std::tuple<int, cv::Point2f>>> &matched_measurement);

    /**
     *
     * @param kframe
     * @param kfBoW
     * @param kfFeatVec
     */
    void createBoWandFeatVec(MultiCameraFrame* kframe, DBoW2::BowVector& kfBoW,
                             DBoW2::FeatureVector& kfFeatVec);

    void createBoWandFeatVec(vector<Mat> imageDescs, DBoW2::BowVector &kfBoW,
            DBoW2::FeatureVector &kfFeatVec);

    ORBVocabulary* orb_vocab{};
    ORBDatabase* orb_database{};
    ORBextractor* feat_extract{};
    MultiCameraFrame* m_frame{};
    GlobalMap* m_map{};
    DVision::FSolver d_fsolver;
    std::string image_dir;
    DBoW2::BowVector last_bowvec;
    // Create the current Bow and FeatVec initialization.
    DBoW2::BowVector curr_BoW;
    DBoW2::FeatureVector curr_FeatVec;
    std::atomic<bool> display_thread_running{false};

    std::vector<MultiCameraFrame*> key_frames;
    std::vector<bool> m_inliers;
    std::vector<long double> entryStamps;
    std::vector<std::vector<IntraMatch>> m_image_intraMatches;
    opengv::bearingVectors_t m_br1;
    opengv::bearingVectors_t m_br2;
    std::vector<int> m_camCorrespondences1;
    std::vector<int> m_camCorrespondences2;
    int m_intraMatchesSize{};

    friend class Relocalization;

};




#endif