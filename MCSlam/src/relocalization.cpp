#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>
#include <mutex>
#include <map>
#include <chrono>
#include "MCSlam/relocalization.h"


//TODO: Shift the Json object outside so that we dont create duplicates for the Tracking module.

Relocalization::Relocalization(ORBVocabulary* voc, ORBVocabulary* map_vocab, ORBextractor* extract, const std::string& db_fileName,
const boost::json::value& json_File_value) : orb_vocabulary(voc), feat_extract(extract)
{
    // Initialize the loop closure parameters
    looper = new LoopCloser(map_vocab, extract);
    // Set up the parameters.
    looper->dloop_param.di_levels = 2;
    looper->dloop_param.dislocal = 1;
    looper->dloop_param.k = 1;
    looper->dloop_param.alpha = 0.05;
    looper->dloop_param.min_Fpoints = 18;
    looper->dloop_param.min_nss_factor = 0.005;
    minInlierRatio = 0.04;
    looper->dloop_param.use_nss = false;
    looper->setParams(looper->dloop_param);
    looper->type = LoopCloser::ABSOLUTE_POSE;
    result.status = DLoopDetector::NO_GROUPS;
    // Load the feature extractor

    // Add the database in the orb vocabulary.
    looper->loadDatabase(db_fileName);

    // Load the landmarks and the descriptors.
    jsonParser = std::make_unique<boost::json::value>(json_File_value);
//    std::ifstream json_file("/home/aryaman/catkin_ws/navability_maps/THUNDERHILL_rosbag_2023_17_08_14_25_40_e83/THUNDERHILL_rosbag_2023_17_08_14_25_40_e83_features.json");
//    navabilityJsonParser = boost::json::parse(json_file).as_object();
//    std:: ifstream json_pose_file("/home/aryaman/catkin_ws/navability_maps/THUNDERHILL_rosbag_2023_17_08_14_25_40_e83/THUNDERHILL_rosbag_2023_17_08_14_25_40_e83_poses.json");
//    navabilityJsonPoseParser = boost::json::parse(json_pose_file).as_object();

}

bool Relocalization::checkRelocalizationNavability(MultiCameraFrame *currentFrame){
    EntryId entryId = looper->orb_database->size();
    result.query = entryId;
    r_frame = currentFrame;

    DBoW2::BowVector currBow;
    DBoW2::FeatureVector currFeatVec;

    auto start_geom = std::chrono::high_resolution_clock::now();
    auto stop_geom = std::chrono::high_resolution_clock::now();

    if((int) entryId <= looper->dloop_param.dislocal)
    {
        VLOG(0) << "CLOSE_MATCH :  entry id - " << entryId << " dislocal - " << looper->dloop_param.dislocal << std::endl;
        result.status = DLoopDetector::CLOSE_MATCHES_ONLY;
    }
    else
    {
        int maxId = (int)entryId - looper->dloop_param.dislocal;

        DBoW2::QueryResults queryResults;
        auto start_T = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < r_frame->num_cams_; i++)
        {
            currBow.clear();
            currFeatVec.clear();
            VLOG(0) << "querrying image - " << i << std::endl;
            looper->createBoWandFeatVec(r_frame->image_descriptors[i], currBow, currFeatVec);
//            looper->orb_database->query(currBow, queryResults, looper->dloop_param.max_db_results, maxId);
            looper->orb_database->query(r_frame->image_descriptors[i], queryResults, looper->dloop_param.max_db_results, maxId);
            if(!queryResults.empty())
            {
                double ns_factor = 1.0;
                if(looper->dloop_param.use_nss)
                {
                    ns_factor = looper->orb_database->getVocabulary()->score(currBow, looper->last_bowvec);
                    VLOG(0) << "This is the ns_factor - " << ns_factor << std::endl;
                }
                if(ns_factor >= looper->dloop_param.min_nss_factor)
                {
                    // Remove the low scoring frames.
                    Mat scores;
                    for (auto &qr : queryResults)
                    {
                        scores.push_back(qr.Score);
                    }
                    VLOG(0) << "Scores of matching frames - " << scores.t() << std::endl;
                    looper->removeLowScores(queryResults, looper->dloop_param.alpha * ns_factor);
                    // get the best entry from query results and compute the temporal window.
                    if(!queryResults.empty())
                    {
                        // get the ID corresponding to best match from queryResults.

                        result.match = queryResults[0].Id;
                        std::vector<DorianLoopDetector::tIsland> islands;
                        looper->computeIslands(queryResults, islands);
                        if(!islands.empty())
                        {
                            const tIsland& island = *std::max_element(islands.begin(), islands.end());
                            // convert the best entry to island object
                            DorianLoopDetector::updateTemporalWindow(island, entryId);
                            result.match = island.best_entry;
                            VLOG(0) << "Best entry - " << island.best_entry << std::endl;
                            VLOG(0) << "Consistent entries in database - " << DorianLoopDetector::getConsistentEntries() << std::endl;
                            if(DorianLoopDetector::getConsistentEntries() >= looper->dloop_param.k)
                            {
                                auto stop_temporal = std::chrono::high_resolution_clock::now();
                                auto duration_temporal = std::chrono::duration_cast
                                        <std::chrono::microseconds >(stop_temporal - start_T);
                                bool detection;
                                if(looper->dloop_param.geom_check == DLoopDetector::GEOM_DI)
                                {
                                    // visualize the image corresponding to the best match with the keypoint matches
                                    VLOG(0) << "Island best score - " << island.best_score << std::endl;
                                    start_geom = std::chrono::high_resolution_clock::now();
                                    detection = geometricVerificationNavability(island.best_entry, result, r_frame->lfFeatVec, i);
                                    stop_geom = std::chrono::high_resolution_clock::now();
                                    VLOG(0) << "Performing geometric verification." << std::endl;
                                }
                                if(detection)
                                {
                                    auto duration_geom = std::chrono::duration_cast
                                            <std::chrono::microseconds>(stop_geom - start_geom);
//                                    VLOG(0) << "- - - Duration for Querying data - " << duration_query.count() << " mu_s" << std::endl;
                                    VLOG(0) << "- - - Duration for Temporal matching - " << duration_temporal.count() << " mu_s" << std::endl;
                                    VLOG(0) << "- - - Duration for geometric verification - " << duration_geom.count() << " mu_s" << std::endl;
                                    result.status = DLoopDetector::LOOP_DETECTED;
                                    break;
                                }
                                else
                                {
                                    VLOG(0) << "NO_GEOMETRICAL_CONSISTENCY" << std::endl;
                                    result.status = DLoopDetector::NO_GEOMETRICAL_CONSISTENCY;
                                }

                            }
                            else
                            {
                                VLOG(0) << "NO_TEMPORAL_CONSISTENCY"<<std::endl;
                                result.status = DLoopDetector::NO_TEMPORAL_CONSISTENCY;
                            }
                        }
                        else
                        {
                            VLOG(0) << "NO_GROUPS" << std::endl;
                            result.status = DLoopDetector::NO_GROUPS;
                        }
                    }

                    else
                    {
                        VLOG(0) << "LOW_SCORES" << std::endl;
                        result.status = DLoopDetector::LOW_SCORES;
                    }
                }
                else
                {
                    VLOG(0) << "LOW_NSS_FACTOR"<< std::endl;
                    result.status = DLoopDetector::LOW_SCORES;
                }
            }
            else
            {
                VLOG(0) << "NO_DB_RESULTS" << std::endl;
                result.status = DLoopDetector::NO_DB_RESULTS;
            }
        }
    }
    // Store the BoW vectors if we are going to use it in the next iterations.
    if(looper->dloop_param.use_nss && (int)entryId + 1 > looper->dloop_param.dislocal)
    {
        looper->last_bowvec = currBow;
    }
    return result.detection();
}

bool Relocalization::checkRelocalization(MultiCameraFrame *currentFrame)
{
    EntryId entryId = looper->orb_database->size();
    result.query = entryId;
    r_frame = currentFrame;

    DBoW2::BowVector currBow;
    DBoW2::FeatureVector currFeatVec;

    // Convert the BoW to the newFile format. IMPORTANT
    looper->createBoWandFeatVec(currentFrame, currBow, currFeatVec);

    // Time :
    auto start_geom = std::chrono::high_resolution_clock::now();
    auto stop_geom = std::chrono::high_resolution_clock::now();

    if((int) entryId <= looper->dloop_param.dislocal)
    {
        VLOG(0) << "CLOSE_MATCH :  entry id - " << entryId << " dislocal - " << looper->dloop_param.dislocal << std::endl;
        result.status = DLoopDetector::CLOSE_MATCHES_ONLY;
    }
    else
    {
        int maxId = (int)entryId - looper->dloop_param.dislocal;

        DBoW2::QueryResults queryResults;
        auto start_T = std::chrono::high_resolution_clock::now();
        // create bow for 1st image alone in the current frame

        looper->orb_database->query(currBow, queryResults, looper->dloop_param.max_db_results, maxId);
        auto stop_query = std::chrono::high_resolution_clock::now();
        auto duration_query = std::chrono::duration_cast<std::chrono::microseconds>(stop_query - start_T);
        if(!queryResults.empty())
        {
            double ns_factor = 1.0;
            if(looper->dloop_param.use_nss)
            {
                ns_factor = looper->orb_database->getVocabulary()->score(currBow, looper->last_bowvec);
                VLOG(0) << "This is the ns_factor - " << ns_factor << std::endl;
            }
            if(ns_factor >= looper->dloop_param.min_nss_factor)
            {
                // Remove the low scoring frames.
                Mat scores;
                for (auto &qr : queryResults)
                {
                    scores.push_back(qr.Score);
                }
                VLOG(0) << "Scores of matching frames - " << scores.t() << std::endl;
                looper->removeLowScores(queryResults, looper->dloop_param.alpha * ns_factor);
                // get the best entry from query results and compute the temporal window.
                if(!queryResults.empty())
                {
                    // get the ID corresponding to best match from queryResults.

                    result.match = queryResults[0].Id;
                    std::vector<DorianLoopDetector::tIsland> islands;
                    looper->computeIslands(queryResults, islands);
                    if(!islands.empty())
                    {
                        const tIsland& island = *std::max_element(islands.begin(), islands.end());
                      // convert the best entry to island object
                        DorianLoopDetector::updateTemporalWindow(island, entryId);
                        result.match = island.best_entry;
                        VLOG(0) << "Best entry - " << island.best_entry << std::endl;
                        // visualize the image corresponding to the best match.
                        double time = jsonParser->as_object()["entry_" + std::to_string(island.best_entry)].as_object()["time"].as_double();
                        VLOG(0) << "Consistent entries in database - " << DorianLoopDetector::getConsistentEntries() << std::endl;
                        if(DorianLoopDetector::getConsistentEntries() >= looper->dloop_param.k)
                        {
                            auto stop_temporal = std::chrono::high_resolution_clock::now();
                            auto duration_temporal = std::chrono::duration_cast
                                    <std::chrono::microseconds >(stop_temporal - start_T);
                            bool detection;
                            if(looper->dloop_param.geom_check == DLoopDetector::GEOM_DI)
                            {
                                // visualize the image corresponding to the best match with the keypoint matches
                                VLOG(0) << "Island best score - " << island.best_score << std::endl;
                                start_geom = std::chrono::high_resolution_clock::now();
                                detection = geometricVerification(island.best_entry, result, r_frame->lfFeatVec);
                                stop_geom = std::chrono::high_resolution_clock::now();
                                VLOG(0) << "Performing geometric verification." << std::endl;
                            }
                            if(detection)
                            {
                                auto duration_geom = std::chrono::duration_cast
                                        <std::chrono::microseconds>(stop_geom - start_geom);
                                VLOG(0) << "- - - Duration for Querying data - " << duration_query.count() << " mu_s" << std::endl;
                                VLOG(0) << "- - - Duration for Temporal matching - " << duration_temporal.count() << " mu_s" << std::endl;
                                VLOG(0) << "- - - Duration for geometric verification - " << duration_geom.count() << " mu_s" << std::endl;
                                result.status = DLoopDetector::LOOP_DETECTED;
                            }
                            else
                            {
                                VLOG(0) << "NO_GEOMETRICAL_CONSISTENCY" << std::endl;
                                result.status = DLoopDetector::NO_GEOMETRICAL_CONSISTENCY;
                            }

                        }
                        else
                        {
                            VLOG(0) << "NO_TEMPORAL_CONSISTENCY"<<std::endl;
                            result.status = DLoopDetector::NO_TEMPORAL_CONSISTENCY;
                        }
                    }
                    else
                    {
                        VLOG(0) << "NO_GROUPS" << std::endl;
                        result.status = DLoopDetector::NO_GROUPS;
                    }
                }
                else
                {
                    VLOG(0) << "LOW_SCORES" << std::endl;
                    result.status = DLoopDetector::LOW_SCORES;
                }
            }
            else
            {
                VLOG(0) << "LOW_NSS_FACTOR"<< std::endl;
                result.status = DLoopDetector::LOW_SCORES;
            }
        }
        else
        {
            VLOG(0) << "NO_DB_RESULTS" << std::endl;
            result.status = DLoopDetector::NO_DB_RESULTS;
        }
    }
    // Store the BoW vectors if we are going to use it in the next iterations.
    if(looper->dloop_param.use_nss && (int)entryId + 1 > looper->dloop_param.dislocal)
    {
        looper->last_bowvec = currBow;
    }
    return result.detection();

}

template <typename T>
void Relocalization::arrayToMat(const boost::json::array& jsonArray, std::vector<cv::Mat>& out_data)
{
    for (auto& i : jsonArray)
    {
        out_data.emplace_back(cv::Mat(boost::json::value_to<std::vector<T>>(i)).t());
    }
}

void Relocalization::featureMatchesBow(const DBoW2::FeatureVector &curr_featVec,
                                       const DBoW2::FeatureVector &best_featVec, vector<unsigned int> &indices_1,
                                       vector<unsigned int> &indices_2, std::vector<cv::Mat>& matched_desc)
{
    indices_1.clear();
    indices_2.clear();
    int itr_cng_match = 0;
    //
    DBoW2::FeatureVector::const_iterator curr_iter, best_iter;
    curr_iter = curr_featVec.begin();
    best_iter = best_featVec.begin();

    const auto curr_end = curr_featVec.end();
    const auto best_end = best_featVec.end();

    std::vector<cv::Mat> current_desc;

    for (auto &d : r_frame->intraMatches)
        current_desc.push_back(d.matchDesc);

    while(curr_iter != curr_end && best_iter != best_end)
    {
        if (curr_iter->first == best_iter->first)
        {
            std::vector<unsigned int> i_ind_temp, j_ind_temp;
            feat_extract->getMatches_distRatio(matched_desc, best_iter->second,
                                               current_desc, curr_iter->second, i_ind_temp, j_ind_temp,
                                               itr_cng_match);

            indices_1.insert(indices_1.end(), i_ind_temp.begin(), i_ind_temp.end());
            indices_2.insert(indices_2.end(), j_ind_temp.begin(), j_ind_temp.end());

            ++curr_iter;
            ++best_iter;
        }
        else if(best_iter->first < curr_iter->first)
        {
            best_iter = best_featVec.lower_bound(curr_iter->first);
        }
        else
        {
            curr_iter = curr_featVec.lower_bound(best_iter->first);
        }
    }
}

void Relocalization::processLandmarks(const DBoW2::EntryId &old_Id, std::map<int, std::vector<double>> &lid_to_point_map,
                                      std::vector<int>& lIds)
{
    boost::json::value landmarks = jsonParser->at("entry_" + std::to_string(old_Id)).as_object().at("l_ids");
    lIds = boost::json::value_to<std::vector<int>>(landmarks);

    boost::json::array map_points = jsonParser->at("entry_" + std::to_string(old_Id)).as_object().at("points").as_array();
    std::vector<std::vector<double>> points3D;
    for(auto& i : map_points)
    {
        points3D.emplace_back(boost::json::value_to<std::vector<double>>(i));
    }
    VLOG(0) << "Size of the landmarks - " << lIds.size() << std::endl;
    // Create the map
    int point_count = 0;
    for (int lId : lIds)
    {
        if(lId == -1)
        {
            continue;
        }
        else
        {
            lid_to_point_map[lId] = points3D[point_count];
            point_count += 1;
        }

    }
}

gtsam::Point2 Relocalization::project3DTo2D(const std::vector<double> &pt3d, Mat &Pose) {
    // project 3d point to 2d coordinates and return
    gtsam::Point3 pt3d_gtsam(pt3d[0], pt3d[1], pt3d[2]);

    // Get the camera intrinsics.
    Mat c_T_w = Pose.inv();
    Mat K = K_mat[0];

    // Project the 3D point to the image plane.
    Mat pt3d_mat = Mat(pt3d).reshape(1, 3);
    // make point homogenous
    pt3d_mat.push_back(Mat::ones(1, 1, CV_64F));
    Mat pt3d_cam = c_T_w * pt3d_mat;
    // reduce dimensions back to 3
    pt3d_cam = pt3d_cam.rowRange(0, 3);
    Mat pt2d = K * pt3d_cam;
    pt2d = pt2d / pt2d.at<double>(2, 0);
    return gtsam::Point2(pt2d.at<double>(0, 0), pt2d.at<double>(1, 0));
}

bool Relocalization::checkAbsolutePose(const DBoW2::EntryId &old_Id, std::vector<cv::DMatch> &matches,
                                       LoopCloser::Detection_Frame &result, std::map<int, std::vector<double>> &lid_to_point_map,
                                       vector<int> &lIds, string &timestamp, int cameraIndex, Mat &Pose)
{
    VLOG(0) << "Number of matches : " << matches.size() << std::endl;
    looper->m_br1.clear();
    looper->m_camCorrespondences1.clear();
    looper->m_inliers.clear();
    looper->m_inliers = std::vector<bool>(matches.size(), false);
    points_t points;

    // vectors of keypoints in query image to database and landmarks projected onto the result image from the database
    vector<gtsam::Point2> imgpts1, imgpts2;
    for (auto &m : matches)
    {
        std::vector<double> pt3d;
        int l_id = lIds[m.queryIdx];
        if(l_id == -1)
            continue;
        pt3d = lid_to_point_map[l_id];
        // project the 3d landmark to the camera frame of pose
        gtsam::Point2 ogPoint = project3DTo2D(pt3d, Pose);
        imgpts1.push_back(ogPoint);
        Eigen::Vector3d pt;
        for(int j = 0; j < 3; j++)
        {
            pt[j] = pt3d.at(j);
        }
        points.push_back(pt);
        IntraMatch intra_curr = r_frame->intraMatches[m.trainIdx];
        for(int i = 0; i < r_frame->num_cams_; i++)
        {
            if(intra_curr.matchIndex[i] != -1)
            {
                cv::Point2f uv_coord = r_frame->image_kps[i][intra_curr.matchIndex[i]].pt;
                gtsam::Point2 pt(uv_coord.x, uv_coord.y);
                imgpts2.push_back(pt);
                double norm_x = (uv_coord.x - K_mat[i].at<double>(0,2))/K_mat[i].at<double>(0,0);
                double norm_y = (uv_coord.y - K_mat[i].at<double>(1,2)) /K_mat[i].at<double>(1,1);
                Eigen::Vector3d pt_n;
                pt_n[0] = norm_x;
                pt_n[1] = norm_y;
                pt_n[2] = 1.0;
                looper->m_br1.emplace_back(pt_n/pt_n.norm());
                looper->m_camCorrespondences1.emplace_back(i);
                break;
            }
        }

    }

    assert(looper->m_camCorrespondences1.size() == points.size());
    // Non-Central Absolute adapter.
    opengv::absolute_pose::NoncentralAbsoluteAdapter adapter(looper->m_br1, looper->m_camCorrespondences1, points,
                                                             r_frame->translations, *r_frame->rotations_ptr);

    opengv::sac::Ransac<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;
    ransac.sac_model_ = std::make_shared<sac_problems::absolute_pose::AbsolutePoseSacProblem>(
            adapter,
            sac_problems::absolute_pose::AbsolutePoseSacProblem::GP3P);
    ransac.threshold_ = 2 * (1.0 - cos(atan(sqrt(2.0)*0.5/K_mat[0].at<double>(0,2))));
    // increase the ransac threshold to include more inliers
//    ransac.threshold_ =
    ransac.max_iterations_ = 10000;   // TODO: Add this parameter to the config file.
    ransac.computeModel();
    double inlierRatio = (double)ransac.inliers_.size()/(double)matches.size();
    VLOG(0) << "Inlier size - " << ransac.inliers_.size() << std::endl;
    VLOG(0) << "Inlier to Outlier ratio - " << inlierRatio << std::endl;
    VLOG(0) << " " << std::endl;
    // Get the absolute pose.x`
    opengv::transformation_t essential_matrix = ransac.model_coefficients_;
    cv::Mat pose;
    cv::eigen2cv(essential_matrix, pose);
    VLOG(0) << "Pose - \n" << pose << std::endl;
    if (!pose.empty() && inlierRatio >= minInlierRatio)
    {
        for(int inlier : ransac.inliers_)
        {
            looper->m_inliers[inlier] = true;
        }
        VLOG(0) << "Relocalization candidate verified!" << std::endl;
        VLOG(0) << "Entry ID - " << old_Id << std::endl;
        VLOG(0) << "Pose - \n" << pose << std::endl;
        VLOG(0) << std::setprecision(15) << "Timestamp of the current KF - " << r_frame->timeStamp << std::endl;
        result.relative_pose = convertPose3_CV2GTSAM(pose);

        return true;
    }
    return false;
}

bool Relocalization::checkAbsolutePose(const DBoW2::EntryId &old_Id, std::vector<cv::DMatch> &matches,
                                       LoopCloser::Detection_Frame &result)
{
    VLOG(0) << "Number of matches : " << matches.size() << std::endl;
    looper->m_br1.clear();
    looper->m_camCorrespondences1.clear();
    looper->m_inliers.clear();
    looper->m_inliers = std::vector<bool>(matches.size(), false);
    points_t points;

    std::map<int, std::vector<double>> lid_to_point_map;
    std::vector<int> lIds;
    processLandmarks(old_Id, lid_to_point_map, lIds);

    // vectors to visualize
    vector<gtsam::Point2> imgpts1, imgpts2;
    // project
    for (auto &m : matches)
    {
        std::vector<double> pt3d;
        int l_id = lIds[m.queryIdx];
        if(l_id == -1)
            continue;
        pt3d = lid_to_point_map[l_id];
        // project the 3d landmark to the camera frame of pose
        gtsam::Point2 ogPoint = project3DTo2D(pt3d, old_Id);
        imgpts1.push_back(ogPoint);
        Eigen::Vector3d pt;
        for(int j = 0; j < 3; j++)
        {
            pt[j] = pt3d.at(j);
        }
        points.push_back(pt);
        IntraMatch intra_curr = r_frame->intraMatches[m.trainIdx];
        for(int i = 0; i < r_frame->num_cams_; i++)
        {
            if(intra_curr.matchIndex[i] != -1)
            {
                cv::Point2f uv_coord = r_frame->image_kps[i][intra_curr.matchIndex[i]].pt;
                gtsam::Point2 pt(uv_coord.x, uv_coord.y);
                imgpts2.push_back(pt);
                double norm_x = (uv_coord.x - K_mat[i].at<double>(0,2))/K_mat[i].at<double>(0,0);
                double norm_y = (uv_coord.y - K_mat[i].at<double>(1,2)) /K_mat[i].at<double>(1,1);
                Eigen::Vector3d pt_n;
                pt_n[0] = norm_x;
                pt_n[1] = norm_y;
                pt_n[2] = 1.0;
                looper->m_br1.emplace_back(pt_n/pt_n.norm());
                looper->m_camCorrespondences1.emplace_back(i);
                break;
            }
        }

    }

    assert(looper->m_camCorrespondences1.size() == points.size());
    // Non-Central Absolute adapter.
    opengv::absolute_pose::NoncentralAbsoluteAdapter adapter(looper->m_br1, looper->m_camCorrespondences1, points,
                                                             r_frame->translations, *r_frame->rotations_ptr);

    opengv::sac::Ransac<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;
    ransac.sac_model_ = std::make_shared<sac_problems::absolute_pose::AbsolutePoseSacProblem>(
            adapter,
            sac_problems::absolute_pose::AbsolutePoseSacProblem::GP3P);
    ransac.threshold_ = 3 * (1.0 - cos(atan(sqrt(2.0)*0.5/K_mat[0].at<double>(0,2))));
    // increase the ransac threshold to include more inliers
//    ransac.threshold_ =
    ransac.max_iterations_ = 500;   // TODO: Add this parameter to the config file.

    ransac.computeModel();
    double inlierRatio = (double)ransac.inliers_.size()/(double)matches.size();
    VLOG(0) << "Inlier size - " << ransac.inliers_.size() << std::endl;
    VLOG(0) << "Inlier to Outlier ratio - " << inlierRatio << std::endl;
    VLOG(0) << " " << std::endl;
    // Get the absolute pose.x`
    opengv::transformation_t essential_matrix = ransac.model_coefficients_;
    cv::Mat pose;
    cv::eigen2cv(essential_matrix, pose);
    /// Visualization -
//    cv::Mat groundTruth;
//    parseFileAndSaveGroundTruth("/home/aryaman/catkin_ws/graph_logs_KRI.txt", (int)old_Id, groundTruth);
//    VLOG(0) << "Ground Truth pose - \n " << groundTruth << std::endl;
//    VLOG(0) << "Relative pose error - \n" << computeRelativePoseError(pose, groundTruth);
//    VLOG(0) << "----" << std::endl;
//    visualizeMatches(const_cast<EntryId &>(old_Id), matches);
    // get the image 0 corresponding to the best matching keyframe id
//    double timestamp = jsonParser->as_object()["entry_" + std::to_string(old_Id)].as_object()["time"].as_double();
//    std::string time_stamp = LoopCloser::convertDoubleToString(timestamp);
//    std::string image_dir = "/home/aryaman/catkin_ws/ISEC_Lab1/rosbag_2023_17_08_14_25/cam0/";
//    std::string image_dir = "/home/aryaman/catkin_ws/ISEC_Lab1/5_1_zed_challenging_test/cam0/";
//    std::string file_path = image_dir + looper->findMatchingFile(image_dir,time_stamp);
//    VLOG(0) << "File path - " << file_path << std::endl;
//    cv::Mat img = cv::imread(file_path, 0);
//    // visualize the matches
//    newVisualizeMatches(imgpts2, imgpts1, r_frame->imgs[0], img);
    VLOG(0) << "Pose - \n" << pose << std::endl;
    if (!pose.empty() && inlierRatio >= minInlierRatio)
    {
        for(int inlier : ransac.inliers_)
        {
            looper->m_inliers[inlier] = true;
        }
        VLOG(0) << "Relocalization candidate verified!" << std::endl;
        VLOG(0) << "Entry ID - " << old_Id << std::endl;
        VLOG(0) << "Pose - \n" << pose << std::endl;
        VLOG(0) << std::setprecision(15) << "Timestamp of the current KF - " << r_frame->timeStamp << std::endl;
        result.relative_pose = convertPose3_CV2GTSAM(pose);

        return true;
    }
    return false;

}

void Relocalization::newVisualizeMatches(vector<gtsam::Point2> currentImagePoints, vector<gtsam::Point2> oldImagePoints, cv::Mat currentImage, cv::Mat oldImage)
{
    // Draw the matches
    // make current and old image colored
    cv::cvtColor(currentImage, currentImage, cv::COLOR_GRAY2BGR);
    cv::cvtColor(oldImage, oldImage, cv::COLOR_GRAY2BGR);
    cv::Mat img_matches = Mat(currentImage.rows, currentImage.cols + oldImage.cols, currentImage.type());
    // draw the keypoints on each corresponding image
    for (int i = 0; i < currentImagePoints.size(); i++)
    {
        cv::circle(currentImage, cv::Point(currentImagePoints[i].x(), currentImagePoints[i].y()), 3, cv::Scalar(0, 255, 0), 2);
    }
    for (int i = 0; i < oldImagePoints.size(); i++) {
        cv::circle(oldImage, cv::Point(oldImagePoints[i].x(), oldImagePoints[i].y()), 3, cv::Scalar(0, 0, 255), 2);
    }
//    // concatenate the 2 images and create new image to show
    cv::hconcat(currentImage, oldImage, img_matches);
//    // draw lines between the corresponding matches on the images
//    for (int i = 0; i < currentImagePoints.size(); i++)
//    {
//        cv::line(img_matches, cv::Point(currentImagePoints[i].x(), currentImagePoints[i].y()),
//                 cv::Point(oldImagePoints[i].x() + currentImage.cols, oldImagePoints[i].y()), cv::Scalar(255, 0, 0), 1);
//    }
    // show the image
    cv::imshow("Matches", img_matches);
    cv::waitKey(0);
}

gtsam::Point2 Relocalization::project3DTo2D(const std::vector<double>& pt3d, const DBoW2::EntryId &old_Id)
{
    // Get the camera pose.
    boost::json::value poses = jsonParser->as_object()["entry_" + std::to_string(old_Id)].as_object()["pose"];
    vector<vector<double>> poseVec;
    poseVec = boost::json::value_to<vector<vector<double>>>(poses);

    // cv Mat pose
    Mat pose = Mat(poseVec.size(), poseVec[0].size(), CV_64F);
    for(int i = 0; i < poseVec.size(); i++)
    {
        for(int j = 0; j < poseVec[i].size(); j++)
        {
            pose.at<double>(i, j) = poseVec[i][j];
        }
    }

    // Get the camera intrinsics.
    Mat c_T_w = pose.inv();
    Mat K = K_mat[0];

    // Project the 3D point to the image plane.
    Mat pt3d_mat = Mat(pt3d).reshape(1, 3);
    // make point homogenous
    pt3d_mat.push_back(Mat::ones(1, 1, CV_64F));
    Mat pt3d_cam = c_T_w * pt3d_mat;
    // reduce dimensions back to 3
    pt3d_cam = pt3d_cam.rowRange(0, 3);
    Mat pt2d = K * pt3d_cam;
    pt2d = pt2d / pt2d.at<double>(2, 0);
    return gtsam::Point2(pt2d.at<double>(0, 0), pt2d.at<double>(1, 0));
}

/* Deprecated function Will be removed in future commit
 * introduced to debug BoW matching */
void Relocalization::bruteForceMatching(Mat &desc1, Mat &desc2, std::vector<cv::DMatch> &matches)
{
    cv::BFMatcher matcher(cv::NORM_HAMMING, true);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    VLOG(0) << "desc1 size - " << desc1.size() << std::endl;
    VLOG(0) << "desc2 size - " << desc2.size() << std::endl;
    matcher.knnMatch(desc1, desc2, knn_matches, 2);
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < 0.8 * knn_matches[i][1].distance)
        {
            matches.push_back(knn_matches[i][0]);
        }
    }
}

bool Relocalization::geometricVerificationNavability(const DBoW2::EntryId &old_entry, LoopCloser::Detection_Frame &result,
                                           DBoW2::FeatureVector& currFeatVec, int cameraIndex)
{
    std::vector<cv::DMatch> matches;
    std::vector<cv::DMatch> mono_matches;
    DBoW2::BowVector matchedBoW;
    DBoW2::FeatureVector  matchedFeatVec;
    vector<unsigned int> i_old, i_cur;
    std::vector<cv::Mat> vecDesc;
    map<int, vector<double>> landmarkToPointMap;
    vector<int> lIds;
    string timestamp;
    Mat Pose;
    // Convert to the vector descriptors.
    getLandmarkDescriptors(old_entry, vecDesc, landmarkToPointMap, lIds, timestamp, Pose);

    orb_vocabulary->transform(vecDesc, matchedBoW, matchedFeatVec, 5);

    featureMatchesBow(currFeatVec, matchedFeatVec, i_old, i_cur, vecDesc);

    VLOG(0) << "timestamp of the matched image - " << timestamp << std::endl;

    for(int i = 0 ; i < i_old.size(); i++)
    {
        cv::DMatch m;
        m.queryIdx = static_cast<int>(i_old[i]);
        m.trainIdx = static_cast<int>(i_cur[i]);
        matches.push_back(m);
    }

    if (vecDesc.size() > 0)
    {
        VLOG(0) << "Matches size - " << matches.size() << std::endl;
        // Apply the GP3P for pose estimation.
        /// Visualization -
        std::string image_dir = "/home/aryaman/catkin_ws/ISEC_Lab1/rosbag_2023_17_08_14_25/cam" + std::to_string(cameraIndex) + "/";
        std::string file_path = image_dir + looper->findMatchingFile(image_dir, timestamp);
        VLOG(0) << "File path - " << file_path << std::endl;
        cv::Mat img = cv::imread(file_path, 0);
        // visualize the matches
        // put all keypoints found in current image in imgpts2 and all landmarks projected to 2d points in imgpts1
        vector<gtsam::Point2> imgpts1, imgpts2;
        VLOG(0) << "landmark to point map size - " << landmarkToPointMap.size() << std::endl;
        for (auto &kp : r_frame->image_kps[cameraIndex])
        {
            imgpts2.push_back(gtsam::Point2(kp.pt.x, kp.pt.y));
        }
        for (auto &l : landmarkToPointMap)
        {
            imgpts1.push_back(project3DTo2D(l.second, Pose));
        }

        VLOG(0) << "sizes of point vectors - " << imgpts1.size() << " - " << imgpts2.size() << std::endl;

        newVisualizeMatches(imgpts2, imgpts1, r_frame->imgs[cameraIndex], img);
        return checkAbsolutePose(old_entry, matches, result, landmarkToPointMap, lIds, timestamp, cameraIndex, Pose);
    }
    else
    {
        VLOG(0) << "Matches size - " << matches.size() << std::endl;
        result.status = DLoopDetector::NO_DB_RESULTS;
        return false;
    }
}

void Relocalization::getLandmarkDescriptors(const DBoW2::EntryId &old_entry, std::vector<cv::Mat> &vecDesc, map<int,
                                            vector<double>> &landmarkToPointMap, vector<int> &lIds,
                                            std::string &timestamp, Mat &Pose)
{
    // parse text file to get camera pose corresponding to the entry id.
    std::ifstream file("/home/aryaman/catkin_ws/navability_maps/THUNDERHILL_rosbag_2023_17_08_14_25_40_e83/THUNDERHILL_rosbag_2023_17_08_14_25_40_e83_camera_pose_idx.txt");
    std::string line;
    while(std::getline(file, line)) {
        std::string delimiter = ",";
        size_t pos = 0;
        std::string token;
        while ((pos = line.find(delimiter)) != std::string::npos) {
            token = line.substr(0, pos);
            line.erase(0, pos + delimiter.length());
        }
        if (token != "dbidx"){
            if (std::stoi(token) == old_entry){
                VLOG(0) << "Entry id found in the text file - " << old_entry << std::endl;
                break;
            }
        }
    }
    std::string camera_pose = line;

    VLOG(0) << "Camera pose read from text file - " << camera_pose << std::endl;


    // create a counter to track landmark id
    int l_id = 0;
    // parse json file and get all landmarks which have the camera_pose string in the adj_cam field
//    std::ifstream json_file("/home/aryaman/catkin_ws/navability_maps/THUNDERHILL_rosbag_2023_17_08_14_35_03_v77_features.json");
//    boost::json::object navabilityJsonParser = boost::json::parse(json_file).as_object();
    map<int, Mat> landmarkToDescMap;
    for (auto& i : navabilityJsonParser) {
        // check if title of the entry contains the camera pose
        string titleCameraPoseString = "_" + camera_pose + "_";
        if (i.key().find(titleCameraPoseString) == std::string::npos) {
            boost::json::array adj_cam = i.value().at("adj_cams").as_array();
            // check if current cam pose is present in array of strings adj_cam
            for (auto &j: adj_cam) {
                string cam_pose = j.as_string().c_str();
                if (cam_pose == camera_pose) {
                    // create a fake landmark id
                    l_id += 1;
                    boost::json::array point = i.value().at("pos").as_array();
                    vector<double> point3D;
                    for (auto &k: point) {
                        point3D.push_back(k.as_double());
                    }
                    landmarkToPointMap[l_id] = point3D;
                    boost::json::value desc = i.value().at("descriptor");
                    vecDesc.emplace_back(Mat(boost::json::value_to<std::vector<uchar>>(desc)).t());
//                    VLOG(0) << "descriptor of landmark - " << Mat(point3D) << " - " << Mat(boost::json::value_to<std::vector<uchar>>(desc));
                    break;
                }
            }
        }
        else {
            l_id += 1;
            boost::json::array point = i.value().at("pos").as_array();
            vector<double> point3D;
            for (auto &k: point) {
                point3D.push_back(k.as_double());
            }
            landmarkToPointMap[l_id] = point3D;
            boost::json::value desc = i.value().at("descriptor");
            vecDesc.emplace_back(Mat(boost::json::value_to<std::vector<uchar>>(desc)));
        }
    }
    for (auto& i : landmarkToPointMap) {
        lIds.push_back(i.first);
    }
    if (vecDesc.empty()) {
        VLOG(0) << "No landmarks found for the camera pose - " << camera_pose << std::endl;
    }
    else {
        VLOG(0) << "Size of the landmark descriptors - " << vecDesc.size() << std::endl;
    }
    for (auto &j : navabilityJsonPoseParser) {
        if (j.key() == camera_pose) {
            VLOG(0) << "Camera pose found in the json file - " << camera_pose << std::endl;
            // get the timestamp
            boost::json::string time_stamp = j.value().at("timestamp").as_string();
            // convert date time format to posix time format
            std::string time_str = time_stamp.c_str();
            VLOG(0) << "Time stamp - " << time_str << std::endl;
            // Parse the input time string
            std::tm timeStruct = {};
            std::istringstream timeStream(time_str);
            timeStream >> std::get_time(&timeStruct, "%Y-%m-%dT%H:%M:%S");


            // Adjust the timezone offset
            timeStruct.tm_gmtoff = 18000;
            timeStruct.tm_hour -= timeStruct.tm_gmtoff / 3600;
            timeStruct.tm_min -= (timeStruct.tm_gmtoff % 3600) / 60;

            std::chrono::system_clock::time_point tp = std::chrono::system_clock::from_time_t(std::mktime(&timeStruct));
            std::chrono::duration<double> dur = tp.time_since_epoch();
            double epochTime = dur.count();
            std::string timestamp = std::to_string(epochTime);
            VLOG(0) << "Timestamp - " << timestamp << std::endl;
            // get position and quartenion from json file
            boost::json::array pos = j.value().at("pos").as_array();
            boost::json::array quat = j.value().at("quat").as_array();
            // convert to vector
            vector<double> position;
            vector<double> quaternion;
            for (auto &k: pos) {
                position.push_back(k.as_double());
            }
            for (auto &k: quat) {
                quaternion.push_back(k.as_double());
            }
            // convert to cv::Mat
            Mat positionMat = Mat(position).reshape(1, 3);
            Mat quaternionMat = Mat(quaternion).reshape(1, 4);
            // convert quaternion to rotation matrix
            Mat rotationMat;
            rotationMat = cv::Mat::eye(3, 3, CV_64F);
            // convert quaternion to rotation matrix
            Eigen::Quaterniond q(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
            Eigen::Matrix3d rotMat = q.toRotationMatrix();
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    rotationMat.at<double>(i, j) = rotMat(i, j);
                }
            }
            // create a 4x4 transformation matrix
            Mat transformationMat = Mat::eye(4, 4, CV_64F);

            // set the rotation and translation part of the transformation matrix
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    transformationMat.at<double>(i, j) = rotationMat.at<double>(i, j);
                }
                transformationMat.at<double>(i, 3) = positionMat.at<double>(i, 0);
            }
            // set the last row of the transformation matrix
            transformationMat.at<double>(3, 0) = 0;
            transformationMat.at<double>(3, 1) = 0;
            transformationMat.at<double>(3, 2) = 0;
            transformationMat.at<double>(3, 3) = 1;
            VLOG(0) << "Transformation matrix - " << transformationMat << std::endl;
            Pose = transformationMat;
            break;
        }
    }
}


bool Relocalization::geometricVerification(const DBoW2::EntryId &old_entry, LoopCloser::Detection_Frame &result,
                                           DBoW2::FeatureVector& currFeatVec)
{
    std::vector<cv::DMatch> matches;
    std::vector<cv::DMatch> mono_matches;
    DBoW2::BowVector matchedBoW;
    DBoW2::FeatureVector  matchedFeatVec;
    vector<unsigned int> i_old, i_cur;
    // Get the old feature vector.
    // Get the descriptors from the JSON file:
    boost::json::array descriptors = jsonParser->as_object()["entry_" + std::to_string(old_entry)].as_object()["descriptor"].as_array();
    std::vector<cv::Mat> vecDesc;
    // Convert to the vector descriptors.
    arrayToMat<uchar>(descriptors, vecDesc);
    orb_vocabulary->transform(vecDesc, matchedBoW, matchedFeatVec, 4);

    featureMatchesBow(currFeatVec, matchedFeatVec, i_old, i_cur, vecDesc);

    for(int i = 0 ; i < i_old.size(); i++)
    {
        cv::DMatch m;
        m.queryIdx = static_cast<int>(i_old[i]);
        m.trainIdx = static_cast<int>(i_cur[i]);
        matches.push_back(m);
        /// Take all the matches.
//        if (i_old[i] < r_frame->intramatch_size)
//            matches.push_back(m);
//        else
//            mono_matches.push_back(m);
    }

    // Apply the GP3P for pose estimation.
    return checkAbsolutePose(old_entry, matches, result);
}

int Relocalization::getORBDatabaseSize()
{
    return looper->getORBDatabaseSize();
}

cv::Mat Relocalization::parseFileAndSaveGroundTruth(const std::string& filename, int Id, cv::Mat& gt_pose)
{
    std::ifstream file(filename);
    std::string line;
    int count = 0;

    while(std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string x;
        ss >> x;
        if (x == "x")
        {
            if (count == Id)
            {
                std::string id;
                double timestamp;
                std::vector<double> poseVector;

                ss >> id >> timestamp;

                for (int i = 0; i < 16; i++)
                {
                    double poseElement;
                    ss >> poseElement;
                    poseVector.push_back(poseElement);
                }

                // Convert vector to 4x4 cv::Mat
                gt_pose = cv::Mat(poseVector).reshape(1, 4);
                file.close();
                VLOG(0) << std::setprecision(15) << "Time stamp of the KF in the database - " << timestamp << std::endl;
                return gt_pose;

            }
            else
            {
                count += 1;
            }
        }
    }
}

cv::Mat Relocalization::computeRelativePoseError(const cv::Mat& pose, const cv::Mat& groundTruthPose)
{
    // Ensure the poses are 4x4 matrices
    assert(pose.rows == 4 && pose.cols == 4);
    assert(groundTruthPose.rows == 4 && groundTruthPose.cols == 4);

    // Get rotation and translation parts of the pose
    cv::Mat poseRot = pose(cv::Rect(0, 0, 3, 3));
    cv::Mat poseTrans = pose(cv::Rect(3, 0, 1, 3));

    // Compute inverse of the pose
    cv::Mat poseInv = cv::Mat::eye(4, 4, CV_64F);
    poseRot.copyTo(poseInv(cv::Rect(0, 0, 3, 3)));
    poseInv(cv::Rect(0, 0, 3, 3)) = poseRot.t();
    poseInv(cv::Rect(3, 0, 1, 3)) = -poseRot.t() * poseTrans;

    // Compute the relative pose error
    cv::Mat relativePoseError = poseInv * groundTruthPose;

    gtsam::Pose3 rel_pose = convertPose3_CV2GTSAM(relativePoseError);

    VLOG(0) << "Norm of the translation - \n" << rel_pose.translation().norm() << std::endl;


    return relativePoseError;
}


void Relocalization::visualizeMatches(DBoW2::EntryId &matched_Id, std::vector<cv::DMatch> &matches)
{
    std::string image_dir = "/home/aryaman/catkin_ws/ISEC_Lab1/KRI/cam";
    // Put the timestamp over the images.
    cv::Point text_position(10, 30);
    int font_size = 1;
    cv::Scalar font_color(0, 255, 0);
    int font_weight = 2;

    std::string current_time = LoopCloser::convertDoubleToString(r_frame->timeStamp);
    current_time = current_time.substr(0, current_time.size() - 2);
    double matched_timestamp = jsonParser->as_object()["entry_" + std::to_string(matched_Id)].as_object()["time"].as_double();
    std::string best_match_time = LoopCloser::convertDoubleToString(matched_timestamp);
    best_match_time = best_match_time.substr(0, best_match_time.size() - 2);

    // Loop through the number of cameras and get the images.
    std::vector<cv::Mat> current_images = {};
    std::vector<cv::Mat> best_match_images = {};
    for(int i = 0 ; i < r_frame->num_cams_; i++) {
        std::string file_path = image_dir + std::to_string(i) + "/";
        // CURRENT
        std::string current_frame_dir = file_path + looper->findMatchingFile(file_path, current_time); //+ "388.png";
        current_images.push_back(cv::imread(current_frame_dir));

        // BEST MATCH
        std::string best_match_frame_dir = file_path + looper->findMatchingFile(file_path, best_match_time); //+ "388.png";
        best_match_images.push_back(cv::imread(best_match_frame_dir));
        if (i == 0)  // Put the database id on the first image only.
        {
            cv::putText(current_images.at(0), "current_image", text_position, cv::FONT_HERSHEY_SIMPLEX,
                        font_size, font_color, font_weight);
            cv::putText(best_match_images.at(0), std::to_string(matched_Id), text_position, cv::FONT_HERSHEY_SIMPLEX,
                        font_size, font_color, font_weight);
        }
    }

    // Safety:
    if(current_images.empty() || best_match_images.empty())
    {
        VLOG(0) << "Error: No Images were read!" << std::endl;
        return;
    }
    // This should work.
    drawMatches(current_images, best_match_images, matches, matched_Id);

}

void Relocalization::drawMatches(std::vector<cv::Mat> current_images, const std::vector<cv::Mat>& best_match_images,
                                 std::vector<cv::DMatch> &matches, DBoW2::EntryId &best_match)
{
    int pp = 0;
    for(auto& m : matches)
    {
//        if(looper->m_inliers[pp])
        if (true)
        {
            IntraMatch im1 = r_frame->intraMatches[m.trainIdx];

            cv::Point2f p1;
            //For the second image:
            cv::Point2f p2;

            int camIndCur = 0;
            cv::Scalar cc;
            if(im1.mono ) // For both the cases.
            {
                cc = cv::Scalar(0,255,0);
            }
            else
            {
                cc = cv::Scalar(0,0,255);
            }
            for(int featInd: im1.matchIndex)
            {
                if(featInd != -1)
                {
                    p1 = r_frame->image_kps_undist[camIndCur][featInd].pt;
                    cv::circle(current_images[camIndCur], p1, 4, cc, 2);
                }
                camIndCur++;
            }
//            // For the line between the matches in the current image and feature points that it matched against in the best match image.
//            IntraMatch im2 = looper->getIntraMatches(best_match)[m.queryIdx];
//            camIndCur = 0;
//            for(int featInd: im2.matchIndex)
//            {
//                if(featInd != -1)
//                {
//                    p2 = r_frame->image_kps_undist[camIndCur][featInd].pt;
//                    cv::circle(best_match_images[camIndCur], p2, 4, cc, 2);
//                }
//                camIndCur++;
//            }
//            cv::line(current_images[0], p1, p2, cc, 2);
        }
        pp++;
    }
    cv::Mat output_curr_image;
    cv::hconcat(current_images, output_curr_image);
    cv::Mat output_best_match;
    cv::hconcat(best_match_images, output_best_match);
    cv::Mat output;
    cv::vconcat(output_curr_image, output_best_match, output);
    assert(output_best_match.empty() == 0);
    cv::resize(output, output, cv::Size(output.cols * 0.6, output.rows * 0.6), 0 , 0, cv::INTER_LINEAR);
    cv::namedWindow("Matches", cv::WINDOW_AUTOSIZE);
    cv::imshow("Matches", output);
    cv::waitKey(0);

//    if(!looper->display_thread_running)
//    {
//        looper->display_thread_running = true;
//        std::thread display_thread([this, current_images, best_match_images]() {
//            this->looper->displayImages(current_images, best_match_images);
//        });
//        display_thread.detach();
//    }
}

Relocalization::~Relocalization()
{
    delete looper;
}


