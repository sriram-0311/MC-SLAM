
#include "MCSlam/LoopCloser.h"
#include "gtest/gtest.h"
// TODO: Check for inliers based on % rather than a values.
// TODO: Decouple DloopDetector.
// TODO: Add choice between the geometrical verification methods, in config file.
// TODO: Future upgrades: Saving to database and retrieving it.
//  db.save("small_db.yml.gz");
//  OrbDatabase db2("small_db.yml.gz");
// TODO: Make changes to include smoother vocab fie creation and Database creation using those files.


LoopCloser::LoopCloser(ORBVocabulary *voc, ORBextractor* extract):orb_vocab(voc), feat_extract(extract), DorianLoopDetector(dloop_param)
{
    VLOG(0) << "Loading Vocab to database, " << *orb_vocab << std::endl;
    orb_database = new ORBDatabase(*orb_vocab, true, 2);
    entryStamps.clear();
    m_inliers.clear();
    image_dir = "/home/aryaman/catkin_ws/ISEC_Lab1/image_data_1/cam";   // TODO: Take from the config file.

    //TODO: Call the setParams only if using the default DloopDetector functionalities.
}


void LoopCloser::setParams(DorianLoopDetector::Parameters &params)
{
    this->m_params = params;
    d_fsolver.setImageSize(params.image_rows, params.image_cols);
}

void LoopCloser::createBoWandFeatVec(MultiCameraFrame *kframe, DBoW2::BowVector &kfBoW,
                                     DBoW2::FeatureVector &kfFeatVec)
{
    std::vector<cv::Mat> vecDesc;
    std::vector<std::vector<cv::Mat>>::iterator ptr;
    for(int i = 0; i < kframe->image_descriptors.size(); i++)
    {
        for(int j = 0; j < kframe->image_descriptors.size(); j++)
        {
            vecDesc.push_back(kframe->image_descriptors[i][j]);
        }
    }
    // Get the BoW and FeatVec according to the smaller database.
    orb_vocab->transform(vecDesc, kfBoW, kfFeatVec, 4);
}

void LoopCloser::createBoWandFeatVec(vector<Mat> imageDescs, DBoW2::BowVector &kfBoW,
                                     DBoW2::FeatureVector &kfFeatVec)
{
//    std::vector<cv::Mat> vecDesc;
//    for(int i = 0; i < imageDescs.size(); i++)
//    {
//        vecDesc.push_back(imageDescs[i]);
//    }
    // Get the BoW and FeatVec according to the smaller database.
    orb_vocab->transform(imageDescs, kfBoW, kfFeatVec, 5);
}

bool LoopCloser::callerDetectLoop(MultiCameraFrame *frame, GlobalMap* map, std::vector<MultiCameraFrame*>& keyframes,
                                  LoopCloser::Detection_Frame &match)
{
    EntryId entryId = orb_database->size();
    match.query = entryId;



    key_frames.push_back(frame);
    m_intraMatchesSize = frame->intramatch_size;
    m_frame= frame;
    m_map = map;
    curr_BoW = m_frame->lfBoW;
    curr_FeatVec = m_frame->lfFeatVec;

    // Add the current frame boW, featVec and the timestamps to the database, take it out of the if case:
    if(key_frames.size() -1 == orb_database->size())
    {
//        createBoWandFeatVec(m_frame, curr_BoW, curr_FeatVec);
        orb_database->add(curr_BoW, curr_FeatVec);
        entryStamps.emplace_back(m_frame->timeStamp);
        m_image_intraMatches.push_back(m_frame->intraMatches);
        VLOG(0) << "Database size - " << orb_database->size() << std::endl;

    }
    else
    {
        VLOG(0) << "Adding all the initialized frames . . ." << std::endl;
        // Add all the keyframes and timestamps to the database.
        for(auto &k : key_frames)
        {
//            createBoWandFeatVec(k, curr_BoW, curr_FeatVec);
            orb_database->add(k->lfBoW, k->lfFeatVec);
            entryStamps.emplace_back(k->timeStamp);
            m_image_intraMatches.push_back(k->intraMatches);
        }
        // Update the entryID as well.
        entryId = orb_database->size() - 1;
        match.query = entryId;
    }
    // Safety: Check if the database and the key_frames are on the same page.
    assert(key_frames.size() == orb_database->size());

    if((int)entryId <= dloop_param.dislocal)
    {
        VLOG(0) << "CLOSE_MATCHES_ONLY : entry_id - " << entryId << " dislocal - " << dloop_param.dislocal << std::endl;
        match.status = DLoopDetector::CLOSE_MATCHES_ONLY;
    }
    else
    {
        int maxId = (int)entryId - dloop_param.dislocal;

        DBoW2::QueryResults qret;
        orb_database->query(curr_BoW, qret, dloop_param.max_db_results, maxId);

        if(!qret.empty())
        {
            double ns_factor = 1.0;
            if(dloop_param.use_nss)
            {
                ns_factor = orb_database->getVocabulary()->score(curr_BoW, last_bowvec);
                VLOG(0) << "This is the ns_factor - " << ns_factor << std::endl;
            }
            if(ns_factor >= dloop_param.min_nss_factor)
            {
                // Remove the low scoring frames.
                DorianLoopDetector::removeLowScores(qret, dloop_param.alpha * ns_factor);
                if(!qret.empty())
                {
                    match.match = qret[0].Id;
                    std::vector<DorianLoopDetector::tIsland> islands;
                    DorianLoopDetector::computeIslands(qret, islands);
//                    std::cout << "Number of Islands - " << islands.size() << std::endl;

                    if(!islands.empty())
                    {
                        const tIsland& island = *std::max_element(islands.begin(), islands.end());
                        DorianLoopDetector::updateTemporalWindow(island, entryId);
                        match.match = island.best_entry;
                        if(DorianLoopDetector::getConsistentEntries() > dloop_param.k)
                        {
                            bool detection;
                            if(dloop_param.geom_check == DLoopDetector::GEOM_DI)
                            {
                                VLOG(0) << "Island best score - " << island.best_score << std::endl;
                                detection = geometricVerification(island.best_entry, match);
                            }
                            if(detection)
                            {
                                match.status = DLoopDetector::LOOP_DETECTED;
                            }
                            else
                            {
                                VLOG(0) << "NO_GEOMETRICAL_CONSISTENCY" << std::endl;
                                match.status = DLoopDetector::NO_GEOMETRICAL_CONSISTENCY;
                            }

                        }
                        else
                        {
                            VLOG(0) << "NO_TEMPORAL_CONSISTENCY"<<std::endl;
                            match.status = DLoopDetector::NO_TEMPORAL_CONSISTENCY;
                        }
                    }
                    else
                    {
                        VLOG(0) << "NO_GROUPS" << std::endl;
                        match.status = DLoopDetector::NO_GROUPS;
                    }
                }
                else
                {
                    VLOG(0) << "LOW_SCORES" << std::endl;
                    match.status = DLoopDetector::LOW_SCORES;
                }
            }
            else
            {
                VLOG(0) << "LOW_NSS_FACTOR"<< std::endl;
                match.status = DLoopDetector::LOW_SCORES;
            }
        }
        else
        {
            VLOG(0) << "NO_DB_RESULTS" << std::endl;
            match.status = DLoopDetector::NO_DB_RESULTS;
        }
    }
    // Store the BoW vectors if we are going to use it in the next iterations.
    if(dloop_param.use_nss && (int)entryId + 1 > dloop_param.dislocal)
    {
        last_bowvec = curr_BoW;
    }
    return match.detection();
}

void LoopCloser::featureMatchesBow(const DBoW2::FeatureVector& curr_featVec,const DBoW2::FeatureVector& best_featVec,
                                   vector<unsigned int> &indices_1, vector<unsigned int> &indices_2, EntryId best_entry)
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

    std::vector<cv::Mat> img_desc1, img_desc2;

    for (auto &d : m_image_intraMatches[best_entry])
        img_desc1.push_back(d.matchDesc);

    for (auto &d : m_image_intraMatches.back())
        img_desc2.push_back(d.matchDesc);

    while(curr_iter != curr_end && best_iter != best_end)
    {
        if (curr_iter->first == best_iter->first)
        {
            std::vector<unsigned int> i_ind_temp, j_ind_temp;
            feat_extract->getMatches_distRatio(img_desc1, best_iter->second,
                                               img_desc2, curr_iter->second, i_ind_temp, j_ind_temp,
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


cv::Mat LoopCloser::getRelativePose(cv::Mat curr_frame_pose, cv::Mat &best_match_pose)
{
    // Make the matrices homogeneous.
    cv::Mat homogeneous = (cv::Mat_<double>(1,4) << 0 , 0 , 0 , 1);
    curr_frame_pose.push_back(homogeneous);

    // Convert to Eigen::Isometry3d
    Eigen::Matrix4d eigenMat;
    Eigen::Isometry3d currentPose, bestMatchPose;

    cv::cv2eigen(curr_frame_pose, eigenMat);
    currentPose = Eigen::Isometry3d(eigenMat);

    cv::cv2eigen(best_match_pose, eigenMat);
    bestMatchPose = Eigen::Isometry3d(eigenMat);

    // Calculate relative pose
    Eigen::Isometry3d relativePose = currentPose.inverse() * bestMatchPose;

    // Convert back to cv::Mat
    cv::eigen2cv(relativePose.matrix(), curr_frame_pose);

    VLOG(0) << "This is the computed matrix - \n" << curr_frame_pose << std::endl;

    return curr_frame_pose;
}

bool LoopCloser::checkAbsolutePose(const DBoW2::EntryId &old_Id, std::vector<cv::DMatch> &matches,
                                   LoopCloser::Detection_Frame &result)
{
    m_br1.clear();
    m_camCorrespondences1.clear();
    m_inliers.clear();
    m_inliers = std::vector<bool>(matches.size(), false);
    points_t points;
    for (auto &m : matches)
    {
        cv::Mat pt3d;
        int l_id = key_frames[(int)old_Id]->lIds[m.queryIdx];
        if(l_id == -1)
            continue;
        pt3d = m_map->getLandmark(l_id)->pt3D;
        Eigen::Vector3d pt;
        for(int j = 0; j < 3; j++)
        {
            pt[j] = pt3d.at<double>(j,0);
        }
        points.push_back(pt);
        IntraMatch intra_curr = m_frame->intraMatches[m.trainIdx];
        for(int i = 0; i < m_frame->num_cams_; i++)
        {
            if(intra_curr.matchIndex[i] != -1)
            {
                cv::Point2f uv_coord = m_frame->image_kps[i][intra_curr.matchIndex[i]].pt;
                double norm_x = (uv_coord.x - K_mat[i].at<double>(0,2))/K_mat[i].at<double>(0,0);
                double norm_y = (uv_coord.y - K_mat[i].at<double>(1,2)) /K_mat[i].at<double>(1,1);
                Eigen::Vector3d pt_n;
                pt_n[0] = norm_x;
                pt_n[1] = norm_y;
                pt_n[2] = 1.0;
                m_br1.emplace_back(pt_n/pt_n.norm());
                m_camCorrespondences1.emplace_back(i);
                break;
            }
        }

    }

    assert(m_camCorrespondences1.size() == points.size());
    // Non-Central Absolute adapter.
    opengv::absolute_pose::NoncentralAbsoluteAdapter adapter(m_br1, m_camCorrespondences1, points,
                                                             m_frame->translations, *m_frame->rotations_ptr);

    opengv::sac::Ransac<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;
    ransac.sac_model_ = std::make_shared<sac_problems::absolute_pose::AbsolutePoseSacProblem>(
            adapter,
            sac_problems::absolute_pose::AbsolutePoseSacProblem::GP3P);
    ransac.threshold_ = 1.0 - cos(atan(sqrt(2.0)*0.5/K_mat[0].at<double>(0,2)));
    ransac.max_iterations_ = 100;   // TODO: Add this parameter to the config file.
    ransac.computeModel();
    VLOG(0) << "Inlier size - " << ransac.inliers_.size() << std::endl;

    // Get the absolute pose.
    opengv::transformation_t essential_matrix = ransac.model_coefficients_;
    cv::Mat pose;
    cv::eigen2cv(essential_matrix, pose);
    if (!pose.empty() && ransac.inliers_.size() >= dloop_param.min_Fpoints)
    {
        for(size_t i = 0; i < ransac.inliers_.size(); i++)
        {
            m_inliers[ransac.inliers_[i]] = true;
        }
        VLOG(0) << "Loop closure candidate verified!" << std::endl;
        // TODO: Convert the pose to the body frame when IMU added.
        result.pose_to_check.first = pose;     // Absolute pose
        cv::Mat relative_pose_cal = getRelativePose(pose, key_frames[old_Id]->pose);
        result.relative_pose = convertPose3_CV2GTSAM(relative_pose_cal);
        result.pose_to_check.second = pose * relative_pose_cal;     // relative pose.
        getLIds(matches, old_Id, result.lIds, result.measurements);
        VLOG(0) << "LId size - " << result.lIds.size() << " number of measuremenets - " << result.measurements.size();
        // EntryId best_match = old_Id;
        // EntryId current_id = orb_database->size() - 1;
        // visualize(current_id, best_match, matches);  // Can only be used with image sequences.
        return true;
    }
    return false;

}

bool LoopCloser::checkEssentialMatrix(const EntryId& old_Id, std::vector<cv::DMatch>& matches, Detection_Frame &result)
{
    try {
        m_br1.clear();
        m_br2.clear();
        m_camCorrespondences1.clear();
        m_camCorrespondences2.clear();
        m_inliers.clear();
        m_inliers = std::vector<bool>(matches.size(), false);

        double inlier_percent = 0.0;

        for (auto& m : matches) {
            IntraMatch iMatch = m_image_intraMatches[old_Id][m.queryIdx];
            for (int i = 0; i < m_frame->num_cams_; i++) {
                if (iMatch.matchIndex[i] != -1) {
                    // TODO: Change to the undistorted keypoints.
                    cv::Point2f uv_coord = key_frames[old_Id]->image_kps[i][iMatch.matchIndex[i]].pt;
                    double norm_x = (uv_coord.x - K_mat[i].at<double>(0, 2)) / K_mat[i].at<double>(0, 0);
                    double norm_y = (uv_coord.y - K_mat[i].at<double>(1, 2)) / K_mat[i].at<double>(1, 1);
                    Eigen::Vector3d pt;
                    pt[0] = norm_x;
                    pt[1] = norm_y;
                    pt[2] = 1.0;
                    m_br1.emplace_back(pt / pt.norm());
                    m_camCorrespondences1.emplace_back(i);
                    break;
                }
            }

            IntraMatch iMatch_cur = m_frame->intraMatches[m.trainIdx];
            for (int i = 0; i < m_frame->num_cams_; i++) {
                if (iMatch_cur.matchIndex[i] != -1) {
                    // TODO: Change the keypoints to the undistorted ones.
                    cv::Point2f uv_coord = m_frame->image_kps[i][iMatch_cur.matchIndex[i]].pt;
                    double norm_x = (uv_coord.x - K_mat[i].at<double>(0, 2)) / K_mat[i].at<double>(0, 0);
                    double norm_y = (uv_coord.y - K_mat[i].at<double>(1, 2)) / K_mat[i].at<double>(1, 1);
                    Eigen::Vector3d pt;
                    pt[0] = norm_x;
                    pt[1] = norm_y;
                    pt[2] = 1.0;
                    m_br2.emplace_back(pt / pt.norm());
                    m_camCorrespondences2.emplace_back(i);
                    break;
                }

            }
        }

        opengv::relative_pose::NoncentralRelativeAdapter adapter(m_br1, m_br2, m_camCorrespondences1,
                                                                 m_camCorrespondences2, m_frame->translations,
                                                                 *m_frame->rotations_ptr);
        opengv::sac::Ransac<sac_problems::relative_pose::NoncentralRelativePoseSacProblem> ransac;
        ransac.sac_model_ = std::make_shared<sac_problems::relative_pose::NoncentralRelativePoseSacProblem>(
                adapter,
                sac_problems::relative_pose::NoncentralRelativePoseSacProblem::SEVENTEENPT
        );
        ransac.threshold_ = 2.0 * (1.0 - cos(atan(sqrt(2.0) * 0.5 / K_mat[0].at<double>(0, 2))));
        ransac.max_iterations_ = 2000; // TODO: Reduce the number of iterations as this is making the Loop closure functionality slower. Add this as a parm in config file.

        // Compute the essential matrix.
        ransac.computeModel();
        VLOG(0) << "Inlier Percentage - " << inlier_percent << std::endl;
        VLOG(0) << "Number of inliers    - " << static_cast<int>(ransac.inliers_.size());

        for(size_t i = 0; i < ransac.inliers_.size(); i++)
        {
            m_inliers[ransac.inliers_[i]] = true;
        }

        opengv::transformation_t essential_matrix = ransac.model_coefficients_;
        cv::Mat pose;
        cv::eigen2cv(essential_matrix, pose);
        if (!pose.empty() && ransac.inliers_.size() >= dloop_param.min_Fpoints)
        {
            VLOG(0) << "Loop closure candidate verified!" << std::endl;
            VLOG(0) << "This is the computed essential matrix - \n" << pose << std::endl;
            // EntryId best_match = old_Id;
            // EntryId current_id = orb_database->size() - 1;
            // TODO: Convert the pose to the body frame when IMU added.
            result.relative_pose = convertPose3_CV2GTSAM(pose);
            getLIds(matches, old_Id, result.lIds, result.measurements);
            VLOG(0) << "LId size - " << result.lIds.size() << " number of measurements - " << result.measurements.size();
            // visualize(current_id, best_match, matches);
            return true;
        }
        return false;
    } catch (const std::exception& e) {
        // Handle memory errors or other exceptions
        std::cerr << "Exception occurred: " << e.what() << std::endl;
        // Add code here to handle the error gracefully, e.g., clean up resources, log the error, etc.
        return false; // Indicate failure
    }
}

std::string LoopCloser::convertDoubleToString(double value)
{
    value = value * 1000000;
    std::ostringstream oss;
    oss << std::fixed << value;
    std::string stringValue = oss.str();

    // Remove the decimal point and any trailing zeros
    stringValue.erase(stringValue.find('.'), stringValue.length());

    return stringValue;
}

std::string LoopCloser::findMatchingFile(std::string& filePath, std::string& partialName) {
    for (const auto& file : std::filesystem::directory_iterator(filePath)) {
        if (file.path().filename().string().find(partialName) != std::string::npos) {
            return file.path().filename().string();
        }
    }
    return "";
}


void LoopCloser::displayImages(const std::vector<cv::Mat>& current_images, const std::vector<cv::Mat>& best_match_images)
{
    cv::Mat output_curr_image;
    cv::hconcat(current_images, output_curr_image);
    cv::Mat output_best_match;
    cv::hconcat(best_match_images, output_best_match);
    cv::Mat output;
    cv::vconcat(output_curr_image, output_best_match, output);
    assert(output_best_match.empty() == 0);
    cv::resize(output, output, cv::Size(output.cols * 0.6, output.rows * 0.6), 0 , 0, cv::INTER_LINEAR);
    cv::namedWindow("Matches", cv::WINDOW_AUTOSIZE);

    while (display_thread_running) {
        cv::imshow("Matches", output);
        int key = cv::waitKey(30);

        if (key == 'q') {
            display_thread_running = false;
            break;
        }

        // Wait for a short duration to control the video speed
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    cv::destroyAllWindows();
}

void LoopCloser::getLIds(std::vector<cv::DMatch> &matches, const DBoW2::EntryId &best_match,
                         std::vector<int>& matched_Lid, std::vector<std::vector<std::tuple<int, cv::Point2f>>> &matched_measurement)
{
    matched_Lid.clear();
    matched_measurement.clear();
    int pp = 0;
    for (auto& m : matches)
    {
        if(m_inliers[pp])
        {
            // Lids of the previous images.
            if(key_frames[best_match]->lIds[m.queryIdx] == -1)
            {
                continue;
            }
            matched_Lid.emplace_back(key_frames[best_match]->lIds[m.queryIdx]);
            // Intra matches for the current image -
            IntraMatch im = m_frame->intraMatches[m.trainIdx];
            int camInd = 0;
            std::vector<std::tuple<int, cv::Point2f>> measurement;
            for(int featInd : im.matchIndex)
            {
                if (featInd != -1)
                {
                    measurement.emplace_back(camInd, m_frame->image_kps_undist[camInd][featInd].pt);
                }
                camInd++;
            }
            matched_measurement.emplace_back(measurement);
        }
        pp++;
    }
    assert(matched_Lid.size() == matched_measurement.size());
}

void LoopCloser::drawMatches(std::vector<cv::Mat> current_images, std::vector<cv::Mat> best_match_images,
                             std::vector<cv::DMatch>& matches, DBoW2::EntryId& best_match)
{
    int pp = 0;
    for(auto& m : matches)
    {
        if(m_inliers[pp])
        {
            IntraMatch im1 = m_frame->intraMatches[m.trainIdx];
            // We want the intramatches for the prev image as well -
            IntraMatch im2 = key_frames[best_match]->intraMatches[m.queryIdx];

            cv::Point2f p1;
            //For the second image:
            cv::Point2f p2;

            int camIndCur = 0;
            cv::Scalar cc;
            if(im1.mono || im2.mono) // For both the cases.
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
                    p1 = m_frame->image_kps_undist[camIndCur][featInd].pt;
                    cv::circle(current_images[camIndCur], p1, 4, cc, 2);
                }
                camIndCur++;
            }
            camIndCur = 0;
            //  -- For the second Image :
            for(int featInd: im2.matchIndex)
            {
                if(featInd != -1)
                {
                    p2 = key_frames[best_match]->image_kps_undist[camIndCur][featInd].pt;
                    cv::circle(best_match_images[camIndCur], p2, 4, cc, 2);
                }
                camIndCur++;
            }
        }
        pp++;
    }

    if(!display_thread_running)
    {
        display_thread_running = true;
        std::thread display_thread([this, current_images, best_match_images]() {
            this->displayImages(current_images, best_match_images);
        });
        display_thread.detach();
    }
}

void LoopCloser::visualize(DBoW2::EntryId current_id, DBoW2::EntryId best_match,
                           std::vector<cv::DMatch>& matches)
{
    // Put the timestamp over the images.
    cv::Point text_position(10, 30);
    int font_size = 1;
    cv::Scalar font_color(0, 255, 0);
    int font_weight = 2;

    std::string current_time = convertDoubleToString(entryStamps[current_id]);
    std::string best_match_time = convertDoubleToString(entryStamps[best_match]);

    // Loop through the number of cameras and get the images.
    std::vector<cv::Mat> current_images = {};
    std::vector<cv::Mat> best_match_images = {};
    // TODO: Investigate the addition of the additional numbers in the end of the images. In this case: 388.
    for(int i = 0 ; i < m_frame->num_cams_; i++) {
        std::string file_path = image_dir + std::to_string(i) + "/";
        // CURRENT
        std::string current_frame_dir = file_path + findMatchingFile(file_path, current_time); //+ "388.png";
        current_images.push_back(cv::imread(current_frame_dir));

        // BEST MATCH
        std::string best_match_frame_dir = file_path + findMatchingFile(file_path, best_match_time); //+ "388.png";
        best_match_images.push_back(cv::imread(best_match_frame_dir));
        if (i == 0)  // Put the database id on the first image only.
        {
            cv::putText(current_images.at(0), std::to_string(current_id), text_position, cv::FONT_HERSHEY_SIMPLEX,
                        font_size, font_color, font_weight);
            cv::putText(best_match_images.at(0), std::to_string(best_match), text_position, cv::FONT_HERSHEY_SIMPLEX,
                        font_size, font_color, font_weight);
        }
    }
    // Safety:
    if(current_images.empty() || best_match_images.empty())
    {
        VLOG(0) << "Error: No Images were read!" << std::endl;
        return;
    }

    drawMatches(current_images, best_match_images, matches, best_match);

}

bool LoopCloser::geometricVerification(const EntryId& old_entry, Detection_Frame &result)
{
    VLOG(0) << "Checking for geometric consistency!" << std::endl;
    DBoW2::FeatureVector featVec = curr_FeatVec;
    std::vector<cv::DMatch> matches;
    std::vector<cv::DMatch> mono_matches;
    const FeatureVector &oldVec = orb_database->retrieveFeatures(old_entry);
    vector<unsigned int> i_old, i_cur;
    featureMatchesBow(featVec, oldVec, i_old, i_cur, old_entry);
    for(int i = 0 ; i < i_old.size(); i++)
    {
        cv::DMatch m;
        m.queryIdx = static_cast<int>(i_old[i]);
        m.trainIdx = static_cast<int>(i_cur[i]);
        if (i_old[i] < m_intraMatchesSize)
            matches.push_back(m);
        else
            mono_matches.push_back(m);
    }
    if((int) matches.size() >= dloop_param.min_Fpoints)
    {
        switch (type)
        {
            case SEVENTEENPT:
                return checkEssentialMatrix(old_entry, matches, result);
            case ABSOLUTE_POSE:
                return checkAbsolutePose(old_entry, matches, result);
            default:
                return checkEssentialMatrix(old_entry, matches, result);
        }
    }

    return false;
}


void LoopCloser::clearDataBase()
{
    orb_database->clear();
    entryStamps.clear();
    m_image_intraMatches.clear();
}

void LoopCloser::saveDatabase(const std::string &fileName)
{
    VLOG(0) << "Saving database to file ..." << std::endl;
    VLOG(0) << *orb_database << std::endl;  // Get the info on the database.
    orb_database->save(fileName);
}

void LoopCloser::loadDatabase(const std::string &fileName)
{
    VLOG(0) << "Loading saved database ..." << std::endl;
    orb_database->load(fileName);
    VLOG(0) << *orb_database << std::endl;
}

int LoopCloser::getORBDatabaseSize()
{
    return (int)orb_database->size();
}

void LoopCloser::setFeatureExtractor(ORBextractor *extract)
{
    // Set the feature extractor
    feat_extract = extract;
}


