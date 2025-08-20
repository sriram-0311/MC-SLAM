//
// Created by aryaman on 11/3/23.
//
#include <fstream>
#include <utility>
#include <filesystem>
#include "glog/logging.h"
#include <opencv2/core.hpp>
#include "MCSlam/Tracking.h"
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>


Tracking::Tracking()
{
}

void Tracking::loadKDTree(const boost::json::value& json_File_value)
{
    jsonParser = std::make_unique<boost::json::value>(json_File_value);
    loadPositions();
    // Create GenericIndex
    index = new cv::flann::GenericIndex<cv::flann::L2<double>>(trainData, cvflann::KDTreeIndexParams(4));
    kdTree = new cv::flann::GenericIndex<cv::flann::L2<double>>(*index);
    VLOG(0) << "Loaded the KDTree ..." << std::endl;
}

std::vector<cv::Mat> Tracking::getAllPoses(cv::Mat Tbc) {
    return posesFromMap;
}


void Tracking::inverse(cv::Mat& a_T_b, cv::Mat& b_T_a)
{
    cv::Mat R_ab = a_T_b(cv::Rect(0, 0, 3, 3));
    cv::Mat t_ab = a_T_b(cv::Rect(3, 0, 1, 3));

    cv::Mat R_ba = R_ab.t();
    cv::Mat t_ba = -R_ba * t_ab;

    b_T_a = cv::Mat::eye(4, 4, CV_64F);
    R_ba.copyTo(b_T_a(cv::Rect(0, 0, 3, 3)));
    t_ba.copyTo(b_T_a(cv::Rect(3, 0, 1, 3)));
}

void Tracking::createCameraRig(std::vector<gtsam::Matrix>& cameraParameters, std::vector<gtsam::Pose3>& R_T_mats,
                               gtsam::Pose3& b_T_c)
{
    multi_cam_rig = boost::make_shared<CameraSet<PinholePose<Cal3_S2>>>();
    // Assign the parameters and the extrinsics.
    int iter = 0;
    for(auto& cam : cameraParameters)
    {
        // For each camera get the extrinsics, the R,T matrix and add to the camera set.
        Cal3_S2::shared_ptr K(new Cal3_S2( cam(0,0), cam(1,1), 0, cam(0,2), cam(1,2)));
        auto camera = PinholePose<Cal3_S2>(R_T_mats[iter].inverse(), K);    // Inverse of the extrinsics. (cn_T_c0)
        multi_cam_rig->push_back(camera);
        iter++;
    }
    std::string p;
    multi_cam_rig->print(p);
    // Set the transformation between the body and the camera.
    cameraRig.b_T_c0 = b_T_c;
}

void Tracking::loadPositions()
{
    // Predefine the size of the trainData matrix
    boost::json::object jsonObject = jsonParser->as_object();
    // Get the total number of entries in the json Object
    int totalEntries = (int) jsonObject.size();
    VLOG(0) << "Total number of entries - " << totalEntries << std::endl;
    // Initialize the trainData matrix
    trainData = cv::Mat(totalEntries, 3, CV_64F);
    // reserve memory for the poses vector
    posesFromMap.reserve(totalEntries);
    for(const auto& entry : jsonObject)
    {
        boost::json::value tempPosition = entry.value().at("position");
        // load the poses from the json file
        boost::json::value tempPose = entry.value().at("pose");
        cv::Mat poseMat = loadPoseFromJson(tempPose);
        auto position = boost::json::value_to<std::vector<double>>(tempPosition);
        // Get the row number
        int rowNumber = std::stoi(entry.key().substr(6));
        // Fill up the trainData matrix
        trainData.at<double>(rowNumber, 0) = position[0];
        trainData.at<double>(rowNumber, 1) = position[1];
        trainData.at<double>(rowNumber, 2) = position[2];
        // push the pose back to the vector
        posesFromMap.push_back(poseMat);
    }

    assert(trainData.size() == cv::Size(3, totalEntries));

    VLOG(0) << "Train data size - " << trainData.size << std::endl;
}

cv::Mat Tracking::loadPoseFromJson(const boost::json::value& jsonPoseValue) {
    // Convert the JSON array to a std::vector<std::vector<double>>
    std::vector<std::vector<double>> vecPose = boost::json::value_to<std::vector<std::vector<double>>>(jsonPoseValue);

    // Convert the std::vector<std::vector<double>> to a cv::Mat
    cv::Mat poseMat(vecPose.size(), vecPose[0].size(), CV_64F);
    for (int i = 0; i < vecPose.size(); ++i) {
        for (int j = 0; j < vecPose[0].size(); ++j) {
            poseMat.at<double>(i, j) = vecPose[i][j];
        }
    }

    return poseMat;
}

void Tracking::queryPoints(const cv::Mat &queryPoint, cv::Mat& result, cv::Mat& dist)
{

    if(result.type() != CV_32S || dist.type() != CV_32F)
    {
        result = cv::Mat(1, Knn, CV_32S);
        dist = cv::Mat(1, Knn, CV_64F);
    }
    kdTree->knnSearch(queryPoint, result, dist, Knn, cvflann::SearchParams(32, 0.0, false));
    VLOG(0) << "Result from tracking module: " << result << std::endl;
    VLOG(0) << "Distance from tracking module: " << dist << std::endl;
}

void Tracking::processLandmarks(int ID, std::unordered_map<int, cv::Mat>& lid_points_pair, std::map<int, cv::Mat>& descriptorsPair)
{
    auto start2 = std::chrono::high_resolution_clock::now();
    boost::json::value landmarks = jsonParser->at("entry_" + std::to_string(ID)).as_object().at("l_ids");
    auto landmarkIndices = boost::json::value_to<std::vector<int>>(landmarks);

    boost::json::array map_points = jsonParser->at("entry_" + std::to_string(ID)).as_object().at("points").as_array();
    std::vector<std::vector<double>> mapPoints;

    // get the descriptors for the points
    boost::json::array descriptors = jsonParser->as_object()["entry_" + std::to_string(ID)].as_object()["descriptor"].as_array();
    std::vector<cv::Mat> descriptorsVector;
    // Convert to the vector descriptors.
    arrayToMat<uchar>(descriptors, descriptorsVector);
    for(const auto& map_point : map_points)
    {
        mapPoints.push_back(boost::json::value_to<std::vector<double>>(map_point));
    }
    auto end2 = std::chrono::high_resolution_clock::now();
    VLOG(0) << "Time taken to convert map points - " << std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start2).count() << std::endl;
    int point_counter = 0;
    auto start3 = std::chrono::high_resolution_clock::now();

    for (int lidIterator = 0; lidIterator < landmarkIndices.size(); lidIterator++) {
        if (landmarkIndices[lidIterator] == -1) {
            continue;
        } else {
            std::unique_lock<std::mutex> lock(landmarkMapMutex);
            if (lid_points_pair.find(landmarkIndices[lidIterator]) == lid_points_pair.end() &&
                descriptorsPair.find(landmarkIndices[lidIterator]) == descriptorsPair.end()) {
                lid_points_pair[landmarkIndices[lidIterator]] = cv::Mat(1, 3, CV_64F);
                lid_points_pair[landmarkIndices[lidIterator]].at<double>(0, 0) = mapPoints[point_counter][0];
                lid_points_pair[landmarkIndices[lidIterator]].at<double>(0, 1) = mapPoints[point_counter][1];
                lid_points_pair[landmarkIndices[lidIterator]].at<double>(0, 2) = mapPoints[point_counter][2];
                descriptorsPair[landmarkIndices[lidIterator]] = descriptorsVector[lidIterator];
                point_counter++;
            } else {
                point_counter++;
            }
            lock.unlock();
        }
    }
    // SANITY CHECKS
    VLOG(2) << "number of features in current frame " << landmarkIndices.size() << std::endl;
    VLOG(2) << "number of landmarks " << point_counter << std::endl;
    VLOG(2) << "map points from json file " << mapPoints.size() << std::endl;
    VLOG(2) << "descriptors from json file " << descriptorsVector.size() << std::endl;
    auto end3 = std::chrono::high_resolution_clock::now();
    VLOG(2) << "Time taken to process landmarks - " << std::chrono::duration_cast<std::chrono::milliseconds>(end3 - start2).count() << std::endl;
}

void Tracking::retrieveLandmarks(cv::Mat& viewPoints, std::map<int, cv::Mat>& descriptorsMap, std::unordered_map<int, cv::Mat>& landmarksMap)
{
    boost::json::object jsonObject = jsonParser->as_object();
    int numThreads = Knn;
//    std::vector<std::thread> threads(numThreads);
    // Create threads to process landmarks concurrently
    auto start1 = std::chrono::high_resolution_clock::now();
    // DISABLED MULTI-THREADING OF KD-TREE SEARCH AND LANDMARK RETRIEVAL FOR NOW. CAN BE ENABLED IN FUTURE COMMIT.
//    for (int i = 0; i < numThreads; ++i)
//    {
//        threads[i] = std::thread(&Tracking::processLandmarks, this, viewPoints.at<int>(0, i), std::ref(landmarkMap), std::ref(descriptorsMap));
//    }
//    auto end = std::chrono::high_resolution_clock::now();
//    VLOG(0) << "Time taken to process landmarks - " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl;
//    // Join the threads to wait for their completion
//    auto start1 = std::chrono::high_resolution_clock::now();
//    for (int i = 0; i < numThreads; ++i)
//    {
//        threads[i].join();
//    }
    ////////////////////////////////////
    // doing it in 1 thread itself
    for (int i = 0; i < numThreads; ++i)
    {
        processLandmarks(viewPoints.at<int>(0, i), landmarksMap, descriptorsMap);
    }
    auto end1 = std::chrono::high_resolution_clock::now();
    VLOG(0) << "Time taken to join threads - " << std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1).count() << std::endl;
}

std::map<int, std::vector<cv::Mat>> Tracking::project_(std::unordered_map<int, cv::Mat> &lid_points_pair,
                        std::map<unsigned int, std::vector<cv::KeyPoint>> &keypoints, std::map<int, cv::Mat>& descriptors,
                        std::map<unsigned int, std::vector<cv::Mat>>& descriptorsMap, std::map<int, std::vector<int>>& projectedLandmarkIds)
{
    // map to store the landmark's 3d points that are successfully projected
    std::map<int, std::vector<cv::Mat>> projectedLandmarks;
    // Iterator for the landmark map
    std::unordered_map<int, cv::Mat>::iterator it;
    // Measurement vector
    std::vector<std::vector<Point2, Eigen::aligned_allocator<Point2>>> measurements;
    // For the landmark map, project the points onto the camera
    for(it = lid_points_pair.begin(); it != lid_points_pair.end(); it++)
    {
        // Get the 3D point and create a gtsam::Point
        gtsam::Point3 point3(it->second.at<double>(0, 0), it->second.at<double>(0, 1), it->second.at<double>(0, 2));
        // Point in the body's frame
        point3 = cameraRig.c0_T_w.transformFrom(point3);
        // If there is a Cheirality exception, skip the landmark.
        try
        {
            std::vector<Point2, Eigen::aligned_allocator<Point2>> measurement = multi_cam_rig->project2(point3);
            // For each of the cameras, get the measurement and pushback to the keypoints vector
            for(int i = 0; i < measurement.size(); ++i)
            {
                // Create a keypoint
                cv::KeyPoint tempKeypoint;
                tempKeypoint.pt.x = (float)measurement[i].x();
                tempKeypoint.pt.y = (float)measurement[i].y();
                tempKeypoint.size = 1.0;
                tempKeypoint.angle = 0.0;
                tempKeypoint.response = 1.0;
                tempKeypoint.octave = 0;
                tempKeypoint.class_id = it->first;
                // check if the keypoint is within the image bounds
                if (tempKeypoint.pt.x < 0 || tempKeypoint.pt.x > imgCols || tempKeypoint.pt.y < 0 || tempKeypoint.pt.y > imgRows)
                {
                    continue;
                }
                keypoints[i].push_back(tempKeypoint);
                projectedLandmarks[i].push_back(it->second);
                projectedLandmarkIds[i].push_back(it->first);
                // Add the descriptor to the map
                descriptorsMap[i].push_back(descriptors[it->first]);
            }
        }
        catch (const std::exception& e)
        {
            continue;
        }
    }
    VLOG(0) << "Successfully projected landmarks - " << keypoints[0].size() << std::endl;
    return projectedLandmarks;
}

std::map<int, std::vector<cv::Mat>> Tracking::projectLandmarks(std::unordered_map<int, cv::Mat> &lid_points_pair,
                                                               std::map<unsigned int, std::vector<cv::KeyPoint>> &keypoints,
                                                               std::map<int, cv::Mat>& descriptors,
                                                               std::map<unsigned int, std::vector<cv::Mat>>& descriptorsMap,
                                                               std::map<int, std::vector<int>>& projectedLandmarkIds)
{
    /**
     *  Formula for getting the keypoints in each of the cameras - keypoints = K * c_T_b * b_T_w * w_L
     */

    std::map<int, std::vector<cv::Mat>> projectedLandmarks = project_(lid_points_pair, keypoints, descriptors,
                                                                      descriptorsMap, projectedLandmarkIds);
    /// Visualize the images
//    int cameras = 3;
//    visualizeKeyPoints(keypoints, cameraRig.timestamp, cameras);
    return projectedLandmarks;
}

/* For Visualization : Created during the initial phase of fast tracking development to visualize the projection of
 * retrieved landmarks from the map. DEPRECATED : Can either be removed in future commit or withheld for debugging */
std::string Tracking::findMatchingFile(std::string &filePath, std::string &partialName)
{
    for (const auto& file : std::filesystem::directory_iterator(filePath)) {
        if (file.path().filename().string().find(partialName) != std::string::npos) {
            return file.path().filename().string();
        }
    }
    return "";
}

/* For Visualization : Created during the initial phase of fast tracking development to visualize the projection of
 * retrieved landmarks from the map. DEPRECATED : Can either be removed in future commit or withheld for debugging */
void Tracking::visualizeKeyPoints(std::map<int, std::vector<cv::KeyPoint>> &keyPoints, double timestamp, int num_cams)
{
    std::string image_dir = "/home/aryaman/catkin_ws/ISEC_Lab1/KRI/cam";
    std::string current_time = std::to_string(timestamp);
    // Remove the decimal point from the std::string
    current_time.erase(std::remove(current_time.begin(), current_time.end(), '.'), current_time.end());
    auto images = std::vector<cv::Mat>(num_cams);
    for(int i = 0; i < num_cams; ++i)
    {
        std::string file_path = image_dir + std::to_string(i) + "/";
        std::string current_frame_dir = file_path + findMatchingFile(file_path, current_time);
        images[i] = cv::imread(current_frame_dir);
    }
    // Visualize the keypoints
    std::vector<cv::Mat> tempImages(num_cams);
    for(int i = 0; i < num_cams; ++i)
    {
        cv::Mat tempImage;
        cv::drawKeypoints(images[i], keyPoints[i], tempImage, cv::Scalar(0.0, 0.0, 255.0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        // Show the image
        cv::imshow("Keypoints", tempImage);
//        cv::waitKey(0);
    }
}

void Tracking::querryEachFrame(int cameraIndex, std::vector<cv::KeyPoint>& keyPoints, std::vector<cv::Mat>& descriptors,
                               std::map<int, std::vector<cv::KeyPoint>>& bestMatches,
                               std::map<int, std::vector<cv::Mat>>& bestMatchLandmarks,
                               std::map<int, std::vector<cv::Mat>>& projectedLandmarks,
                               std::map<int, std::vector<int>>& bestMatchLandmarkIds,
                                 std::map<int, std::vector<int>>& landmark_ids,
                               MultiCameraFrame  *currentFrame, ORBextractor* orbextractor) {
    std::vector<cv::Point2f> pts;
    vector<double> bestDists;
    bestDists.reserve(currentFrame->image_kps[cameraIndex].size());
    for (int j = 0; j < keyPoints.size(); j++){
        // get the query point
        cv::Mat queryPoint = cv::Mat::zeros(1, 2, CV_64F);
        queryPoint.at<double>(0, 0) = keyPoints[j].pt.x;
        queryPoint.at<double>(0, 1) = keyPoints[j].pt.y;
        // indices of the nearest points
        cv::Mat indices = cv::Mat::zeros(1, 10, CV_32S);
        // distance to the nearest points
        cv::Mat dists = cv::Mat::zeros(1, 10, CV_64F);

        if (queryPoint.empty() || indices.empty() || dists.empty()) {
            std::cout << "queryPoint, indices, or dists is empty" << std::endl;
            return;
        }

        // query the kd tree
        currentFrame->image_kps_kdtree[cameraIndex]->knnSearch(queryPoint, indices, dists, 10, cvflann::SearchParams(64, 0.0, false));
        VLOG(3) << "Indices close to the query point - " << keyPoints[j].pt << " - " << indices << std::endl;
        // eliminiate the indices that correspond to key points more than 20 pixels away
        for (int k = 0; k < indices.cols; k++) {
            if (dists.at<double>(0, k) > 10000) {
                indices.at<int>(0, k) = -1;
            }
        }
        VLOG(3) << "After eliminating far away indices - " << indices << std::endl;
        // match the nearest points using orb descriptor matching and store the best matching point alone
        // descriptor corresponding to the querry point
        cv::Mat queryDescriptor = descriptors[j];
        // descriptors corresponding to the closest point from the kd tree
        std::vector<cv::Mat> imageDescs;
        // store the original indices of the descriptors
        std::vector<int> originalIndices;
        // get the best matching descriptor
        int bestMatchIndex = -1;
        double best = 10000;
        for (int k =0; k < indices.cols; k++) {
            // get the descriptor corresponding to the index
            if (indices.at<int>(0, k) == -1) {
                continue;
            }
            cv::Mat desc = currentFrame->image_descriptors[cameraIndex][indices.at<int>(0, k)];
            // match the descriptors
            int dist = orbextractor->DescriptorDistance(queryDescriptor, desc);
            VLOG(3) << "Distance between the descriptors - " << dist << std::endl;
            if (dist < best && dist < 20) {
                bestMatchIndex = indices.at<int>(0, k);
                best = dist;
            }
        }
        // store the best matching keypoint
        // lock the mutex
        std::unique_lock<std::mutex> lock(currentFrameKeyPointMutex);
        bool isAlreadyMatched = false;
        if (bestMatchIndex != -1) {
            // check if the bestMatches map has the keypoint already, then do no add this landmark to that keypoint again
            for (auto it = bestMatches[cameraIndex].begin(); it != bestMatches[cameraIndex].end(); it++) {
                if (static_cast<int>(it->pt.x) == static_cast<int>(currentFrame->image_kps[cameraIndex][bestMatchIndex].pt.x) &&
                    static_cast<int>(it->pt.y) == static_cast<int>(currentFrame->image_kps[cameraIndex][bestMatchIndex].pt.y)) {
                    isAlreadyMatched = true;
                    if (bestDists[bestMatchIndex] > best) {
                        bestMatches[cameraIndex].erase(it);
                        bestMatchLandmarks[cameraIndex].erase(bestMatchLandmarks[cameraIndex].begin() + std::distance(bestMatches[cameraIndex].begin(), it));
                        bestMatchLandmarkIds[cameraIndex].erase(bestMatchLandmarkIds[cameraIndex].begin() + std::distance(bestMatches[cameraIndex].begin(), it));
                        bestMatches[cameraIndex].push_back(currentFrame->image_kps[cameraIndex][bestMatchIndex]);
                        bestMatchLandmarks[cameraIndex].push_back(projectedLandmarks[cameraIndex][j]);
                        bestMatchLandmarkIds[cameraIndex].push_back(landmark_ids[cameraIndex][j]);
                        bestDists[bestMatchIndex] = best;
                        VLOG(3) << "Query descriptor - " << queryDescriptor.t() << std::endl;
                        VLOG(3) << "Best matching descriptor - " << currentFrame->image_descriptors[cameraIndex][bestMatchIndex] << "  distance : "<< best << std::endl;
                    }
                    break;
                }
            }
            if (!isAlreadyMatched) {
                bestMatchLandmarks[cameraIndex].push_back(projectedLandmarks[cameraIndex][j]);
                VLOG(3) << "All landmark ids in the camera Index - " << landmark_ids[cameraIndex].size() << std::endl;
                bestMatchLandmarkIds[cameraIndex].push_back(landmark_ids[cameraIndex][j]);
                vector<float> bestMatchingKps(2);
                bestMatchingKps[0] = currentFrame->image_kps[cameraIndex][bestMatchIndex].pt.x;
                bestMatchingKps[1] = currentFrame->image_kps[cameraIndex][bestMatchIndex].pt.y;
                bestMatches[cameraIndex].push_back(currentFrame->image_kps[cameraIndex][bestMatchIndex]);
                bestDists[bestMatchIndex] = best;
                VLOG(3) << "Query descriptor - " << queryDescriptor.t() << std::endl;
                VLOG(3) << "Best matching descriptor - " << currentFrame->image_descriptors[cameraIndex][bestMatchIndex] << std::endl;
            }
        }
        lock.unlock();
    }
}

std::map<int, std::vector<cv::KeyPoint>> Tracking::queryCurrentFrame(std::map<unsigned int, std::vector<cv::KeyPoint>>& keyPoints,
                                                                     std::map<int, cv::Mat>& descriptors,
                                                                     std::map<unsigned int, std::vector<cv::Mat>>& descriptors_map,
                                                                     std::map<int, std::vector<cv::Mat>>& projectedLandmarks,
                                                                     std::map<int, std::vector<int>>& landmark_ids,
                                                                     MultiCameraFrame* currentFrame, ORBextractor* orbextractor){
    // data structure to store the best matching keypoint for each point obtained from relocalization
    std::map<int, std::vector<cv::KeyPoint>> bestMatches;
    std::map<int, std::vector<cv::Mat>> bestMatchLandmarks;
    std::map<int, std::vector<int>> bestMatchLandmarkIds;
    auto num_cams = static_cast<unsigned int>(currentFrame->num_cams_);
    std::vector<std::thread> threads(num_cams);
    // for each point obtained from relocalization find the 10 nearest points in the current frame by querying the KD tree
    for (unsigned int cameraIndex = 0; cameraIndex < static_cast<unsigned int>(currentFrame->num_cams_); cameraIndex++)
    {
        threads[cameraIndex] = std::thread(&Tracking::querryEachFrame, this, cameraIndex, std::ref(keyPoints[cameraIndex]),
                                           std::ref(descriptors_map[cameraIndex]), std::ref(bestMatches),
                                           std::ref(bestMatchLandmarks),
                                           std::ref(projectedLandmarks), std::ref(bestMatchLandmarkIds),
                                           std::ref(landmark_ids), currentFrame, orbextractor);
    }
    // join the threads
    for (unsigned int it = 0; it < num_cams; it++)
    {
        threads[it].join();
    }
    // overwrite projected landmarks with the best matches
    projectedLandmarks = bestMatchLandmarks;
    landmark_ids = bestMatchLandmarkIds;
    return bestMatches;
}

void Tracking::visualizeMatches(std::map<int, std::vector<cv::KeyPoint>>& bestMatches, MultiCameraFrame* currentFrame){
    // visualize the best matches from the current frame using bestMatches[0]
    // image 0
    cv::Mat allImagesTogether = cv::Mat::zeros(Size(currentFrame->num_cams_* currentFrame->img_size.width ,currentFrame->img_size.height), CV_8UC1);
    for (auto it = bestMatches.begin(); it != bestMatches.end(); it++)
    {
        cv::Mat tempImage1;
        cv::Mat img = currentFrame->imgs[it->first];
        cv::drawKeypoints(img, it->second, tempImage1, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//            cv::drawKeypoints(img, keypoints[it->first], tempImage1, cv::Scalar(255, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        // copy the image to the allImagesTogether stiched image
        img.copyTo(allImagesTogether.rowRange(0, currentFrame->img_size.height).colRange(it->first * currentFrame->img_size.width, (it->first + 1) * currentFrame->img_size.width));
//        tempImage1.copyTo(allImagesTogether.rowRange(0, currentFrame->img_size.height).colRange(it->first * currentFrame->img_size.width, (it->first + 1) * currentFrame->img_size.width));
    }
    cv::resize(allImagesTogether,allImagesTogether, Size(currentFrame->num_cams_* currentFrame->img_size.width/2 ,currentFrame->img_size.height/2) );
    // resize the image while keeping the aspect ratio

    // show the image continuously and refresh every time without waiting for key press
    cv::imshow("Best matches", allImagesTogether);
    cv::waitKey(3);
}

void Tracking::visualizeMatches(MultiCameraFrame* currentFrame) {
    Mat all;
    //////////////  This //////////////////////////////////////
    all.create(currentFrame->imgs[0].rows, currentFrame->imgs[0].cols * currentFrame->num_cams_, CV_8UC3);
    for(int i=0; i < currentFrame->num_cams_ ; i++){
        Mat imgBGR;
        cvtColor(currentFrame->imgs[i],imgBGR , COLOR_GRAY2BGR);
        imgBGR.copyTo(all(Rect(i*currentFrame->imgs[0].cols, 0, currentFrame->imgs[0].cols, currentFrame->imgs[0].rows)));
    }
    /////////////////////////////////////////////////////////
    cv::imshow("Best matches", all);
    cv::waitKey(3);
}

template <typename T>
void Tracking::arrayToMat(const boost::json::array& jsonArray, std::vector<cv::Mat>& out_data)
{
    for (auto& i : jsonArray)
    {
        out_data.emplace_back(cv::Mat(boost::json::value_to<std::vector<T>>(i)).t());
    }
}

// Destructor
Tracking::~Tracking()
{
    delete index;
    delete kdTree;
}

