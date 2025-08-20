//
// Created by pushyami Kaveti on 8/21/21.
//

#include <MCSlam/FrontEnd.h>
#include "MCSlam/Backend.h"
#include <chrono>
//// This is the class for writing optimization functions
//// todo: Create a ISAM instance, setup the factor graph and initialise the pose prior.
//// todo: at each time step update isam and get the optmized pose and land mark estimates
//// todo : backend methods should be called from lf_slam_app after trackLF(). trackFrame() is a
//// todo : front-end method which generates the initial pose and landmark estimates.

Backend::Backend(string strSettingsFile, CamArrayConfig &camconfig, FrontEnd *fe)
        : backend_config_file(strSettingsFile) {

    cv::FileStorage fSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if (!fSettings.isOpened()) {
        cerr << "Failed to open settings file at: " << strSettingsFile << endl;
        exit(-1);
    }


    camID = (int) fSettings["CamID"];
    camTransformation = cv::Mat(4, 4, CV_64F);
    for (int i = 0; i < 3; ++i) {
        camTransformation.at<double>(i, 3) = camconfig.t_mats_[camID].at<double>(i, 0);
        for (int j = 0; j < 3; ++j)
            camTransformation.at<double>(i, j) = camconfig.R_mats_[camID].at<double>(i, j);
    }

    camTransformation.at<double>(3,0) = 0;
    camTransformation.at<double>(3,1) = 0;
    camTransformation.at<double>(3,2) = 0;
    camTransformation.at<double>(3,3) = 1;

    camTransformation = camTransformation.inv();
    camArrayConfig = &camconfig;
    for(int i =0 ; i < camconfig.num_cams_ ; i++){

        K.push_back(gtsam::Cal3_S2::shared_ptr(new gtsam::Cal3_S2(camconfig.K_mats_[i].at<double>(0, 0),
                                                                 camconfig.K_mats_[i].at<double>(1, 1), 0,
                                                                 camconfig.K_mats_[i].at<double>(0, 2),
                                                                 camconfig.K_mats_[i].at<double>(1, 2))));

        Mat R = camconfig.Kalibr_R_mats_[i];/// store the pose of the cam chain
        Mat t = camconfig.Kalibr_t_mats_[i];
        Mat kalibrPose = Mat::eye(4,4, CV_64F);
        kalibrPose.rowRange(0,3).colRange(0,3) = R.t();
        kalibrPose.rowRange(0,3).colRange(3,4) = -1* R.t()*t;
        Rt_mats_kalib_.push_back(kalibrPose.clone());

        Mat R2 = camconfig.R_mats_[i];/// store the pose of the cam chain
        Mat t2 = camconfig.t_mats_[i];
        Mat camPose = Mat::eye(4,4, CV_64F);
        camPose.rowRange(0,3).colRange(0,3) = R2.t();
        camPose.rowRange(0,3).colRange(3,4) = -1* R2.t()*t2;

        Rt_mats_.push_back(camPose.clone());
        // VLOG(3)<<"RTMats cam: "<<i<<" : "<<endl;
        // VLOG(3) << camPose <<endl;

    }

    measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, (double)fSettings["MeasurementNoiseSigma"]);

    huberModel = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(
            sqrt(5.991)), measurementNoise);

    optimizationMethod = (int)fSettings["Optimization"];
//    optimizationMethod = 1;
    reinitialized_ = false;
    if(optimizationMethod == 0){
        //parameters.optimizationParams = ISAM2DoglegParams(1.0, 1e-5, DoglegOptimizerImpl::SEARCH_EACH_ITERATION);
        parameters.relinearizeThreshold = (double)fSettings["ISAMRelinearizeThreshold"];
        parameters.relinearizeSkip = (int)fSettings["ISAMRelinearizeSkip"];
        parameters.factorization = gtsam::ISAM2Params::CHOLESKY;
        isam = gtsam::ISAM2(parameters);
    }
    else if(optimizationMethod == 1){
        params.orderingType = gtsam::Ordering::METIS;
//        params.verbosity = gtsam::NonlinearOptimizerParams::SUMMARY;
        params.maxIterations = 15;
    }
    else if(optimizationMethod == 2){
        double lag = 3.0;
        params.lambdaInitial = 1e-2;
        params.maxIterations = 15;
        fixedLagOptimizer = gtsam::BatchFixedLagSmoother(lag, params );


    }
    else{
        VLOG(0)<<"WRONG OPTIMIZATION METHOD OPTION. Must be 0-ISAM2 1-LevenbergMarquardt 2-fixedlag levenberg";
        exit(0);
    }

    windowSize = (int)fSettings["WindowBad"];
    windowCounter = windowSize;
    angleThresh = (double)fSettings["AngleThresh"];
    frontEnd = fe;

    backendType = MULTI;
    /*# backendType = 0 - MONO, 1-Multi, 2-Multi_rigid. 
    0 and 2 - deprecated
    backendType = static_cast<BACKEND_TYPE>((int)fSettings["BackEndType"]); 
    if other methods are needed later, uncomment the above lines and get the data from lf_backend.yaml
    */


    // Smart factors related
    cameraRig = static_cast<boost::shared_ptr<gtsam::CameraSet<gtsam::PinholePose< gtsam::Cal3_S2 > > > >(
            new gtsam::CameraSet<gtsam::PinholePose< gtsam::Cal3_S2 > >());


    // create a camera rig
    int iter = 0;
    for(auto &item: K){

        cv::Mat Tb_ci = frontEnd->Tbc_cv * Rt_mats_[iter];
        gtsam::PinholePose<Cal3_S2> cam(convertPose3_CV2GTSAM(Tb_ci), item);
        cameraRig->push_back(cam);
        iter++;
    }

    cameraRig->print("cameraRig");

    // set Smart factor params
    smartFactor_params.linearizationMode = gtsam::HESSIAN;
    smartFactor_params.degeneracyMode = gtsam::ZERO_ON_DEGENERACY;

    // A map of lids and rigfactor pointers
    std::map<int, RigFactor::shared_ptr> smartFactor;



}
Backend::~Backend()= default;

void Backend::displayMatches(MultiCameraFrame* prev_frame, MultiCameraFrame* current_frame){

    vector<IntraMatch> *intramatches1 = &prev_frame->intraMatches;
    vector<IntraMatch> *intramatches2 = &current_frame->intraMatches;
    vector<int> *lids1 = &prev_frame->lIds;
    vector<int> *lids2 = &current_frame->lIds;
    vector<int> lids_filtered1, lids_filtered2;
    vector<cv::KeyPoint> keyPoints1, keyPoints2;

    for(int i=0; i<intramatches1->size(); ++i){
        if(lids1->at(i)!=-1 && intramatches1->at(i).matchIndex[camID]!=-1 /* this is to filter only 0th camera feature for now*/){
            lids_filtered1.push_back(lids1->at(i));
            keyPoints1.push_back(prev_frame->image_kps[camID][intramatches1->at(i).matchIndex[camID]]);
        }
    }

    for(int i=0; i<intramatches2->size(); ++i){
        if(lids2->at(i)!=-1 && intramatches2->at(i).matchIndex[camID]!=-1 /* this is to filter only 0th camera feature for now*/){
            lids_filtered2.push_back(lids2->at(i));
            keyPoints2.push_back(current_frame->image_kps[camID][intramatches2->at(i).matchIndex[camID]]);
        }
    }

    vector<cv::DMatch> matches;
    std::cout<<"backend draw matches size: "<<lids_filtered1.size()<<std::endl;
    for(int i=0; i<lids_filtered1.size(); ++i){
        for(int j=0; j<lids_filtered2.size(); ++j){
            if(lids_filtered1.at(i) == lids_filtered2.at(j)){
                cv::DMatch match;
                match.trainIdx = j;
                match.queryIdx = i;
                matches.push_back(match);
                break;
            }
        }
    }

    cv::Mat outimg;
    cv::drawMatches(current_frame->imgs[camID], keyPoints1, prev_frame->imgs[camID], keyPoints2, matches, outimg);
    cv::imshow("backend_matches", outimg);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

bool Backend::checkTriangulationAngle(gtsam::Point3 &pose1, gtsam::Point3 &pose2, gtsam::Point3 &landmark,
                                      double &angle_deg) const{
    ///Get the Ray1 between landmark and the previous pose
    gtsam::Vector3 ray1 = landmark - pose1;
    double norm1 = ray1.norm();

    ///Get the Ray2 between landmark and the current pose
    gtsam::Vector3 ray2 = landmark - pose2;
    double norm2 = ray2.norm();

    /// Compute the angle between the two rays
    double cosTheta = ray1.dot(ray2)/(norm1*norm2);
    angle_deg = acos(cosTheta)*(180.0/3.141592653589793238463);

    if(angle_deg > angleThresh){
        return true;
    }

    return false;
}

double Backend::computeReprojectionError(gtsam::Pose3 &pose1, int camID1, gtsam::Point3 &landmark, gtsam::Point2 &obs) {
    //Get the extrisic parameters of the states
    gtsam::Matrix R1 = pose1.rotation().transpose();
    gtsam::Matrix t1 = -1 * R1 * pose1.translation();

    //COnvert the world point into the pose reference frames and apply K matrix
    gtsam::Vector pt_c1 = R1 * landmark + t1;
    double invZ1 = 1.0/pt_c1(2);
    double u_1 = camArrayConfig->K_mats_[camID1].at<double>(0,0) * pt_c1(0)*invZ1 + camArrayConfig->K_mats_[camID1].at<double>(0,2);
    double v_1 = camArrayConfig->K_mats_[camID1].at<double>(1,1) * pt_c1(1)*invZ1 + camArrayConfig->K_mats_[camID1].at<double>(1,2);
    double squareError = (u_1-obs.x())*(u_1-obs.x())+(v_1-obs.y())*(v_1-obs.y());
    return squareError;

}

gtsam::Pose3 Backend::calCompCamPose(gtsam::Pose3 &pose) {
    VLOG(3)<<"Cam Transformation: "<<camTransformation<<endl;
    return convertPose3_CV2GTSAM(camTransformation) * pose;
    return pose;
}

gtsam::Pose3 Backend::calCompCamPose(MultiCameraFrame* lf, int cID){
    Mat finalPoseCV = lf->pose * Rt_mats_[cID];
    return  convertPose3_CV2GTSAM(finalPoseCV);
}

/// insert pose prior for first initialized frame to factor graph and add initial pose estimate
void Backend::addPosePrior(int lid, GlobalMap *map){
    Landmark* l = map->getLandmark(lid);
    MultiCameraFrame* prevLF= l->KFs[0];
    gtsam::Pose3 previous_pose = convertPose3_CV2GTSAM(prevLF->pose);
    previous_pose = calCompCamPose(previous_pose);

    // Add a prior on pose x0
    gtsam::noiseModel::Diagonal::shared_ptr poseNoise = gtsam::noiseModel::Diagonal::Sigmas
            ((gtsam::Vector(6)<< gtsam::Vector3::Constant(0.1),gtsam::Vector3::Constant(0.1)).finished());
    graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', prevLF->kfID), previous_pose, poseNoise));
    initialEstimate.insert(gtsam::Symbol('x', prevLF->kfID), previous_pose);
}

/// insert landmark prior for first landmark to factor graph
void Backend::addLandmarkPrior(int lid, gtsam::Point3 &landmark){

    gtsam::noiseModel::Isotropic::shared_ptr pointNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
    //cout<<lid<< landmark<<endl;
    graph.push_back(gtsam::PriorFactor<gtsam::Point3>(gtsam::Symbol('l', lid), landmark, pointNoise)); // add directly to graph
}

/// function for monocular backend
/// finds the previous keyframe in which the landmark was last observed in the same component camera. returns the
/// corresponding measurement, pose id and pose.
bool Backend::obtainPrevMeasurement(Landmark* landmark, gtsam::Point2 &prev_measurement, int &prev_pose_id, gtsam::Pose3 &prev_pose, bool init) {

    vector<MultiCameraFrame *> KFs = landmark->KFs;
    vector<int> *feat_inds = &landmark->featInds;

    auto kf_it = KFs.rbegin();
    auto feat_it = feat_inds->rbegin();

    kf_it++;
    feat_it++;

    auto prev_KF_it = frontEnd->lfFrames.rbegin()++;
    int init_prev_pose_id = (*prev_KF_it)->frameId;

    for(; kf_it!=KFs.rend() && feat_it!=feat_inds->rend(); kf_it++, feat_it++){
        vector<IntraMatch> *intramatches = &((*kf_it)->intraMatches);
        int kp_ind = intramatches->at(*feat_it).matchIndex[camID];
        int pose_id = (*kf_it)->frameId;
        if(init){
            if(kp_ind!=-1 && pose_id == init_prev_pose_id){
                prev_measurement = convertPoint2_CV2GTSAM((*kf_it)->image_kps[camID][kp_ind]);
                prev_pose_id = (*kf_it)->frameId;
                prev_pose = convertPose3_CV2GTSAM((*kf_it)->pose);
                prev_pose = calCompCamPose(prev_pose);
                return true;
            }
        }
        else if(kp_ind!=-1 && (currentEstimate.exists(gtsam::Symbol('x', pose_id)) || initialEstimate.exists(gtsam::Symbol('x', pose_id)))){
            prev_measurement = convertPoint2_CV2GTSAM((*kf_it)->image_kps[camID][kp_ind]);
            prev_pose_id = (*kf_it)->frameId;
            prev_pose = convertPose3_CV2GTSAM((*kf_it)->pose);
            prev_pose = calCompCamPose(prev_pose);
            return true;
        }
    }
    return false;
}

/// function for monocular backend
/// filters landmarks in current frame based on observability in component camera (current and any of the previous frames) and triangulation angle
/// returns vectors of lids, landmarks, current measurements, previous measurements and previous pose ids
void Backend::filterLandmarks(MultiCameraFrame* currentFrame, GlobalMap *map, vector<int>
        &lids_filtered, vector<gtsam::Point3> &landmarks, vector<gtsam::Point2> &current_measurements,
                              vector<gtsam::Point2> &previous_measurements, vector<int> &previous_pose_ids){

    int landmarks_tot=0, landmarks_filtered=0;

    vector<IntraMatch> *intramatches = &currentFrame->intraMatches;
    vector<int> *lids = &currentFrame->lIds;

    gtsam::Pose3 current_pose = convertPose3_CV2GTSAM(currentFrame->pose);
    current_pose = calCompCamPose(current_pose);

    bool is_first_frame = currentEstimate.empty() && initialEstimate.empty();

    // browse through all instramatches
    for(int i=0; i<intramatches->size(); ++i){
        if(lids->at(i)!=-1 && intramatches->at(i).matchIndex[camID]!=-1){ // landmarks seen in current frame in the component camera

            landmarks_tot++;
            Landmark* l = map->getLandmark(lids->at(i));
            gtsam::Point3 landmark = convertPoint3_CV2GTSAM(l->pt3D);

            if(!currentEstimate.exists(gtsam::Symbol('l', lids->at(i))) and
               !initialEstimate.exists(gtsam::Symbol('l', lids->at(i)))){ // Newly triangulated landmark

                gtsam::Point2 previous_measurement;
                int previous_pose_id;
                gtsam::Pose3 previous_pose;
                bool add_landmark = obtainPrevMeasurement(l, previous_measurement, previous_pose_id, previous_pose, is_first_frame);

                //check if landmark has been observed in a previous kf on component camera
                if (!add_landmark)
                    continue;

                //check triangulation angle
                double tri_angle_deg=0;
                if(checkTriangulationAngle(const_cast<gtsam::Point3 &>(previous_pose.translation()),
                                           const_cast<gtsam::Point3 &>(current_pose.translation()), landmark, tri_angle_deg)) {
                    landmarks_filtered++;
                    lids_filtered.push_back(lids->at(i));
                    landmarks.push_back(landmark);
                    current_measurements.push_back(convertPoint2_CV2GTSAM(currentFrame->image_kps[camID][intramatches->at(i).matchIndex[camID]]));
                    previous_measurements.push_back(previous_measurement);
                    previous_pose_ids.push_back(previous_pose_id);
                }
            }

            else{ // Tracked landmark
                landmarks_filtered++;
                lids_filtered.push_back(lids->at(i));
                landmarks.push_back(landmark);
                current_measurements.push_back(convertPoint2_CV2GTSAM(currentFrame->image_kps[camID][intramatches->at(i).matchIndex[camID]]));
                previous_measurements.emplace_back();
                previous_pose_ids.push_back(-1);
            }
        }
    }
    VLOG(3)<<"Landmarks before and after filtering: "<<landmarks_tot<<" "<<landmarks_filtered<<endl;
}


/// function for monocular backend
/// filters landmarks in current frame based on observability in component camera (current and any of the previous frames) and triangulation angle
/// returns vectors of lids, landmarks, current measurements, previous measurements and previous pose ids
void Backend::filterLandmarksStringent(MultiCameraFrame* currentFrame, GlobalMap *map, vector<int>
&lids_filtered, vector<gtsam::Point3> &landmarks, vector<gtsam::Point2> &current_measurements,
                                       vector<gtsam::Point2> &previous_measurements, vector<int> &previous_pose_ids){

    int landmarks_tot=0, landmarks_filtered=0;

    vector<IntraMatch> *intramatches = &currentFrame->intraMatches;
    vector<int> *lids = &currentFrame->lIds;

    gtsam::Pose3 current_pose = convertPose3_CV2GTSAM(currentFrame->pose);
    current_pose = calCompCamPose(current_pose);

    bool is_first_frame = currentEstimate.empty() && initialEstimate.empty();
    double tri_ang;
    for(int i=0; i<intramatches->size(); ++i){
        if(lids->at(i)!=-1 && intramatches->at(i).matchIndex[camID]!=-1){ // landmarks seen in current frame in the component camera

            landmarks_tot++;
            Landmark* l = map->getLandmark(lids->at(i));
            gtsam::Point3 landmark = convertPoint3_CV2GTSAM(l->pt3D);

            gtsam::Point2 previous_measurement;
            int previous_pose_id;
            gtsam::Pose3 previous_pose;
            bool add_landmark = obtainPrevMeasurement(l, previous_measurement, previous_pose_id, previous_pose, is_first_frame);

            if(!currentEstimate.exists(gtsam::Symbol('l', lids->at(i))) and
               !initialEstimate.exists(gtsam::Symbol('l', lids->at(i)))){ // Newly triangulated landmark

                //check if landmark has been observed in a previous kf on component camera
                if (!add_landmark)
                    continue;

                //check triangulation angle
                if(checkTriangulationAngle(const_cast<gtsam::Point3 &>(previous_pose.translation()),
                                           const_cast<gtsam::Point3 &>(current_pose.translation()), landmark, tri_ang)) {
                    landmarks_filtered++;
                    lids_filtered.push_back(lids->at(i));
                    landmarks.push_back(landmark);
                    current_measurements.push_back(convertPoint2_CV2GTSAM(currentFrame->image_kps[camID][intramatches->at(i).matchIndex[camID]]));
                    previous_measurements.push_back(previous_measurement);
                    previous_pose_ids.push_back(previous_pose_id);
                }
            }

            else{ // Tracked landmark
                if(checkTriangulationAngle(const_cast<gtsam::Point3 &>(previous_pose.translation()),
                                           const_cast<gtsam::Point3 &>(current_pose.translation()), landmark, tri_ang)) {
                    landmarks_filtered++;
                    lids_filtered.push_back(lids->at(i));
                    landmarks.push_back(landmark);
                    current_measurements.push_back(convertPoint2_CV2GTSAM(
                            currentFrame->image_kps[camID][intramatches->at(i).matchIndex[camID]]));
                    previous_measurements.emplace_back();
                    previous_pose_ids.push_back(-1);
                }
            }
        }
    }
    VLOG(3)<<"Landmarks before and after filtering: "<<landmarks_tot<<" "<<landmarks_filtered<<endl;
}

bool Backend::addKeyFrameMultiSensorOffset() {

    MultiCameraFrame *currentFrame = frontEnd->currentFrame;
    GlobalMap *map = frontEnd->map;
    bool ret = true;


    /// Variable declaration
    std::map<int, set<int> > insertedRigid;
    int num_lms_filtered=0;
    vector<int> lids_filtered;
    vector<gtsam::Point3> landmarks;
    vector<gtsam::Point2> current_measurements;
    vector<gtsam::Point2> previous_measurements;
    vector<int>  previous_compcam_ids, cur_compcam_ids;
    vector<MultiCameraFrame*> previous_KFs;
    vector<bool> new_landmark_flags;

    vector<IntraMatch> *intramatches = &currentFrame->intraMatches;
    vector<int> *lids = &currentFrame->lIds;
    int current_pose_id = currentFrame->frameId;

    // browse through all instramatches
    for (int i = 0; i < intramatches->size(); ++i) {
        // if this intra match is a landmark i.e it is tracked in the current frame and is a triangulated inlier
        if (lids->at(i) != -1 ) {

            //record the landmark
            Landmark* l = map->getLandmark(lids->at(i));
            //cout<<"LANDMARK :"<<l->pt3D<<endl;
            gtsam::Point3 landmark = convertPoint3_CV2GTSAM(l->pt3D);

            if(!currentEstimate.exists(gtsam::Symbol('l', lids->at(i))) and
               !initialEstimate.exists(gtsam::Symbol('l', lids->at(i))))
            {
                /// Get the previous LF frame of the landmark and the observtaions in the component cameras
                int numKFs = l->KFs.size();
                MultiCameraFrame* lf1 = l->KFs[numKFs - 2]; // previous KF
                MultiCameraFrame* lf2 = currentFrame;  // same as l->KFs[numKFs-1]; // last KF
                IntraMatch* im1 = &lf1->intraMatches.at(l->featInds[numKFs-2]);
                IntraMatch* im2 = &lf2->intraMatches.at(l->featInds[numKFs-1]);

                Mat angles = Mat::zeros(lf1->num_cams_,lf2->num_cams_, CV_64FC1);
                Mat tri_errors = Mat::zeros(lf1->num_cams_,lf2->num_cams_, CV_64FC1);
                VLOG(3)<<"LID : "<<lids->at(i)<<", PT: "<<landmark.x()<<","<<landmark.y()<<","<<landmark.z()<<endl;
                VLOG(3)<<"Cam1 cam2 angle error1 error2"<<endl;
                ///check all combinations of rays and get the triangulation angles and reprojection errors
                for (int camID1 = 0; camID1 < lf1->num_cams_ ; camID1++){
                    int kp_ind1 = im1->matchIndex[camID1];
                    if(kp_ind1 == -1)
                        continue;
                    for (int camID2 = 0; camID2 < lf2->num_cams_ ; camID2++){
                        int kp_ind2 = im2->matchIndex[camID2];
                        if(kp_ind2 == -1)
                            continue;

                        gtsam::Pose3 transformedPose1 = calCompCamPose(lf1, camID1); // take input of the LF frame pose and comp cam id to compute the comp cam pose
                        gtsam::Pose3 transformedPose2 = calCompCamPose(lf2, camID2);
                        gtsam::Point2 obs1 = convertPoint2_CV2GTSAM(lf1->image_kps_undist[camID1][kp_ind1]);
                        gtsam::Point2 obs2  = convertPoint2_CV2GTSAM(lf2->image_kps_undist[camID2][kp_ind2]);

                        //check the reprojection error again to filter any outliers
                        double error1 = computeReprojectionError(transformedPose1, camID1, landmark, obs1);
                        double error2 = computeReprojectionError(transformedPose2, camID2, landmark, obs2);
                        double tri_angle_deg=0;
                        //compute angle
                        bool angleCheck = checkTriangulationAngle(const_cast<gtsam::Point3 &>(transformedPose1.translation()),
                                                                  const_cast<gtsam::Point3 &>(transformedPose2.translation()),
                                                                  landmark , tri_angle_deg);
                        VLOG(3)<<camID1<<"  "<<camID2<<"  "<<tri_angle_deg<<"," <<error1<<", "<<error2<<endl;
                        if(angleCheck and error1 <=4 and error2 <=4 ){

                            angles.at<double>(camID1,camID2) = tri_angle_deg;
                            tri_errors.at<double>(camID1,camID2) = error1 + error2;

                        }
                    }
                }

                // compare all the angles and choose the  largest one for insertion
                //first check the diagonal angles i.e intersecting component cameras between two LF frame
                double maxAngleSameCam = 0;
                double maxAngle = 0;
                int curKFCamID = -1, prevKFCamID = -1, curKFSameCamID = -1, prevKFSameCamID = -1;
                for(int camID1 = 0; camID1 < camArrayConfig->num_cams_ ; camID1++){

                    for(int camID2 = 0; camID2 < camArrayConfig->num_cams_ ; camID2++){

                        if(maxAngle < angles.at<double>(camID1,camID2)){
                            maxAngle = angles.at<double>(camID1,camID2);
                            prevKFCamID = camID1;
                            curKFCamID = camID2;
                        }

                        if(camID1 == camID2){
                            if( maxAngleSameCam < angles.at<double>(camID1,camID2)){
                                maxAngleSameCam = angles.at<double>(camID1,camID2);
                                prevKFSameCamID = camID1;
                                curKFSameCamID = camID2;
                            }

                        }
                    }

                }
                VLOG(3)<<"LID : "<<lids->at(i)<<" , max angle : "<<maxAngle<<" , max angle same cam: "<<maxAngleSameCam<<endl;

                if (maxAngleSameCam > 0 || maxAngle > 0 ) {
                    num_lms_filtered++;
                    lids_filtered.push_back(lids->at(i));
                    landmarks.push_back(landmark);
                    previous_KFs.push_back(lf1);
                    new_landmark_flags.push_back(true);
                    // choose the landmark, pose id and comp cam id, observations
                    if(maxAngleSameCam > 0){ /// For now we are only choosing the same index cameras
                        current_measurements.push_back(convertPoint2_CV2GTSAM(lf2->image_kps_undist[curKFSameCamID][im2->matchIndex[curKFSameCamID]]));
                        cur_compcam_ids.push_back(curKFSameCamID);
                        /// record the previous measurements and previous KF and comp cam ID
                        previous_measurements.push_back(convertPoint2_CV2GTSAM(lf1->image_kps_undist[prevKFSameCamID][im1->matchIndex[prevKFSameCamID]]));
                        previous_compcam_ids.push_back(prevKFSameCamID);

                    }
                    else{

                        current_measurements.push_back(convertPoint2_CV2GTSAM(lf2->image_kps_undist[curKFCamID][im2->matchIndex[curKFCamID]]));
                        cur_compcam_ids.push_back(curKFCamID);
                        /// record the previous measurements and previous KF and comp cam ID
                        previous_measurements.push_back(convertPoint2_CV2GTSAM(lf1->image_kps_undist[prevKFCamID][im1->matchIndex[prevKFCamID]]));
                        previous_compcam_ids.push_back(prevKFCamID);
                    }
                }

            }
            else{

                /// this is an existing landmark in the graph.
                /// Grab the last observation
                vector<int> facs = lmFactorRecord[lids->at(i)];
                assert(facs.size() != 0);

                /// Get the prev factor's frameID and componenet camera ID
                int prevFacStateID = facs.back();
                int prevFrameID = 0;
                string s = to_string(prevFacStateID);
                int CamID1 = stoi(&s.back());
                s.pop_back();
                if(!s.empty()){
                    prevFrameID = stoi(s);
                }
                VLOG(3)<<"prevFrameID: "<<prevFrameID<<", CamID1: "<<CamID1<<endl;
                MultiCameraFrame* lf1;
                IntraMatch* im1;
                vector<MultiCameraFrame *> KFs = l->KFs;
                vector<int> *feat_inds = &l->featInds;
                /// iterate over the KF list of this landmark to get
                /// the last KF with the frme ID equal to prevFarmeID
                auto kf_it = KFs.rbegin();
                auto feat_it = feat_inds->rbegin();
                kf_it++;
                feat_it++;
                for(; kf_it!=KFs.rend() && feat_it!=feat_inds->rend(); kf_it++, feat_it++){
                    if((*kf_it)->frameId == prevFrameID){
                        lf1 = *kf_it;
                        im1 = &lf1->intraMatches.at(*feat_it);
                    }
                }



                MultiCameraFrame* lf2 = currentFrame;  // same as l->KFs[numKFs-1]; // last KF
                IntraMatch* im2 = &lf2->intraMatches.at(l->featInds.back());

                /// check and make sure that the intramatch has an observation in CamID1 obtained form factor
                int kp_ind1 = im1->matchIndex[CamID1];
                assert(kp_ind1 != -1);
                gtsam::Pose3 transformedPose1 = calCompCamPose(lf1, CamID1);
                gtsam::Point2 obs1 = convertPoint2_CV2GTSAM(lf1->image_kps_undist[CamID1][kp_ind1]);
                double error1 = computeReprojectionError(transformedPose1, CamID1, landmark, obs1);

                Mat angles = Mat::zeros(1,lf2->num_cams_, CV_64FC1);
                Mat tri_errors = Mat::zeros(1,lf2->num_cams_, CV_64FC1);

                ///In the current frame see if we have the matching camera
                for (int camID2 = 0; camID2 < lf2->num_cams_ ; camID2++){
                    int kp_ind2 = im2->matchIndex[camID2];
                    if(kp_ind2 == -1)
                        continue;

                    // take input of the LF frame pose and comp cam id to compute the comp cam pose
                    gtsam::Pose3 transformedPose2 = calCompCamPose(lf2, camID2);
                    gtsam::Point2 obs2  = convertPoint2_CV2GTSAM(lf2->image_kps_undist[camID2][kp_ind2]);

                    //check the reprojection error again to filter any outliers
                    double error2 = computeReprojectionError(transformedPose2, camID2, landmark, obs2);
                    double tri_angle_deg=0;
                    //compute angle
                    bool angleCheck = checkTriangulationAngle(const_cast<gtsam::Point3 &>(transformedPose1.translation()),
                                                              const_cast<gtsam::Point3 &>(transformedPose2.translation()),
                                                              landmark , tri_angle_deg);
                    VLOG(3)<<CamID1<<"  "<<camID2<<"  "<<tri_angle_deg<<"," <<error1<<", "<<error2<<endl;
                    if(angleCheck and error1 <=4 and error2 <=4 ){

                        angles.at<double>(0,camID2) = tri_angle_deg;
                        tri_errors.at<double>(0,camID2) = error1 + error2;

                    }
                }



                // compare all the angles and choose the  largest one for insertion
                //first check the diagonal angles i.e intersecting component cameras between two LF frame
                double maxAngleSameCam = angles.at<double>(0,CamID1);
                double maxAngle = 0;
                int curKFCamID = -1;
                for(int camID2 = 0; camID2 < camArrayConfig->num_cams_ ; camID2++){
                    if(maxAngle < angles.at<double>(0,camID2)){
                        maxAngle = angles.at<double>(0,camID2);
                        curKFCamID = camID2;
                    }

                }

                VLOG(3)<<"LID : "<<lids->at(i)<<" , max angle : "<<maxAngle<<" , max angle same cam: "<<maxAngleSameCam<<endl;
                if (maxAngleSameCam > 0 || maxAngle > 0 ) {
                    num_lms_filtered++;
                    lids_filtered.push_back(lids->at(i));
                    landmarks.push_back(landmark);
                    previous_KFs.push_back(lf1);
                    new_landmark_flags.push_back(false);
                    // choose the landmark, pose id and comp cam id, observations
                    if(maxAngleSameCam > 0){ /// For now we are only choosing the same index cameras
                        current_measurements.push_back(convertPoint2_CV2GTSAM(lf2->image_kps_undist[CamID1][im2->matchIndex[CamID1]]));
                        cur_compcam_ids.push_back(CamID1);
                        /// record the previous measurements and previous KF and comp cam ID
                        previous_measurements.push_back(convertPoint2_CV2GTSAM(lf1->image_kps_undist[CamID1][im1->matchIndex[CamID1]]));
                        previous_compcam_ids.push_back(CamID1);

                    }
                    else{

                        current_measurements.push_back(convertPoint2_CV2GTSAM(lf2->image_kps_undist[curKFCamID][im2->matchIndex[curKFCamID]]));
                        cur_compcam_ids.push_back(curKFCamID);
                        /// record the previous measurements and previous KF and comp cam ID
                        previous_measurements.push_back(convertPoint2_CV2GTSAM(lf1->image_kps_undist[CamID1][im1->matchIndex[CamID1]]));
                        previous_compcam_ids.push_back(CamID1);
                    }
                }


            }


        }
    }


    // VLOG(3)<<"Backend Landmarks size: "<<landmarks.size()<<endl;
    if(landmarks.size() < 15){
        return false;
    }

    /// check if this is the first frame we are inserting
    /// if it is, we have to add prior to the pose and a landmark
    if(currentEstimate.empty() && initialEstimate.empty()) {

        VLOG(3)<<"Went inside GRAPH==0"<<endl;

        Landmark* l = map->getLandmark(lids_filtered.at(0));
        int numKFs = l->KFs.size();
        MultiCameraFrame* prevLF=  l->KFs[numKFs - 2];

        assert(previous_KFs.at(0)->frameId == prevLF->frameId);
        // Concatenate both pose id and the cam id
        // Convert the concatenated string
        // to integer

        /// Add a prior on pose x0
        gtsam::noiseModel::Diagonal::shared_ptr poseNoise = gtsam::noiseModel::Diagonal::Sigmas
                ((gtsam::Vector(6)<< gtsam::Vector3::Constant(0.001),gtsam::Vector3::Constant(0.001)).finished());
                //((gtsam::Vector(6)<< gtsam::Vector3::Constant(0.3),gtsam::Vector3::Constant(0.1)).finished());

        graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', prevLF->frameId), convertPose3_CV2GTSAM(prevLF->pose), poseNoise));
        VLOG(3)<<"Inserted prior for the state x"<<prevLF->frameId<<endl;
        /// LANDMARK PRIOR
        addLandmarkPrior(lids_filtered.at(0), landmarks[0]);
    }


    /// create graph edges
    /// insert initial values for the variables
    for (int i = 0; i < landmarks.size(); ++i) {
        ///Previous state ID
        Landmark* l = map->getLandmark(lids_filtered.at(i));
        MultiCameraFrame* prev_LF = previous_KFs.at(i);

        string s1 = to_string(current_pose_id);
        string s2 = to_string(cur_compcam_ids[i]);
        // Concatenate both pose id and the cam id
        string s = s1 + s2;
        // Convert the concatenated string
        // to integer
        int curStateIDComp = stoi(s);


        s1 = to_string( prev_LF->frameId);
        s2 = to_string(previous_compcam_ids[i]);
        // Concatenate both pose id and the cam id
        s = s1 + s2;
        // Convert the concatenated string
        // to integer
        int prevStateIDComp = stoi(s);


        ///  Current State id
         int curStateID = current_pose_id;

        int prevStateID = prev_LF->frameId;

        if(!(currentEstimate.exists(gtsam::Symbol('x', curStateID)) || initialEstimate.exists(gtsam::Symbol('x', curStateID)))){
            /// Initial estimate for the current pose.
            initialEstimate.insert(gtsam::Symbol('x', curStateID), convertPose3_CV2GTSAM(currentFrame->pose));
            /// state factor record
            xFactorRecord[curStateID] = vector<int>();

            VLOG(3)<<"Inserting Initial Estimate for x"<<curStateID<<endl;
        }

        if(new_landmark_flags[i]){
            /// for each measurement add a projection factor
            if(!(currentEstimate.exists(gtsam::Symbol('x', prevStateID)) || initialEstimate.exists(gtsam::Symbol('x', prevStateID)))){
                // Initial estimate for the current pose.
                initialEstimate.insert(gtsam::Symbol('x', prevStateID), convertPose3_CV2GTSAM(prev_LF->pose));
                /// state factor record
                xFactorRecord[prevStateID] = vector<int>();
                VLOG(3)<<"Inserting Initial Estimate for x"<<prevStateID<<endl;
            }

            // Insert projection factor with previous kf to factor graph
            //VLOG(3)<<"measurement from prev frame: "<<previous_measurements[i].x()<<" "<<previous_measurements[i].y()<<endl;
            graph.push_back(gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>
                                    (previous_measurements[i], huberModel, gtsam::Symbol('x', prevStateID),
                                     gtsam::Symbol('l', lids_filtered.at(i)), K[previous_compcam_ids[i]], convertPose3_CV2GTSAM(Rt_mats_[previous_compcam_ids[i]])));
            /// state factor record
            xFactorRecord[prevStateID].push_back(lids_filtered.at(i));
            lmFactorRecord[lids_filtered.at(i)] = vector<int>({prevStateIDComp});

            initialEstimate.insert(gtsam::Symbol('l', lids_filtered.at(i)), landmarks[i]);
            VLOG(3)<<"Inserting Initial Estimate for l"<<lids_filtered.at(i)<<endl;
        }

        // insert projection factor with current frame to factor graph
        graph.push_back(gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>
                                (current_measurements[i], huberModel, gtsam::Symbol('x', curStateID),
                                 gtsam::Symbol('l', lids_filtered.at(i)), K[cur_compcam_ids[i]], convertPose3_CV2GTSAM(Rt_mats_[cur_compcam_ids[i]])));
        /// state factor record
        xFactorRecord[curStateID].push_back(lids_filtered.at(i));
        lmFactorRecord[lids_filtered.at(i)].push_back(curStateIDComp);


    }

    VLOG(3)<<"graph size: "<<graph.size()<<endl;
    if (windowCounter%windowSize != 0)
        ret = false;
    windowCounter++;
    VLOG(3)<<"add keyframe optimize flag: "<<ret<<endl;
    return ret;

}

bool Backend::addKeyFrameMultiLatest() {

    MultiCameraFrame *currentFrame = frontEnd->currentFrame;
    GlobalMap *map = frontEnd->map;
    bool ret = true;

    /// Variable declaration
    std::map<int, set<int> > insertedRigid;
    int num_lms_filtered=0;
    vector<int> lids_filtered;
    vector<gtsam::Point3> landmarks;
    vector<gtsam::Point2> current_measurements;
    vector<gtsam::Point2> previous_measurements;
    vector<int>  previous_compcam_ids, cur_compcam_ids;
    vector<MultiCameraFrame*> previous_KFs;
    vector<bool> new_landmark_flags;

    vector<IntraMatch> *intramatches = &currentFrame->intraMatches;
    vector<int> *lids = &currentFrame->lIds;
    int current_pose_id = currentFrame->frameId;

    // browse through all intramatches of current frame
    for (int i = 0; i < intramatches->size(); ++i) {
        // if this intra match is a landmark i.e it is tracked in the current frame and is a triangulated inlier
        if (lids->at(i) != -1 ) {

            //get the reference to landmark and its 3D point
            Landmark* l = map->getLandmark(lids->at(i));
            gtsam::Point3 landmark = convertPoint3_CV2GTSAM(l->pt3D);

            /// if the landmarek is new i.e it is not present in current estimates or initial estimates
            if(!currentEstimate.exists(gtsam::Symbol('l', lids->at(i))) and
               !initialEstimate.exists(gtsam::Symbol('l', lids->at(i))))
            {
                /// make sure it is seen only in two KFs
                //assert(l->KFs.size() == 2);

                /// Get the previous and current multiframe of the landmark and its intramatches
                int numKFs = l->KFs.size();
                MultiCameraFrame* lf1 = l->KFs[numKFs-2]; // previous KF
                MultiCameraFrame* lf2 = currentFrame;  // same as l->KFs[numKFs-1]; // last KF
                IntraMatch* im1 = &lf1->intraMatches.at(l->featInds[numKFs-2]);
                IntraMatch* im2 = &lf2->intraMatches.at(l->featInds[numKFs-1]);

                Mat angles = Mat::zeros(lf1->num_cams_,lf2->num_cams_, CV_64FC1);
                Mat tri_errors = Mat::zeros(lf1->num_cams_,lf2->num_cams_, CV_64FC1);
                VLOG(3)<<"LID : "<<lids->at(i)<<", PT: "<<landmark.x()<<","<<landmark.y()<<","<<landmark.z()<<endl;
                VLOG(3)<<"Cam1 cam2 angle error1 error2"<<endl;
                ///check all combinations of rays and get the triangulation angles and reprojection errors
                for (int camID1 = 0; camID1 < lf1->num_cams_ ; camID1++){
                    int kp_ind1 = im1->matchIndex[camID1];
                    if(kp_ind1 == -1)
                        continue;
                    for (int camID2 = 0; camID2 < lf2->num_cams_ ; camID2++){
                        int kp_ind2 = im2->matchIndex[camID2];
                        if(kp_ind2 == -1)
                            continue;

                        gtsam::Pose3 transformedPose1 = calCompCamPose(lf1, camID1); // take input of the LF frame pose and comp cam id to compute the comp cam pose
                        gtsam::Pose3 transformedPose2 = calCompCamPose(lf2, camID2);
                        gtsam::Point2 obs1 = convertPoint2_CV2GTSAM(lf1->image_kps_undist[camID1][kp_ind1]);
                        gtsam::Point2 obs2  = convertPoint2_CV2GTSAM(lf2->image_kps_undist[camID2][kp_ind2]);

                        //check the reprojection error again to filter any outliers
                        double error1 = computeReprojectionError(transformedPose1, camID1, landmark, obs1);
                        double error2 = computeReprojectionError(transformedPose2, camID2, landmark, obs2);
                        double tri_angle_deg=0;
                        //compute angle
                        bool angleCheck = checkTriangulationAngle(const_cast<gtsam::Point3 &>(transformedPose1.translation()),
                                                                  const_cast<gtsam::Point3 &>(transformedPose2.translation()),
                                                                  landmark , tri_angle_deg);
                        VLOG(3)<<camID1<<"  "<<camID2<<"  "<<tri_angle_deg<<"," <<error1<<", "<<error2<<endl;
                        if(angleCheck and error1 <=4 and error2 <=4 ){

                            angles.at<double>(camID1,camID2) = tri_angle_deg;
                            tri_errors.at<double>(camID1,camID2) = error1 + error2;

                        }
                    }
                }

                // compare all the angles and choose the  largest one for insertion
                //first check the diagonal angles i.e intersecting component cameras between two LF frame
                double maxAngleSameCam = 0;
                double maxAngle = 0;
                int curKFCamID = -1, prevKFCamID = -1, curKFSameCamID = -1, prevKFSameCamID = -1;
                for(int camID1 = 0; camID1 < camArrayConfig->num_cams_ ; camID1++){

                    for(int camID2 = 0; camID2 < camArrayConfig->num_cams_ ; camID2++){

                        if(maxAngle < angles.at<double>(camID1,camID2)){
                            maxAngle = angles.at<double>(camID1,camID2);
                            prevKFCamID = camID1;
                            curKFCamID = camID2;
                        }

                        if(camID1 == camID2){
                            if( maxAngleSameCam < angles.at<double>(camID1,camID2)){
                                maxAngleSameCam = angles.at<double>(camID1,camID2);
                                prevKFSameCamID = camID1;
                                curKFSameCamID = camID2;
                            }

                        }
                    }

                }
                VLOG(3)<<"LID : "<<lids->at(i)<<" , max angle : "<<maxAngle<<" , max angle same cam: "<<maxAngleSameCam<<endl;

                if (maxAngleSameCam > 0 || maxAngle > 0 ) {
                    num_lms_filtered++;
                    lids_filtered.push_back(lids->at(i));
                    landmarks.push_back(landmark);
                    previous_KFs.push_back(lf1);
                    new_landmark_flags.push_back(true);
                    // choose the landmark, pose id and comp cam id, observations
                    if(maxAngleSameCam > 0){ /// For now we are only choosing the same index cameras
                        current_measurements.push_back(convertPoint2_CV2GTSAM(lf2->image_kps_undist[curKFSameCamID][im2->matchIndex[curKFSameCamID]]));
                        cur_compcam_ids.push_back(curKFSameCamID);
                        /// record the previous measurements and previous KF and comp cam ID
                        previous_measurements.push_back(convertPoint2_CV2GTSAM(lf1->image_kps_undist[prevKFSameCamID][im1->matchIndex[prevKFSameCamID]]));
                        previous_compcam_ids.push_back(prevKFSameCamID);

                    }
                    else{

                        current_measurements.push_back(convertPoint2_CV2GTSAM(lf2->image_kps_undist[curKFCamID][im2->matchIndex[curKFCamID]]));
                        cur_compcam_ids.push_back(curKFCamID);
                        /// record the previous measurements and previous KF and comp cam ID
                        previous_measurements.push_back(convertPoint2_CV2GTSAM(lf1->image_kps_undist[prevKFCamID][im1->matchIndex[prevKFCamID]]));
                        previous_compcam_ids.push_back(prevKFCamID);
                    }
                }

            }
            else{

                /// this is an existing landmark in the graph.
                /// Grab the last observation
                vector<int> facs = lmFactorRecord[lids->at(i)];
                assert(facs.size() != 0);

                /// Get the prev factor's frameID and componenet camera ID
                int prevFacStateID = facs.back();
                int prevFrameID = 0;
                string s = to_string(prevFacStateID);
                VLOG(3)<<s<<endl;
                int CamID1 = stoi(&s.back());
                s.pop_back();
                if(!s.empty()){
                    prevFrameID = stoi(s);
                }
                VLOG(3)<<"prevFrameID: "<<prevFrameID<<", CamID1: "<<CamID1<<endl;
                MultiCameraFrame* lf1;
                IntraMatch* im1;
                vector<MultiCameraFrame *> KFs = l->KFs;
                vector<int> *feat_inds = &l->featInds;
                /// iterate over the KF list of this landmark to get
                /// the last KF with the frme ID equal to prevFarmeID
                auto kf_it = KFs.rbegin();
                auto feat_it = feat_inds->rbegin();
                kf_it++;
                feat_it++;
                for(; kf_it!=KFs.rend() && feat_it!=feat_inds->rend(); kf_it++, feat_it++){
                    if((*kf_it)->frameId == prevFrameID){
                        lf1 = *kf_it;
                        im1 = &lf1->intraMatches.at(*feat_it);
                    }
                }



                MultiCameraFrame* lf2 = currentFrame;  // same as l->KFs[numKFs-1]; // last KF
                IntraMatch* im2 = &lf2->intraMatches.at(l->featInds.back());

                /// check and make sure that the intramatch has an observation in CamID1 obtained form factor
                int kp_ind1 = im1->matchIndex[CamID1];
                assert(kp_ind1 != -1);
                gtsam::Pose3 transformedPose1 = calCompCamPose(lf1, CamID1);
                gtsam::Point2 obs1 = convertPoint2_CV2GTSAM(lf1->image_kps_undist[CamID1][kp_ind1]);
                double error1 = computeReprojectionError(transformedPose1, CamID1, landmark, obs1);

                Mat angles = Mat::zeros(1,lf2->num_cams_, CV_64FC1);
                Mat tri_errors = Mat::zeros(1,lf2->num_cams_, CV_64FC1);

                ///In the current frame see if we have the matching camera
                for (int camID2 = 0; camID2 < lf2->num_cams_ ; camID2++){
                    int kp_ind2 = im2->matchIndex[camID2];
                    if(kp_ind2 == -1)
                        continue;

                    // take input of the LF frame pose and comp cam id to compute the comp cam pose
                    gtsam::Pose3 transformedPose2 = calCompCamPose(lf2, camID2);
                    gtsam::Point2 obs2  = convertPoint2_CV2GTSAM(lf2->image_kps_undist[camID2][kp_ind2]);

                    //check the reprojection error again to filter any outliers
                    double error2 = computeReprojectionError(transformedPose2, camID2, landmark, obs2);
                    double tri_angle_deg=0;
                    //compute angle
                    bool angleCheck = checkTriangulationAngle(const_cast<gtsam::Point3 &>(transformedPose1.translation()),
                                                              const_cast<gtsam::Point3 &>(transformedPose2.translation()),
                                                              landmark , tri_angle_deg);
                    VLOG(3)<<CamID1<<"  "<<camID2<<"  "<<tri_angle_deg<<"," <<error1<<", "<<error2<<endl;
                    if(angleCheck and error1 <=4 and error2 <=4 ){

                        angles.at<double>(0,camID2) = tri_angle_deg;
                        tri_errors.at<double>(0,camID2) = error1 + error2;

                    }
                }



                // compare all the angles and choose the  largest one for insertion
                //first check the diagonal angles i.e intersecting component cameras between two LF frame
                double maxAngleSameCam = angles.at<double>(0,CamID1);
                double maxAngle = 0;
                int curKFCamID = -1;
                for(int camID2 = 0; camID2 < camArrayConfig->num_cams_ ; camID2++){
                    if(maxAngle < angles.at<double>(0,camID2)){
                        maxAngle = angles.at<double>(0,camID2);
                        curKFCamID = camID2;
                    }

                }

                VLOG(3)<<"LID : "<<lids->at(i)<<" , max angle : "<<maxAngle<<" , max angle same cam: "<<maxAngleSameCam<<endl;
                if (maxAngleSameCam > 0 || maxAngle > 0 ) {
                    num_lms_filtered++;
                    lids_filtered.push_back(lids->at(i));
                    landmarks.push_back(landmark);
                    previous_KFs.push_back(lf1);
                    new_landmark_flags.push_back(false);
                    // choose the landmark, pose id and comp cam id, observations
                    if(maxAngleSameCam > 0){ /// For now we are only choosing the same index cameras
                        current_measurements.push_back(convertPoint2_CV2GTSAM(lf2->image_kps_undist[CamID1][im2->matchIndex[CamID1]]));
                        cur_compcam_ids.push_back(CamID1);
                        /// record the previous measurements and previous KF and comp cam ID
                        previous_measurements.push_back(convertPoint2_CV2GTSAM(lf1->image_kps_undist[CamID1][im1->matchIndex[CamID1]]));
                        previous_compcam_ids.push_back(CamID1);

                    }
                    else{

                        current_measurements.push_back(convertPoint2_CV2GTSAM(lf2->image_kps_undist[curKFCamID][im2->matchIndex[curKFCamID]]));
                        cur_compcam_ids.push_back(curKFCamID);
                        /// record the previous measurements and previous KF and comp cam ID
                        previous_measurements.push_back(convertPoint2_CV2GTSAM(lf1->image_kps_undist[CamID1][im1->matchIndex[CamID1]]));
                        previous_compcam_ids.push_back(CamID1);
                    }
                }


            }


        }
    }


    // VLOG(3)<<"Backend Landmarks size: "<<landmarks.size()<<endl;
    if(landmarks.size() < 15){

        return false;
    }

    /// check if this is the first frame we are inserting
    /// if it is, we have to add prior to the pose and a landmark
    if(currentEstimate.empty() && initialEstimate.empty()) {

        VLOG(3)<<"Went inside GRAPH==0"<<endl;

        Landmark* l = map->getLandmark(lids_filtered.at(0));
        int numKFs = l->KFs.size();
        MultiCameraFrame* prevLF=  l->KFs[numKFs - 2];
        gtsam::Pose3 previous_pose = calCompCamPose(prevLF , previous_compcam_ids[0]);

        assert(previous_KFs.at(0)->frameId == prevLF->frameId);
        // Concatenate both pose id and the cam id
        // Convert the concatenated string
        // to integer
        int prevStateID = stoi(to_string( prevLF->frameId) + to_string(previous_compcam_ids[0]));

        /// Add a prior on pose x0
        gtsam::noiseModel::Diagonal::shared_ptr poseNoise = gtsam::noiseModel::Diagonal::Sigmas
                ((gtsam::Vector(6)<< gtsam::Vector3::Constant(0.3),gtsam::Vector3::Constant(0.1)).finished());
        graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', prevStateID), previous_pose, poseNoise));
        VLOG(3)<<"Inserted prior for the state x"<<prevStateID<<endl;
        /// LANDMARK PRIOR
        addLandmarkPrior(lids_filtered.at(0), landmarks[0]);
    }


    /// create graph edges
    /// insert initial values for the variables
    for (int i = 0; i < landmarks.size(); ++i) {
        ///Previous state ID
        Landmark* l = map->getLandmark(lids_filtered.at(i));
        MultiCameraFrame* prev_LF = previous_KFs.at(i);

        ///  Current State id
        string s1 = to_string(current_pose_id);
        string s2 = to_string(cur_compcam_ids[i]);
        // Concatenate both pose id and the cam id
        string s = s1 + s2;
        // Convert the concatenated string
        // to integer
        int curStateID = stoi(s);


        s1 = to_string( prev_LF->frameId);
        s2 = to_string(previous_compcam_ids[i]);
        // Concatenate both pose id and the cam id
        s = s1 + s2;
        // Convert the concatenated string
        // to integer
        int prevStateID = stoi(s);


        if(!(currentEstimate.exists(gtsam::Symbol('x', curStateID)) || initialEstimate.exists(gtsam::Symbol('x', curStateID)))){
            /// Initial estimate for the current pose.
            gtsam::Pose3 current_pose = calCompCamPose(currentFrame , cur_compcam_ids[i]);
            initialEstimate.insert(gtsam::Symbol('x', curStateID), current_pose);
            /// state factor record
            xFactorRecord[curStateID] = vector<int>();

            VLOG(3)<<"Inserting Initial Estimate for x"<<curStateID<<endl;
            /*
            this mode is deprecated. 
            current code supports MULTI mode
            if(backendType == MULTI_RIGID){
                /// if the pose is being inserted for the firt time
                if(insertedRigid.find(current_pose_id) == insertedRigid.end())
                    insertedRigid[current_pose_id] = set<int>({ cur_compcam_ids[i]});
                else
                    insertedRigid[current_pose_id].insert( cur_compcam_ids[i]);
            }*/


        }

        if(new_landmark_flags[i]){
            /// for each measurement add a projection factor
            if(!(currentEstimate.exists(gtsam::Symbol('x', prevStateID)) || initialEstimate.exists(gtsam::Symbol('x', prevStateID)))){
                // Initial estimate for the current pose.
                gtsam::Pose3 previous_pose = calCompCamPose(prev_LF , previous_compcam_ids[i]);
                initialEstimate.insert(gtsam::Symbol('x', prevStateID), previous_pose);
                /// state factor record
                xFactorRecord[prevStateID] = vector<int>();
                VLOG(3)<<"Inserting Initial Estimate for x"<<prevStateID<<endl;
                /*
                this mode is deprecated. 
                current code supports MULTI mode
                if(backendType == MULTI_RIGID){
                    /// if the pose is being inserted for the firt time
                    if(insertedRigid.find( prev_LF->frameId) == insertedRigid.end())
                        insertedRigid[ prev_LF->frameId] = set<int>({ previous_compcam_ids[i]});
                    else
                        insertedRigid[ prev_LF->frameId].insert( previous_compcam_ids[i]);
                }*/

            }

            // Insert projection factor with previous kf to factor graph
            //VLOG(3)<<"measurement from prev frame: "<<previous_measurements[i].x()<<" "<<previous_measurements[i].y()<<endl;
            graph.push_back(gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>
                                    (previous_measurements[i], measurementNoise, gtsam::Symbol('x', prevStateID),
                                     gtsam::Symbol('l', lids_filtered.at(i)), K[previous_compcam_ids[i]]));
            /// state factor record
            xFactorRecord[prevStateID].push_back(lids_filtered.at(i));
            lmFactorRecord[lids_filtered.at(i)] = vector<int>({prevStateID});
            //record the lm factors
            /*if(lmFactorRecord.find(lids_filtered.at(i)) != lmFactorRecord.end()){
                lmFactorRecord[lids_filtered.at(i)].push_back(prevStateID);
            }
            else{
                lmFactorRecord[lids_filtered.at(i)] = vector<int>({prevStateID});
            }*/

            initialEstimate.insert(gtsam::Symbol('l', lids_filtered.at(i)), landmarks[i]);
            VLOG(3)<<"Inserting Initial Estimate for l"<<lids_filtered.at(i)<<endl;
        }

        // insert projection factor with current frame to factor graph
        graph.push_back(gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>
                                (current_measurements[i], measurementNoise, gtsam::Symbol('x', curStateID),
                                 gtsam::Symbol('l', lids_filtered.at(i)), K[cur_compcam_ids[i]]));
        /// state factor record
        xFactorRecord[curStateID].push_back(lids_filtered.at(i));
        lmFactorRecord[lids_filtered.at(i)].push_back(curStateID);
        //record the lm factors
        /*if(lmFactorRecord.find(lids_filtered.at(i)) != lmFactorRecord.end()){
            lmFactorRecord[lids_filtered.at(i)].push_back(curStateID);
        }
        else{
            lmFactorRecord[lids_filtered.at(i)] = vector<int>({curStateID});
        }*/

    }

    /*
    deprecated - current version supports backendtype = MULTI
    if(backendType == MULTI_RIGID){
        gtsam::noiseModel::Diagonal::shared_ptr  betweenNoise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6)<<gtsam::Vector3::Constant(0.001), gtsam::Vector3::Constant(0.001)).finished());
        std::map<int, std::set<int>>::iterator it;
        for(it = insertedRigid.begin() ; it != insertedRigid.end() ; ++it){
            int poseid = it->first;
            set<int> compcamids = it->second;
            set<int>::iterator it_set = compcamids.begin();
            int comp_cam_id_prev = *it_set;
            ++it_set;
            for( ; it_set != compcamids.end() ; ++it_set){

                Mat poseDiff = Mat::eye(4,4, CV_64FC1);
                for(int ii=comp_cam_id_prev+1; ii <= *it_set ; ii++ ){
                    poseDiff =  poseDiff * Rt_mats_kalib_[ii];
                }
                gtsam::Pose3 betweenPose = convertPose3_CV2GTSAM(poseDiff);
                int prev_stateid = stoi(to_string(poseid) + to_string(comp_cam_id_prev));
                int state_id = stoi(to_string(poseid) + to_string(*it_set));
                graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(gtsam::Symbol('x', prev_stateid), gtsam::Symbol('x', state_id),  betweenPose, betweenNoise);
                VLOG(3) <<"Inserted Between factor for PoseId: "<<poseid<<" Between: "<<comp_cam_id_prev<<" and "<<*it_set<<endl;
                comp_cam_id_prev = *it_set;
            }

        }

    }
    */

    VLOG(3)<<"graph size: "<<graph.size()<<endl;
    if (windowCounter%windowSize != 0)
        ret = false;
    VLOG(3)<<"add keyframe optimize flag: "<<ret<<endl;
    windowCounter++;
    return ret;

}

void Backend::getCamIDs_Angles(Landmark* l, int KFID1, int KFID2 , Mat& angles , vector<int>& prevRaysCamIds,
                               vector<int>& curRaysCamIDs, int& maxAnglePrevCamID, int& maxAngleCurCamID){

    MultiCameraFrame* lf1 = l->KFs[KFID1]; // previous KF
    MultiCameraFrame* lf2 = l->KFs[KFID2];  // same as l->KFs[numKFs-1]; // last KF
    IntraMatch* im1 = &lf1->intraMatches.at(l->featInds[KFID1]);
    IntraMatch* im2 = &lf2->intraMatches.at(l->featInds[KFID2]);
    gtsam::Point3 landmark = convertPoint3_CV2GTSAM(l->pt3D);

    angles = Mat::zeros(lf1->num_cams_,lf2->num_cams_, CV_64FC1);
    Mat tri_errors = Mat::zeros(lf1->num_cams_,lf2->num_cams_, CV_64FC1);
    VLOG(3)<<"LID : "<<l->lId<<", PT: "<<landmark.x()<<","<<landmark.y()<<","<<landmark.z()<<endl;
    VLOG(3)<<"Cam1 cam2 angle error1 error2"<<endl;

    ///check all combinations of rays and get the triangulation angles and reprojection errors
    double maxAngle = 0;
    maxAngleCurCamID = -1;
    maxAnglePrevCamID = -1;
    if(prevRaysCamIds.size() == 0){
        for (int camID1 = 0; camID1 < lf1->num_cams_ ; camID1++) {
            int kp_ind1 = im1->matchIndex[camID1];
            if (kp_ind1 == -1)
                continue;
            prevRaysCamIds.push_back(camID1);
        }
    }

    int firstcID = prevRaysCamIds[0];
    for (auto& camID1 : prevRaysCamIds){
        int kp_ind1 = im1->matchIndex[camID1];

        for (int camID2 = 0; camID2 < lf2->num_cams_ ; camID2++){
            int kp_ind2 = im2->matchIndex[camID2];
            if(kp_ind2 == -1)
                continue;
            if(camID1 == firstcID)
                curRaysCamIDs.push_back(camID2);
            gtsam::Pose3 transformedPose1 = calCompCamPose(lf1, camID1); // take input of the LF frame pose and comp cam id to compute the comp cam pose
            gtsam::Pose3 transformedPose2 = calCompCamPose(lf2, camID2);
            gtsam::Point2 obs1 = convertPoint2_CV2GTSAM(lf1->image_kps_undist[camID1][kp_ind1]);
            gtsam::Point2 obs2  = convertPoint2_CV2GTSAM(lf2->image_kps_undist[camID2][kp_ind2]);

            //check the reprojection error again to filter any outliers
            double error1 = computeReprojectionError(transformedPose1, camID1, landmark, obs1);
            double error2 = computeReprojectionError(transformedPose2, camID2, landmark, obs2);
            double tri_angle_deg=0;
            //compute angle
            bool angleCheck = checkTriangulationAngle(const_cast<gtsam::Point3 &>(transformedPose1.translation()),
                                                      const_cast<gtsam::Point3 &>(transformedPose2.translation()),
                                                      landmark , tri_angle_deg);
            VLOG(3)<<camID1<<"  "<<camID2<<"  "<<tri_angle_deg<<"," <<error1<<", "<<error2<<endl;
            if(angleCheck and error1 <=4 and error2 <=4 ){

                angles.at<double>(camID1,camID2) = tri_angle_deg;
                tri_errors.at<double>(camID1,camID2) = error1 + error2;

                if(maxAngle < angles.at<double>(camID1,camID2)){
                    maxAngle = angles.at<double>(camID1,camID2);
                    maxAnglePrevCamID = camID1;
                    maxAngleCurCamID = camID2;
                }

            }
        }
    }

}


void Backend::insertLandmarkInGraph(Landmark *l, int prevKFID, int curKFID, bool newlm, vector<int> previous_compcam_ids,
                               vector<gtsam::Point2> previous_measurements, vector<int> cur_compcam_ids,
                               vector<gtsam::Point2> current_measurements, vector<int> prevKPOctaves,
                               vector<int> curKPOctaves) {
    ///Previous state ID
    MultiCameraFrame* prev_LF = l->KFs[prevKFID];
    MultiCameraFrame* currentFrame = l->KFs[curKFID];

    int prevStateID = prev_LF->kfID;

    if(prev_Kf == -1){
        prev_Kf = prevStateID;
    }

    int curStateID = currentFrame->kfID;





    int lid = l->lId;

    newTimeStamps[gtsam::Symbol('l', lid)] = (windowCounter-1);

//    VLOG(0) << "prev stateid " << prevStateID;
//    VLOG(0) << "curr state id " << curStateID;

    // VLOG(0) << "exists in current estimate? " << currentEstimate.exists(gtsam::Symbol('x', curStateID)) << endl;
    // VLOG(0) << "exists in initial estimate? " << initialEstimate.exists(gtsam::Symbol('x', curStateID)) << endl;

  if(!(currentEstimate.exists(gtsam::Symbol('x', curStateID)) || initialEstimate.exists(gtsam::Symbol('x', curStateID)))){




      cv::Mat tmp_ = currentFrame->pose * frontEnd->Tbc_cv.inv();
      gtsam::Pose3 current_pose = convertPose3_CV2GTSAM(tmp_);
      //gtsam::Pose3 current_pose = convertPose3_CV2GTSAM(currentFrame->pose);
      /// Initial estimate for the current pose.
      initialEstimate.insert(gtsam::Symbol('x', curStateID), current_pose);
      /// state factor record
      xFactorRecord[curStateID] = vector<int>();
      //This will be used only in case of fixedlag smoothing
      newTimeStamps[gtsam::Symbol('x', curStateID)] = (windowCounter-1);
       VLOG(0)<<"Inserting Initial Estimate for x"<<curStateID<<endl;
      VLOG(1)<<"Current pose being added: "<<current_pose<<endl;

    //    exit(0);

  }


    if(newlm){

        /// for each measurement add a projection factor
       if(!(currentEstimate.exists(gtsam::Symbol('x', prevStateID)) || initialEstimate.exists(gtsam::Symbol('x', prevStateID)))){
           // Initial estimate for the current pose.
           cv::Mat tmp_ = prev_LF->pose * frontEnd->Tbc_cv.inv();
           gtsam::Pose3 previous_pose = convertPose3_CV2GTSAM(tmp_);
           //gtsam::Pose3 previous_pose = convertPose3_CV2GTSAM(prev_LF->pose);
           initialEstimate.insert(gtsam::Symbol('x', prevStateID), previous_pose);
           /// state factor record
           xFactorRecord[prevStateID] = vector<int>();
           newTimeStamps[gtsam::Symbol('x', prevStateID)] = (windowCounter-1);
           VLOG(1)<<"Inserting Initial Estimate for x"<<prevStateID<<endl;
           VLOG(1) << " Pose added is " << previous_pose << endl;
           VLOG(1)  << "PREV KF" << prev_Kf << " and " << "prevStateID" << prevStateID << endl;
//            exit(0);
       }


        int c=0;
        for(auto& prevCompId : previous_compcam_ids){

            // Insert projection factor with previous kf to factor graph
            //VLOG(3)<<"measurement from prev frame: "<<previous_measurements[i].x()<<" "<<previous_measurements[i].y()<<endl;
            noiseModel::Robust::shared_ptr huberModel1;
            cv::Mat tmp_cv = frontEnd->Tbc_cv*Rt_mats_[prevCompId];
            if(!prevKPOctaves.empty()){
                auto measurementNoise1 =
                        noiseModel::Isotropic::Sigma(2, frontEnd->orBextractor->GetInverseScaleSigmaSquares()[ prevKPOctaves[c]]);
                huberModel1 = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(
                        sqrt(5.991)), measurementNoise1);
                graph.push_back(gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>
                                        (previous_measurements[c], huberModel1, gtsam::Symbol('x', prevStateID),
                                         gtsam::Symbol('l', lid), K[prevCompId], convertPose3_CV2GTSAM(tmp_cv) ));
            }
            else{
                graph.push_back(gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>
                                        (previous_measurements[c], huberModel, gtsam::Symbol('x', prevStateID),
                                         gtsam::Symbol('l', lid), K[prevCompId], convertPose3_CV2GTSAM(tmp_cv) ));
            }
            if(lmFactorRecordMulti.find(lid) != lmFactorRecordMulti.end() ){
                std::map<int, vector<int>> rec;
                rec[prevKFID] = vector<int>({prevCompId});
                lmFactorRecordMulti[lid] = rec;
            }
            else{
                std::map<int, vector<int>> rec = lmFactorRecordMulti[lid];
                if(rec.find(prevStateID) != rec.end()){
                    rec[prevKFID].push_back(prevCompId);
                    lmFactorRecordMulti[lid] = rec;
                }
                else{
                    rec[prevKFID] = vector<int>({prevCompId});
                    lmFactorRecordMulti[lid] = rec;
                }

            }

            c++;
        }
        /// state factor record
        xFactorRecord[prevStateID].push_back(lid);
        initialEstimate.insert(gtsam::Symbol('l', lid), convertPoint3_CV2GTSAM(l->pt3D));
        //VLOG(0)<<"Inserting Initial Estimate for l"<<lid<<endl;
    }

    int c=0;


    for(auto& curCompId : cur_compcam_ids){
        // insert projection factor with current frame to factor graph
        cv::Mat tmp_cv = frontEnd->Tbc_cv*Rt_mats_[curCompId];
        noiseModel::Robust::shared_ptr huberModel1;
        if(!curKPOctaves.empty()){
            auto measurementNoise1 =
                    noiseModel::Isotropic::Sigma(2, frontEnd->orBextractor->GetInverseScaleSigmaSquares()[ curKPOctaves[c]]);
            huberModel1 = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(
                    sqrt(5.991)), measurementNoise1);
            graph.push_back(gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>
                                    (current_measurements[c], huberModel1, gtsam::Symbol('x', curStateID),
                                     gtsam::Symbol('l',lid), K[curCompId], convertPose3_CV2GTSAM(tmp_cv) ));
        }
        else{
            graph.push_back(gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>
                                    (current_measurements[c], huberModel, gtsam::Symbol('x', curStateID),
                                     gtsam::Symbol('l',lid), K[curCompId], convertPose3_CV2GTSAM(tmp_cv) ));
        }


        std::map<int, vector<int>> rec = lmFactorRecordMulti[lid];
        if(rec.find(curKFID) != rec.end()){
            rec[curKFID].push_back(curCompId);
            lmFactorRecordMulti[lid] = rec;
        }
        else{
            rec[curKFID] = vector<int>({curCompId});
            lmFactorRecordMulti[lid] = rec;
        }

        c++;
    }

    /// state factor record
    xFactorRecord[curStateID].push_back(lid);
}

void Backend::insertPriors(int lid){
    /// check if this is the first frame we are inserting
    /// if it is, we have to add prior to the pose and a landmark
    if(currentEstimate.empty() && initialEstimate.empty()) {

        VLOG(0)<<"Went inside GRAPH==0"<<endl;

        Landmark* l = frontEnd->map->getLandmark(lid);
        int numKFs = l->KFs.size();
        MultiCameraFrame* prevLF=  l->KFs[numKFs - 2];

        int prevStateID = prevLF->kfID;
        VLOG(0) <<" prevStateID: " << prevStateID;
        // if imu, add velocity prior, bias prior, pose prior should be such that
        // the rotation of the world frame is taken into account and Tbc is also taken into account

        // initial pose
        if(!reinitialized_){
            prev_Kf = prevStateID;
        }

        /// T_wb = Twc * Tcb = Twc * Tbc.inv()
        cv::Mat initial_pose = prevLF->pose * frontEnd->Tbc_cv.inv();
        // cout << "initial pose " << initial_pose << endl;
        prevPose_ = convertPose3_CV2GTSAM(initial_pose);
        /// Add a prior on pose x0
        gtsam::noiseModel::Diagonal::shared_ptr poseNoise = gtsam::noiseModel::Diagonal::Sigmas
                ((gtsam::Vector(6)<< gtsam::Vector3::Constant(0.1),gtsam::Vector3::Constant(0.1)).finished());
        if(!reinitialized_ || (reinitialized_ && prevStateID!=prev_Kf )){
            graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', prevStateID), prevPose_, poseNoise));
            VLOG(0)<<"Inserted prior for the state x"<<prevStateID<<endl;
            VLOG(1) << "Prior pose added is " << prevPose_ << endl;
        }



        if(frontEnd->useIMU){

            if(!reinitialized_){

                prevVel_ = gtsam::Vector3(0, 0, 0);
                prevBias_ = frontEnd->bias_prior;
                prevState_ = gtsam::NavState(prevPose_, prevVel_);
                graph.push_back(gtsam::PriorFactor<gtsam::Vector3>(gtsam::Symbol('v', prevStateID), prevVel_,
                                                                   frontEnd->priorVelNoise));
                graph.push_back(
                        gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', prevStateID), prevBias_,
                                                                         frontEnd->bias_noise_model));

                initialEstimate.insert(gtsam::Symbol('x', prevStateID), prevPose_);
                initialEstimate.insert(gtsam::Symbol('v', prevStateID), prevVel_);
                initialEstimate.insert(gtsam::Symbol('b', prevStateID), prevBias_);


                VLOG(0) << "Inserted IMU priors" << "v" << prevStateID << " b" << prevStateID << endl;

            }

            else{

//                prevState_ = gtsam::NavState(prevPose_, prevVel_);

                if(!initialEstimate.exists(gtsam::Symbol('x', prev_Kf))){
                    graph.push_back(gtsam::PriorFactor<gtsam::Vector3>(gtsam::Symbol('v', prev_Kf), prevVel_,
                                                                       frontEnd->priorVelNoise));
                    graph.push_back(
                            gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', prev_Kf), prevBias_,
                                                                             frontEnd->bias_noise_model));
                    graph.push_back(
                            gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', prev_Kf), prevState_.pose(),
                                                                             poseNoise));

                    initialEstimate.insert(gtsam::Symbol('x', prev_Kf), prevState_.pose());
                    initialEstimate.insert(gtsam::Symbol('v', prev_Kf), prevVel_);
                    initialEstimate.insert(gtsam::Symbol('b', prev_Kf), prevBias_);

                    VLOG(0) << " prev state added during reint = " << prevState_;
                    VLOG(0) <<" prev bias added during reint " << prevBias_;

                    VLOG(0) << "Inserted IMU priors" << "v" << prev_Kf << " b" << prev_Kf << endl;


                }
            }

        }



        /// LANDMARK PRIOR
        gtsam::Point3 lmgtsam = convertPoint3_CV2GTSAM(l->pt3D);
        addLandmarkPrior(lid, lmgtsam);


        if(false){
            reinitialized_ = false;
            vector<IntraMatch> *intramatches = &frontEnd->currentFrame->intraMatches;
            vector<int> *lids = &frontEnd->currentFrame->lIds;

            std::set<MultiCameraFrame*> kfSet;
            for (int i = 0; i < intramatches->size(); ++i) {
                if (lids->at(i) != -1 ) {
                    int l_id = lids->at(i);
                    vector<MultiCameraFrame*> observedKFs = frontEnd->map->getLandmark(l_id)->KFs;
                    std::copy(observedKFs.begin(), observedKFs.end(), std::inserter(kfSet, kfSet.end()));
                }
            }
            for(auto kf : kfSet){
                int prevStateID = kf->kfID;
                gtsam::Pose3 previous_pose = convertPose3_CV2GTSAM(kf->pose);
                /// Add a prior on pose x0
                gtsam::noiseModel::Diagonal::shared_ptr poseNoise = gtsam::noiseModel::Diagonal::Sigmas
                        ((gtsam::Vector(6)<< gtsam::Vector3::Constant(0.01),gtsam::Vector3::Constant(0.01)).finished());
                graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', prevStateID), previous_pose, poseNoise));
                VLOG(0)<<"Inserted prior for the state x"<<prevStateID<<endl;
            }

        }

    }

}

/**
 * Adds a between factor for Loop Closure.
 * @param curKFID
 * @param best_matchKFID
 * @param relative_pose
 */
void Backend::addLoopClosureFactor()
{
    if(lvar.loop_candidate_)
    {
        VLOG(0) << "Current frame - " << lvar.curr_frame_idx_ << std::endl;
        VLOG(0) << "Best Match frame - " << lvar.best_match_idx_ << std::endl;

        const gtsam::noiseModel::Diagonal::shared_ptr between_noise_model =
                gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6)<< gtsam::Vector3::Constant(0.1),
                        gtsam::Vector3::Constant(0.1)).finished());
        graph.push_back(gtsam::BetweenFactor<gtsam::Pose3>(
                gtsam::Symbol('x', lvar.curr_frame_idx_),
                gtsam::Symbol('x', lvar.best_match_idx_),
                lvar.rel_pose_,
                between_noise_model
        ));
        lvar.loop_candidate_ = false; // Set it back to false.
    }
}

void Backend::addKeyframeGPS(bool addimufactor){


    //create a dummy kf without landmarks but with gps and imu messages when we are using gps and imu
    //this is to ensure that the gps factors are added to the graph at the right time, accounting for the time offset
    if (initialEstimate.empty()) {
        factorNewAffectedKeys.clear();
        newFactor_to_lm.clear();
    }

    int currKFID = frontEnd->currentKFID;
    int prevKFID = frontEnd->lfFrames.back()->kfID;

    if(addimufactor == false){
        // dont add gps to the new frame but instead attach it to the prev vision kf
        currKFID = prevKFID;
    }

    if(frontEnd->useGPS){
        assert(frontEnd->gps_initialized == true);
        assert(frontEnd->gps_msgs_combined.size()!=0);
        addGPSFactor(frontEnd->gps_msgs_combined, currKFID);
    }

    if(addimufactor){
        //add imu factor if we are going to use this gps kf as a separate kf
        if(frontEnd->useIMU){
            addIMUFactor(prev_Kf, currKFID);
        }
        frontEnd->currentKFID++; // for the next step
    }
    return;
}


bool Backend::insert_prior_condition(){

    if(currentEstimate.empty() && initialEstimate.empty()) {
        return true;
    }
    else{
        return false;
    }

}


void Backend::insert_priors_smartFactor(){

    VLOG(0) <<"Inserting priors for the first time"<<endl;


    // After initialization we will have 2 KFs. Add priors for the first KF
    assert(frontEnd->lfFrames.size() == 2);
    // 1. add prior for the first KF
    MultiCameraFrame* first_MCFrame = frontEnd->lfFrames[0];
    int firstKFID = first_MCFrame->kfID;
    prev_Kf = firstKFID;



    // 1.add prior for the first KF pose
    cv::Mat initial_pose = first_MCFrame->pose * frontEnd->Tbc_cv.inv();
    gtsam::Pose3 first_pose = convertPose3_CV2GTSAM(initial_pose);

    // noise model for the first pose
    gtsam::noiseModel::Diagonal::shared_ptr poseNoise = gtsam::noiseModel::Diagonal::Sigmas
    ((gtsam::Vector(6)<< gtsam::Vector3::Constant(0.1),gtsam::Vector3::Constant(0.1)).finished());

    graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', firstKFID), first_pose, poseNoise));
    initialEstimate.insert(gtsam::Symbol('x', firstKFID), first_pose);
    VLOG(0) << "Inserted prior for the state x" << firstKFID << endl;



    if(frontEnd->useIMU){

        prevBias_ = frontEnd->bias_prior;

        // add prior for the first KF velocity
        graph.push_back(gtsam::PriorFactor<gtsam::Vector3>(gtsam::Symbol('v', firstKFID), prevVel_,
                                                           frontEnd->priorVelNoise));

        VLOG(0) << "prev bias added " << prevBias_;

        // add prior for the first KF bias
        graph.push_back(
                        gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', firstKFID), prevBias_,
                                                                         frontEnd->bias_noise_model));

        initialEstimate.insert(gtsam::Symbol('v', firstKFID), prevVel_);
        initialEstimate.insert(gtsam::Symbol('b', firstKFID), prevBias_);

        VLOG(0) << "Inserted IMU priors " << "v" << firstKFID << " b" << firstKFID << endl;


    }


}

void Backend::process_SmartFactor(MultiCameraFrame* currentFrame, GlobalMap* map){

    //Reset the factorNewAffectedKeys map for each new keyframe
    if (initialEstimate.empty()) {
        factorNewAffectedKeys.clear();
        newFactor_to_lm.clear();
    }

    std::vector<IntraMatch> *intramatches = &currentFrame->intraMatches;
    vector<int> *lids = &currentFrame->lIds;

    VLOG(0) <<"intra matches size: "<<intramatches->size()<<endl;
    VLOG(0) <<"lids size: "<<lids->size()<<endl;
    for(int i = 0; i < intramatches->size(); i++) {
        // check: i++ vs ++i

        // if this intra match is a landmark i.e it is tracked in the current frame and is a triangulated inlier
        if(lids->at(i) != -1){

            // 2. get the landmark
            Landmark* l = map->getLandmark(lids->at(i));
            // 3. get the keyframes where this landmark is observed
            vector<MultiCameraFrame*> landmark_kfs = l->KFs;
            // if a smart factor does not exist, it is a new landmark so add prior
            if(smartFactor.count(lids->at(i)) == 0) {

                assert(landmark_kfs.size() == 2); // make sure that this is a new landmark which is just created

                vector<int> compcam_ids;
                IntraMatch *im = &landmark_kfs[0]->intraMatches[l->featInds[0]];

                for (int cam_id = 0; cam_id < landmark_kfs[0]->num_cams_; cam_id++) {
                    int kp_ind = im->matchIndex[cam_id];
                    if (kp_ind != -1) {
                        compcam_ids.push_back(cam_id);
                    }
                }

                insertSmartFactorinGraph(lids->at(i), compcam_ids, im, landmark_kfs[0]);

            }
            else{
                // add the
                if(lm_to_facInds.find(lids->at(i)) == lm_to_facInds.end() ){
                    VLOG(0)<<"DID not find the factor in lm_to_facInds corresponding to LID: "<<lids->at(i);
                    VLOG(0)<<"Smart factor for the landmark "<< (smartFactor.find(lids->at(i)) !=smartFactor.end());
                    VLOG(0)<<"Landmark in the map: "<<map->getLandmark(lids->at(i));
                    VLOG(0)<<"Size of the map: "<<map->num_lms;
                }
//                try {
                    factorNewAffectedKeys[lm_to_facInds.at(lids->at(i))].insert(gtsam::Symbol('x', currentFrame->kfID));
//                }
//                catch (const std::out_of_range& oor) {
//                    VLOG(0) << "Out of Range error: " << oor.what() << endl;
//                    VLOG(0) << "Landmark ID: " << lids->at(i) << endl;
//                    VLOG(0) << "Landmark in the map: " << map->getLandmark(lids->at(i));
//                    VLOG(0) << "Size of the map: " << map->num_lms;
//                }
            }

            // Insert smart factor for the current keyframe

            // make sure the current keyframe is same as the last keyframe associated with the landmarks
            assert(currentFrame->kfID == landmark_kfs.back()->kfID);
            vector<int> compcam_ids;
            IntraMatch *im = &intramatches->at(i);

            for (int cam_id = 0; cam_id < currentFrame->num_cams_; cam_id++) {
                int kp_ind = im->matchIndex[cam_id];
                if (kp_ind != -1) {
                    compcam_ids.push_back(cam_id);
                }
            }

            insertSmartFactorinGraph(lids->at(i), compcam_ids, im, currentFrame);

        }
    }
}


void Backend::insertSmartFactorinGraph(int lid, vector<int> compcam_ids, IntraMatch* im, MultiCameraFrame*  currentFrame){

    int pixel_noise = 15.0;
    gtsam::SharedIsotropic  measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, pixel_noise);
    int kf_id = currentFrame->kfID;
    if(smartFactor.count(lid) == 0) {

        debug_lids.push_back(lid);
        // create a new smart factor

//        int octave_value = currentFrame->image_kps_undist[compcam_ids[0]][im->matchIndex[compcam_ids[0]]].octave;
//        VLOG(0) << "octave value: " << octave_value << endl;
//        auto measurementNoise1 =
//                noiseModel::Isotropic::Sigma(2, frontEnd->orBextractor->GetInverseScaleSigmaSquares()[octave_value]);
//        VLOG(0) << "measurement noise: " << frontEnd->orBextractor->GetInverseScaleSigmaSquares()[octave_value] << endl;
        smartFactor[lid] = RigFactor::shared_ptr(new RigFactor(measurementNoise, cameraRig, smartFactor_params));
        newFactor_to_lm[graph.size()] = lid;
//        VLOG(0)<<"new factor inserted for : "<<lid;
        graph.push_back(smartFactor[lid]);
//        VLOG(0) << "inserting smart factor for lid: " << lid << " to x" << currentFrame->kfID << endl;
    }


    for (auto cam_id : compcam_ids) {

        int featIdx = im->matchIndex[cam_id];
        gtsam::Point2 measurement = convertPoint2_CV2GTSAM(currentFrame->image_kps_undist[cam_id][featIdx]);
        smartFactor[lid]->add(measurement, gtsam::Symbol('x', kf_id), cam_id);
    }

}

void Backend::addPosetoInitialEstimate(MultiCameraFrame* currentFrame){

    // add pose to the graph if it is not already added
//    assert(initialEstimate.exists(gtsam::Symbol('x', currentFrame->kfID)) == false);
    // add pose to the graph

    if(initialEstimate.exists(gtsam::Symbol('x', currentFrame->kfID))){
        VLOG(0) << "pose already exists in the initial estimate " <<currentFrame->kfID<< endl;
        return;
    }
    if(currentEstimate.exists(gtsam::Symbol('x', currentFrame->kfID))){
        VLOG(0) << "pose already exists in the current estimate " << currentFrame->kfID<< endl;
        return;
    }
    if(prev_Kf == -1){
        prev_Kf = frontEnd->lfFrames[0]->kfID;
    }
    cv::Mat pose = currentFrame->pose * frontEnd->Tbc_cv.inv();
    gtsam::Pose3 current_pose = convertPose3_CV2GTSAM(pose);
    initialEstimate.insert(gtsam::Symbol('x', currentFrame->kfID), current_pose);
    VLOG(0)<<"Inserting Initial Estimate for x"<<currentFrame->kfID<<endl;
    VLOG(0)<<"Current pose being added: "<< endl << current_pose<<endl;

}

void Backend::addPoseInitialEstimateRelocalization(MultiCameraFrame* currentFrame) {

        // add pose to the graph if it is not already added
        if(initialEstimate.exists(gtsam::Symbol('x', currentFrame->kfID))){
            VLOG(0) << "pose already exists in the initial estimate " <<currentFrame->kfID<< endl;
            return;
        }
        if(currentEstimate.exists(gtsam::Symbol('x', currentFrame->kfID))){
            VLOG(0) << "pose already exists in the current estimate " << currentFrame->kfID<< endl;
            return;
        }
        if(prev_Kf == -1){
            prev_Kf = frontEnd->lfFrames[0]->kfID;
        }
        cv::Mat pose = currentFrame->pose;
        gtsam::Pose3 current_pose = convertPose3_CV2GTSAM(pose);
        initialEstimate.insert(gtsam::Symbol('x', currentFrame->kfID), current_pose);
        VLOG(0)<<"Inserting Initial Estimate for x"<<currentFrame->kfID<<endl;
        VLOG(0)<<"Current pose being added: "<< endl << current_pose<<endl;
        return;
}

void Backend::addLoopClosure(int curr_frame_id){


    // reqd variables
    vector<int> matched_lids = frontEnd->result.lIds;
    for(int i = 0; i < matched_lids.size(); i++){
        int matched_lid = matched_lids[i];

        // do we need this?
        if(smartFactor.count(matched_lid) == 0){
            VLOG(0) << "Lid " << matched_lid << " not found in smart factor" << endl;
            continue;
        }

        for(auto &measurement : frontEnd->result.measurements[i]){
            int comp_cam_id = std::get<0>(measurement);
            cv::Point2f  pixel_meas = std::get<1>(measurement);

            gtsam::Point2 meas(pixel_meas.x, pixel_meas.y);

            assert(meas.x() >= 0 && meas.y() >= 0);

            // add the smart factor
            smartFactor[matched_lid]->add(meas, gtsam::Symbol('x', curr_frame_id), comp_cam_id);
            VLOG(0) << "added measurement for lid " << matched_lid << " to x" << curr_frame_id << " for cam " << comp_cam_id << endl;

        }

        factorNewAffectedKeys[lm_to_facInds.at(matched_lid)].insert(gtsam::Symbol('x', curr_frame_id));
    }
}

bool Backend::RelocalizationGraphConstructor(gtsam::NavState currentState) {
    // add smart factor for the current frame using the member variables reprojectedLandmarks and reprojectedKeypoints
    // in frontend.h. add IMU factor between previous and current keyframe and add IMU predicted pose as initial estimate
    // for the current keyframe
    float pixel_noise = 10.0;
//    gtsam::SharedIsotropic  measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, pixel_noise);
//    noiseModel::Robust::shared_ptr huberModel = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(
//            sqrt(5.991)), measurementNoise);
        // iterate through each camera
        for (int camId = 0; camId < frontEnd->currentFrame->num_cams_; camId++) {
            // iterate through each keypoint stored in std::map<int, std::vector<cv::KeyPoint>> reprojectedKeypoints for this
            // camInd and use genericProjectionFactor to add the measurement to each pose node
            cv::Mat tempTbc = frontEnd->Tbc_cv * Rt_mats_[camId];
            gtsam::Pose3 bTc = convertPose3_CV2GTSAM(tempTbc);
            Cal3_S2::shared_ptr K(new Cal3_S2(frontEnd->camconfig_.K_mats_[camId].at<double>(0,0),
                                              frontEnd->camconfig_.K_mats_[camId].at<double>(1,1), 0.0,
                                              frontEnd->camconfig_.K_mats_[camId].at<double>(0,2),
                                              frontEnd->camconfig_.K_mats_[camId].at<double>(1,2)));
            // for debugging purpose, print out the reprojected keypoints and landmark
            for (int kpIndex = 0; kpIndex < frontEnd->reprojectedKeypoints[camId].size(); kpIndex++) {
                float measX = frontEnd->reprojectedKeypoints[camId][kpIndex].pt.x;
                float measY = frontEnd->reprojectedKeypoints[camId][kpIndex].pt.y;
                gtsam::Point2 measurement(measX, measY);
                vector<double> landmark;
                for (int i = 0; i < 3; i++) {
                    landmark.push_back(frontEnd->reprojectedLandmarks[camId][kpIndex].at<double>(i));
                }
                gtsam::Point3 landmark_gtsam(landmark[0], landmark[1], landmark[2]);
                // create a generic projection factor for this measurement
//                graph.push_back(gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>
//                                        (measurement, huberModel,
//                                         gtsam::Symbol('x', frontEnd->currentFrame->kfID),
//                                         gtsam::Symbol('l', frontEnd->reprojectedLandmarkIds[camId][kpIndex]), K,
//                                         bTc));
                // create RigResectioningFactor for this measurement
                auto factor = RigResectioningFactor(huberModel, gtsam::Symbol('x', frontEnd->currentFrame->kfID), K,
                                                   measurement, landmark_gtsam, frontEnd->reprojectedKeypoints[camId][kpIndex].octave, camId, bTc);
                graph.push_back(factor);

                // if initial estimate does not contain the landmark, add it
//                if (!currentEstimate.exists(gtsam::Symbol('l', frontEnd->reprojectedLandmarkIds[camId][kpIndex])) &&
//                !initialEstimate.exists(gtsam::Symbol('l', frontEnd->reprojectedLandmarkIds[camId][kpIndex]))) {
//                    initialEstimate.insert(gtsam::Symbol('l', frontEnd->reprojectedLandmarkIds[camId][kpIndex]),
//                                           landmark_gtsam);
//
//                    // add strong prior for the landmark so the optimization doesn't move the landmarks
//                    graph.push_back(gtsam::PriorFactor<gtsam::Point3>(
//                            gtsam::Symbol('l', frontEnd->reprojectedLandmarkIds[camId][kpIndex]),
//                            landmark_gtsam, gtsam::noiseModel::Isotropic::Sigma(3, 0.01)));
//                }
            }
        }
        // insert timestamp for the current keyframe
        newTimeStamps[gtsam::Symbol('x', frontEnd->currentFrame->kfID)] = frontEnd->currentFrame->kfID;
//    }
    //
//    addIMUFactor(prev_Kf, frontEnd->currentFrame->kfID);
    addIMUFactorRelocalization(prev_Kf, frontEnd->currentFrame->kfID, currentState);
    if (frontEnd->lfFrames.size() == 2) {
        // get the pose of the kf with keyframe id as prev_Kf
        cv::Mat initial_pose = frontEnd->lfFrames[prev_Kf]->pose;
        VLOG(0) << "pose of the first keyframe x" << prev_Kf << " : " << initial_pose << endl;
        gtsam::Pose3 first_pose = convertPose3_CV2GTSAM(initial_pose);
        graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', prev_Kf), first_pose, gtsam::noiseModel::Diagonal::Sigmas
                ((gtsam::Vector(6)<< gtsam::Vector3::Constant(0.01),gtsam::Vector3::Constant(0.01)).finished())));
        // add b0 and v0 initial estimate
        gtsam::Vector3 prevVel(0, 0, 0);
        gtsam::imuBias::ConstantBias prevBias = frontEnd->bias_prior;
        graph.push_back(gtsam::PriorFactor<gtsam::Vector3>(gtsam::Symbol('v', prev_Kf), prevVel, frontEnd->priorVelNoise));
        graph.push_back(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', prev_Kf), prevBias, frontEnd->bias_noise_model));
        initialEstimate.insert(gtsam::Symbol('b', prev_Kf), prevBias);
        initialEstimate.insert(gtsam::Symbol('v', prev_Kf), prevVel);
        initialEstimate.insert(gtsam::Symbol('x', prev_Kf), first_pose);
    }
    addPoseInitialEstimateRelocalization(frontEnd->currentFrame);
    // add the new landmarks to the initial estimate
    poseCounter++;
    return true;
}

void Backend::addIMUFactorRelocalization(int prev_kf, int curr_kf, gtsam::NavState currentSate) {
    VLOG(0) << "Adding IMU factor between " << prev_kf << " and " << curr_kf << endl;
    auto preint_imu_combined = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(*frontEnd->imu_integrator_comb);
    gtsam::CombinedImuFactor imu_factor(gtsam::Symbol('x', prev_kf ), gtsam::Symbol('v', prev_kf ),
                                        gtsam::Symbol('x', curr_kf ), gtsam::Symbol('v', curr_kf ),
                                        gtsam::Symbol('b', prev_kf ), gtsam::Symbol('b', curr_kf ), preint_imu_combined);
    graph.push_back(imu_factor);
    VLOG(0) << "prev Bias while adding imu factor \n" << prevBias_ << endl;
    VLOG(0) << "prev State while adding imu factor for state \n" << prevState_ << endl;

    VLOG(0) << "Predicted estimate for x" << curr_kf << " is: \n"<<currentSate.pose()<<endl;
    VLOG(0) << "Velocity in Nav frame - " << curr_kf << " is : \n" << currentSate.v() << endl;
    VLOG(0) << "Velocity in Body Frame - " << curr_kf << "is :\n" << currentSate.bodyVelocity() << endl;
    VLOG(0) << "Initial estimate for bias b" << curr_kf << "is :  \n" << prevBias_ << endl;

    if(!initialEstimate.exists(gtsam::Symbol('x', curr_kf))){
        VLOG(1) << "next estimate sent from IMU!!" ;
        initialEstimate.insert(gtsam::Symbol('x', curr_kf), currentSate.pose());
    }

    initialEstimate.insert(gtsam::Symbol('v', curr_kf), currentSate.v());
    initialEstimate.insert(gtsam::Symbol('b', curr_kf), prevBias_);
}

bool Backend::SmartFactor_backend(bool loop_closed){

    // 1. process the landmark
    // during intialization, we need to attach landmarks for both the frames.
    // its missed here.
    MultiCameraFrame *currentFrame = frontEnd->currentFrame;
    GlobalMap *map = frontEnd->map;


    // 2. insert priors
    if(insert_prior_condition() == true){
        insert_priors_smartFactor();
    }

//    graph.print("Initial Factor Graph: \n");
//    initialEstimate.print("Initial Estimate: \n");

    // 3. add smart factors
    // browse through all the intramatches and add smart factors

    auto start_time = std::chrono::high_resolution_clock::now();
    process_SmartFactor(frontEnd->lfFrames.back(), map);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    VLOG(0) << "Time taken to add smart factors: " << duration.count() << endl;


    addPosetoInitialEstimate(frontEnd->lfFrames.back());

//    graph.print("Factor Graph after adding smart factors: \n");
//    initialEstimate.print("Initial Estimate after adding smart factors: \n");


    // handle it for the imu case
    if(frontEnd->useIMU){
        if(prev_Kf != -1){

            if(frontEnd->imu_message_empty == true) {
                // dont add the imu factor and turn the flag to false
                frontEnd->imu_message_empty = false;
                return false;
            }

            if(frontEnd->useGPS && frontEnd->imu_message_empty == true){
                // dont add the imu factor and turn the flag to false
                frontEnd->imu_message_empty = false;
            }
            else{
                addIMUFactor(prev_Kf, currentFrame->kfID);
            }
        }
    }
//    else{
//        addPosetoInitialEstimate(frontEnd->lfFrames.back());
//    }


    // check for loop closure.
    // if yes, then add the landmarks to the smart factor of the correspoding lid

    if(loop_closed == true){
        // close the loop here

        addLoopClosure(currentFrame->kfID);

        VLOG(0) << "Loop closed" << endl;
    }




    // do we return anything else?
    // check if there is such a condition
    return true;

}



bool Backend::addKeyFrameMulti() {

    //check if camconfig has been update, if so update here as well
    if(true){
        camArrayConfig = &frontEnd->camconfig_;
        camTransformation = cv::Mat(4, 4, CV_64F);
        for (int i = 0; i < 3; ++i) {
            camTransformation.at<double>(i, 3) = camArrayConfig->t_mats_[camID].at<double>(i, 0);
            for (int j = 0; j < 3; ++j)
                camTransformation.at<double>(i, j) = camArrayConfig->R_mats_[camID].at<double>(i, j);
        }

        camTransformation.at<double>(3,0) = 0;
        camTransformation.at<double>(3,1) = 0;
        camTransformation.at<double>(3,2) = 0;
        camTransformation.at<double>(3,3) = 1;

        camTransformation = camTransformation.inv();

        for(int i =0 ; i < camArrayConfig->num_cams_ ; i++){

            K.push_back(gtsam::Cal3_S2::shared_ptr(new gtsam::Cal3_S2(camArrayConfig->K_mats_[i].at<double>(0, 0),
                                                                      camArrayConfig->K_mats_[i].at<double>(1, 1), 0,
                                                                      camArrayConfig->K_mats_[i].at<double>(0, 2),
                                                                      camArrayConfig->K_mats_[i].at<double>(1, 2))));

            Mat R = camArrayConfig->Kalibr_R_mats_[i];/// store the pose of the cam chain
            //VLOG(3)<<R;
            Mat t = camArrayConfig->Kalibr_t_mats_[i];
            Mat kalibrPose = Mat::eye(4,4, CV_64F);
            kalibrPose.rowRange(0,3).colRange(0,3) = R.t();
            kalibrPose.rowRange(0,3).colRange(3,4) = -1* R.t()*t;
            Rt_mats_kalib_.push_back(kalibrPose.clone());

            Mat R2 = camArrayConfig->R_mats_[i];/// store the pose of the cam chain
            Mat t2 = camArrayConfig->t_mats_[i];
            Mat camPose = Mat::eye(4,4, CV_64F);
            camPose.rowRange(0,3).colRange(0,3) = R2.t();
            camPose.rowRange(0,3).colRange(3,4) = -1* R2.t()*t2;

            Rt_mats_.push_back(camPose.clone());
            //VLOG(3)<<"RTMats cam: "<<i<<" : "<<camPose<<endl;

        }
    }

    MultiCameraFrame *currentFrame = frontEnd->currentFrame;
    GlobalMap *map = frontEnd->map;
    bool ret = true;

    /// Variable declaration
    std::map<int, set<int> > insertedRigid;
    int num_lms_filtered=0;
    vector<int> lids_filtered;
    vector<gtsam::Point3> landmarks;
    vector<vector<gtsam::Point2>> current_measurements;
    vector<vector<gtsam::Point2>> previous_measurements;
    vector<vector<int>>  previous_compcam_ids, cur_compcam_ids;
    vector<MultiCameraFrame*> previous_KFs;
    vector<int> prevKFInds;
    vector<bool> new_landmark_flags;

    vector<IntraMatch> *intramatches = &currentFrame->intraMatches;
    vector<int> *lids = &currentFrame->lIds;
    int current_pose_id = currentFrame->kfID;

    std::set<MultiCameraFrame*> kfSetOlder;







    // browse through all instramatches
    for (int i = 0; i < intramatches->size(); ++i) {
        // if this intra match is a landmark i.e it is tracked in the current frame and is a triangulated inlier
        if (lids->at(i) != -1 ) {

            //record the landmark
            Landmark* l = map->getLandmark(lids->at(i));
            gtsam::Point3 landmark = convertPoint3_CV2GTSAM(l->pt3D);
            int numKFs = l->KFs.size();
            if(!currentEstimate.exists(gtsam::Symbol('l', lids->at(i))) and
               !initialEstimate.exists(gtsam::Symbol('l', lids->at(i))))
            {
               // if(numKFs <=2)
               //     continue;
                /// Get the previous LF frame of the landmark and the observations in the component cameras
                vector<int> prevRaysCamIds, curRaysCamIDs;
                bool firstPair = true;
                for (int pairIdx = (numKFs-1) ; pairIdx< numKFs ; pairIdx++){
                    int prevKFIdx =  pairIdx-1;     //(numKFs-2);
                    int curKFIdx =   pairIdx;       //numKFs-1;
                    MultiCameraFrame* lf1 = l->KFs[prevKFIdx]; // previous KF
                    MultiCameraFrame* lf2 = l->KFs[curKFIdx];  // same as l->KFs[numKFs-1]; // last KF
                    IntraMatch* im1 = &lf1->intraMatches.at(l->featInds[prevKFIdx]);
                    IntraMatch* im2 = &lf2->intraMatches.at(l->featInds[curKFIdx]);

                    Mat angles = Mat::zeros(lf1->num_cams_,lf2->num_cams_, CV_64FC1);
                    Mat tri_errors = Mat::zeros(lf1->num_cams_,lf2->num_cams_, CV_64FC1);

                    ///check all combinations of rays and get the triangulation angles and reprojection errors

                    int curKFCamID = -1, prevKFCamID = -1;
                    double maxAngle = 0;
                    curRaysCamIDs.clear();
                    getCamIDs_Angles(l, prevKFIdx, curKFIdx, angles , prevRaysCamIds,curRaysCamIDs, prevKFCamID, curKFCamID);
                    if(prevKFCamID != -1 and curKFCamID != -1)
                        maxAngle = angles.at<double>(prevKFCamID,curKFCamID);
                    // compare all the angles and choose the  largest one for insertion
                    //first check the diagonal angles i.e intersecting component cameras between two LF fram

                    vector<int> acceptedPrevCamIDs, acceptedCurCamIDs, acceptedPrevKPOctaves, acceptedCurKPOctaves;
                    vector<gtsam::Point2> acceptedPrevMeas, acceptedCurMeas;
                    VLOG(3)<<"MAx Angle : "<<maxAngle;
                    if(maxAngle > 0){

                        acceptedPrevCamIDs.push_back(prevKFCamID);
                        acceptedPrevKPOctaves.push_back(lf1->image_kps_undist[prevKFCamID][im1->matchIndex[prevKFCamID]].octave);
                        acceptedPrevMeas.push_back(convertPoint2_CV2GTSAM(lf1->image_kps_undist[prevKFCamID][im1->matchIndex[prevKFCamID]]));

                        acceptedCurCamIDs.push_back(curKFCamID);
                        acceptedCurMeas.push_back(convertPoint2_CV2GTSAM(lf2->image_kps_undist[curKFCamID][im2->matchIndex[curKFCamID]]));
                        acceptedCurKPOctaves.push_back(lf2->image_kps_undist[curKFCamID][im2->matchIndex[curKFCamID]].octave);

                        vector<int>::iterator itr1 = prevRaysCamIds.begin();
                        vector<int>::iterator itr2 = curRaysCamIDs.begin();

                        for( ; itr1!=prevRaysCamIds.end() or itr2!= curRaysCamIDs.end() ; ){
                            int camID1 = *itr1;
                            int camID2 = *itr2;
                            /////////////// Check the latest in prevrays
                            if( itr1!=prevRaysCamIds.end()){
                                /// if this is the first KF pair for the landmark
                                /// choose which rays to insert from the previous  component cams
                                if(camID1 != prevKFCamID){
                                    if(firstPair){
                                        bool accept=true;
                                        vector<int>::iterator it_set = acceptedPrevCamIDs.begin();

                                        for( ; it_set != acceptedPrevCamIDs.end() ; ++it_set){
                                            int kp_ind1 = im1->matchIndex[camID1];
                                            int kp_ind2 = im1->matchIndex[*it_set];
                                            gtsam::Pose3 transformedPose1 = calCompCamPose(lf1, camID1); // take input of the LF frame pose and comp cam id to compute the comp cam pose
                                            gtsam::Pose3 transformedPose2 = calCompCamPose(lf1, *it_set);
                                            gtsam::Point2 obs1 = convertPoint2_CV2GTSAM(lf1->image_kps_undist[camID1][kp_ind1]);
                                            gtsam::Point2 obs2  = convertPoint2_CV2GTSAM(lf1->image_kps_undist[*it_set][kp_ind2]);

                                            //check the reprojection error again to filter any outliers
                                            double error1 = computeReprojectionError(transformedPose1, camID1, landmark, obs1);
                                            double error2 = computeReprojectionError(transformedPose2, *it_set, landmark, obs2);
                                            double tri_angle_deg=0;
                                            //compute angle
                                            bool angleCheck = checkTriangulationAngle(const_cast<gtsam::Point3 &>(transformedPose1.translation()),
                                                                                      const_cast<gtsam::Point3 &>(transformedPose2.translation()),
                                                                                      landmark , tri_angle_deg);
                                            //VLOG(3)<<"Prev Cams Tri ANgle : "<<tri_angle_deg;
                                            if(!angleCheck || error1 >5.991*frontEnd->orBextractor->GetScaleSigmaSquares()[lf1->image_kps_undist[camID1][kp_ind1].octave] ||
                                            error2 >5.991*frontEnd->orBextractor->GetScaleSigmaSquares()[lf1->image_kps_undist[*it_set][kp_ind2].octave]){
                                                accept = false;
                                                break;
                                            }
                                        }

                                        if(accept){
                                            it_set = acceptedCurCamIDs.begin();
                                            for( ; it_set != acceptedCurCamIDs.end() ; ++it_set){
                                                if(angles.at<double>(camID1, *it_set) == 0){
                                                    accept = false;
                                                    break;
                                                }
                                            }
                                            if(accept)
                                            {
                                                acceptedPrevCamIDs.push_back(camID1);
                                                acceptedPrevMeas.push_back(convertPoint2_CV2GTSAM(lf1->image_kps_undist[camID1][im1->matchIndex[camID1]]));
                                                acceptedPrevKPOctaves.push_back(lf1->image_kps_undist[camID1][im1->matchIndex[camID1]].octave);

                                            }
                                        }

                                    }
                                        /// if this is not the first KF pair for the landmark
                                        /// accept all the rays in the previous frame, since they are already chosen
                                    else{
                                        acceptedPrevCamIDs.push_back(camID1);
                                        acceptedPrevMeas.push_back(convertPoint2_CV2GTSAM(lf1->image_kps_undist[camID1][im1->matchIndex[camID1]]));
                                        acceptedPrevKPOctaves.push_back(lf1->image_kps_undist[camID1][im1->matchIndex[camID1]].octave);
                                    }
                                }
                                ++itr1;
                            }

                            if(itr2!= curRaysCamIDs.end()){

                                if(camID2 != curKFCamID){
                                    bool accept=true;

                                    vector<int>::iterator it_set = acceptedCurCamIDs.begin();

                                    for( ; it_set != acceptedCurCamIDs.end() ; ++it_set){
                                        int kp_ind1 = im2->matchIndex[camID2];
                                        int kp_ind2 = im2->matchIndex[*it_set];
                                        gtsam::Pose3 transformedPose1 = calCompCamPose(lf2, camID2); // take input of the LF frame pose and comp cam id to compute the comp cam pose
                                        gtsam::Pose3 transformedPose2 = calCompCamPose(lf2, *it_set);
                                        gtsam::Point2 obs1 = convertPoint2_CV2GTSAM(lf2->image_kps_undist[camID2][kp_ind1]);
                                        gtsam::Point2 obs2  = convertPoint2_CV2GTSAM(lf2->image_kps_undist[*it_set][kp_ind2]);

                                        //check the reprojection error again to filter any outliers
                                        double error1 = computeReprojectionError(transformedPose1, camID2, landmark, obs1);
                                        double error2 = computeReprojectionError(transformedPose2, *it_set, landmark, obs2);
                                        double tri_angle_deg=0;
                                        //compute angle
                                        bool angleCheck = checkTriangulationAngle(const_cast<gtsam::Point3 &>(transformedPose1.translation()),
                                                                                  const_cast<gtsam::Point3 &>(transformedPose2.translation()),
                                                                                  landmark , tri_angle_deg);

                                        if(!angleCheck || error1 >5.991*frontEnd->orBextractor->GetScaleSigmaSquares()[lf2->image_kps_undist[camID2][kp_ind1].octave] ||
                                        error2 >5.991*frontEnd->orBextractor->GetScaleSigmaSquares()[lf2->image_kps_undist[*it_set][kp_ind2].octave]){
                                            accept = false;
                                            break;
                                        }
                                    }

                                    if(accept){
                                        it_set = acceptedPrevCamIDs.begin();
                                        for( ; it_set != acceptedPrevCamIDs.end() ; ++it_set){
                                            if(angles.at<double>(*it_set, camID2) == 0){
                                                accept = false;
                                                break;
                                            }
                                        }
                                        if(accept)
                                        {
                                            acceptedCurCamIDs.push_back(camID2);
                                            acceptedCurMeas.push_back(convertPoint2_CV2GTSAM(lf2->image_kps_undist[camID2][im2->matchIndex[camID2]]));
                                            acceptedCurKPOctaves.push_back(lf2->image_kps_undist[camID2][im2->matchIndex[camID2]].octave);
                                        }
                                    }
                                }
                                ++itr2;
                            }
                        }
                        VLOG(3)<<"New landmark LID : "<<lids->at(i)<<"  KF1 :"<<lf1->kfID<<", KF2:"<<lf2->kfID<<", PT: "<<landmark.x()<<","<<landmark.y()<<","<<landmark.z()<<" , max angle : "<<maxAngle<<"Num meas: "<<acceptedCurCamIDs.size()+acceptedPrevCamIDs.size()<<endl;

                        if(firstPair){
                            firstPair  = false;
                            num_lms_filtered++;
                            if(num_lms_filtered == 1){
                                insertPriors(lids->at(i));
                            }
                            else{
                                if(lf1->kfID < last_reset_kfid)
                                    kfSetOlder.insert(lf1);
                            }

                            insertLandmarkInGraph(l, prevKFIdx, curKFIdx, true, acceptedPrevCamIDs,
                                                  acceptedPrevMeas, acceptedCurCamIDs, acceptedCurMeas, acceptedPrevKPOctaves,
                                                  acceptedCurKPOctaves);

                            prevRaysCamIds = acceptedCurCamIDs;

                        }
                        else{
                            insertLandmarkInGraph(l, prevKFIdx, curKFIdx, false, acceptedPrevCamIDs,
                                                  acceptedPrevMeas, acceptedCurCamIDs, acceptedCurMeas, acceptedPrevKPOctaves,
                                                  acceptedCurKPOctaves);
                            VLOG(0)<<"NEVER COMES HERE";

                        }

                    }

                }

            }
            else{
                /// this is an existing landmark in the graph.
                /// Grab the last observation
                std::map<int, vector<int>> facs = lmFactorRecordMulti[lids->at(i)];
                VLOG(3)<<"Existing Landmark"<<endl;
                assert(facs.size() != 0);

                /// Get the prev factor's frameID and componenet camera IDs
                vector<int> prevRaysCamIds = facs.rbegin()->second;
                int prevKFIdx = facs.rbegin()->first;
                int curKFIdx = numKFs-1;
                MultiCameraFrame* lf1;
                IntraMatch* im1;

                lf1 = l->KFs[prevKFIdx];
                im1 = &lf1->intraMatches.at(l->featInds[prevKFIdx]);
                int prevFrameID = lf1->kfID;
                VLOG(3)<<"prevFrameID: "<<prevFrameID<<endl; //<<", CamIDs: "<<CamID1<<endl;

                MultiCameraFrame* lf2 = currentFrame;  // same as l->KFs[numKFs-1]; // last KF
                IntraMatch* im2 = &lf2->intraMatches.at(l->featInds.back());

                double maxAngle = 0;
                int curKFCamID = -1, prevKFCamID = -1;
                vector<int>  curRaysCamIDs;
                Mat angles = Mat::zeros(lf1->num_cams_,lf2->num_cams_, CV_64FC1);
                Mat tri_errors = Mat::zeros(lf1->num_cams_,lf2->num_cams_, CV_64FC1);

                getCamIDs_Angles(l, prevKFIdx, curKFIdx , angles , prevRaysCamIds,curRaysCamIDs, prevKFCamID, curKFCamID);
                if(prevKFCamID != -1 and curKFCamID != -1)
                    maxAngle = angles.at<double>(prevKFCamID,curKFCamID);
                // compare all the angles and choose the  largest one for insertion
                //first check the diagonal angles i.e intersecting component cameras between two LF frame
                vector<int> acceptedPrevCamIDs, acceptedCurCamIDs, acceptedPrevKPOctaves, acceptedCurKPOctaves;
                vector<gtsam::Point2> acceptedPrevMeas, acceptedCurMeas;
                if(maxAngle > 0){
                    acceptedPrevCamIDs.push_back(prevKFCamID);
                    acceptedPrevMeas.push_back(convertPoint2_CV2GTSAM(lf1->image_kps_undist[prevKFCamID][im1->matchIndex[prevKFCamID]]));
                    acceptedPrevKPOctaves.push_back(lf1->image_kps_undist[prevKFCamID][im1->matchIndex[prevKFCamID]].octave);

                    acceptedCurCamIDs.push_back(curKFCamID);
                    acceptedCurMeas.push_back(convertPoint2_CV2GTSAM(lf2->image_kps_undist[curKFCamID][im2->matchIndex[curKFCamID]]));
                    acceptedCurKPOctaves.push_back(lf2->image_kps_undist[curKFCamID][im2->matchIndex[curKFCamID]].octave);
                }

                vector<int>::iterator itr1 = prevRaysCamIds.begin();
                for( ; itr1!=prevRaysCamIds.end() ; ++itr1){
                    int camID1 = *itr1;
                    if(camID1 != prevKFCamID){
                        acceptedPrevCamIDs.push_back(camID1);
                        acceptedPrevMeas.push_back(convertPoint2_CV2GTSAM(lf1->image_kps_undist[camID1][im1->matchIndex[camID1]]));
                        acceptedPrevKPOctaves.push_back(lf1->image_kps_undist[camID1][im1->matchIndex[camID1]].octave);
                    }
                }


                vector<int>::iterator itr2 = curRaysCamIDs.begin();
                for( ;  itr2!= curRaysCamIDs.end() ; ++itr2){
                    int camID2 = *itr2;

                    if(camID2 != curKFCamID){
                        bool accept=true;

                        vector<int>::iterator it_set = acceptedCurCamIDs.begin();

                        for( ; it_set != acceptedCurCamIDs.end() ; ++it_set){
                            int kp_ind1 = im2->matchIndex[camID2];
                            int kp_ind2 = im2->matchIndex[*it_set];
                            gtsam::Pose3 transformedPose1 = calCompCamPose(lf2, camID2); // take input of the LF frame pose and comp cam id to compute the comp cam pose
                            gtsam::Pose3 transformedPose2 = calCompCamPose(lf2, *it_set);
                            gtsam::Point2 obs1 = convertPoint2_CV2GTSAM(lf2->image_kps_undist[camID2][kp_ind1]);
                            gtsam::Point2 obs2  = convertPoint2_CV2GTSAM(lf2->image_kps_undist[*it_set][kp_ind2]);

                            //check the reprojection error again to filter any outliers
                            double error1 = computeReprojectionError(transformedPose1, camID2, landmark, obs1);
                            double error2 = computeReprojectionError(transformedPose2, *it_set, landmark, obs2);
                            double tri_angle_deg=0;
                            //compute angle
                            bool angleCheck = checkTriangulationAngle(const_cast<gtsam::Point3 &>(transformedPose1.translation()),
                                                                      const_cast<gtsam::Point3 &>(transformedPose2.translation()),
                                                                      landmark , tri_angle_deg);

                            if(!angleCheck || error1 >5.991*frontEnd->orBextractor->GetScaleSigmaSquares()[lf2->image_kps_undist[camID2][kp_ind1].octave] ||
                            error2 > 5.991*frontEnd->orBextractor->GetScaleSigmaSquares()[lf2->image_kps_undist[*it_set][kp_ind2].octave]){
                                accept = false;
                                break;
                            }
                        }

                        if(accept){
                            it_set = acceptedPrevCamIDs.begin();
                            for( ; it_set != acceptedPrevCamIDs.end() ; ++it_set){
                                if(angles.at<double>(*it_set, camID2) == 0){
                                    accept = false;
                                    break;
                                }
                            }
                            if(accept)
                            {
                                acceptedCurCamIDs.push_back(camID2);
                                acceptedCurMeas.push_back(convertPoint2_CV2GTSAM(lf2->image_kps_undist[camID2][im2->matchIndex[camID2]]));
                                acceptedCurKPOctaves.push_back(lf2->image_kps_undist[camID2][im2->matchIndex[camID2]].octave);
                            }
                        }
                    }
                }

                VLOG(3)<<"LID : "<<lids->at(i)<<", PT: "<<landmark.x()<<","<<landmark.y()<<","<<landmark.z()<<" , max angle : "<<maxAngle<<"Num meas: "<<acceptedCurCamIDs.size()+acceptedPrevCamIDs.size()<<endl;
                if ( maxAngle > 0 ) {
                    num_lms_filtered++;
                    /*lids_filtered.push_back(lids->at(i));
                    landmarks.push_back(landmark);
                    previous_KFs.push_back(lf1);
                    prevKFInds.push_back(prevKFIdx);
                    new_landmark_flags.push_back(false);
                    current_measurements.push_back(acceptedCurMeas);
                    cur_compcam_ids.push_back(acceptedCurCamIDs);
                    / record the previous measurements and previous KF and comp cam ID
                    previous_measurements.push_back(acceptedPrevMeas);
                    previous_compcam_ids.push_back(acceptedPrevCamIDs);
                    */
                    insertLandmarkInGraph(l, prevKFIdx, curKFIdx, false, acceptedPrevCamIDs,
                                          acceptedPrevMeas, acceptedCurCamIDs, acceptedCurMeas, acceptedPrevKPOctaves,
                                          acceptedCurKPOctaves);

                }

            }

        }
    }


    if(true){
        reinitialized_ = false;

        VLOG(0) << "size of kfSetOlder: " << kfSetOlder.size() << endl;
        //VLOG(0) << "size of ordered map keyframe_data: " << keyframe_data.size() << endl;

        for(auto kf : kfSetOlder){

            //CHECK: which kfs are being added to see if we have to update the prevVel in insert prior after the LIS error

            int prevStateID = kf->kfID;
            cv::Mat tmp_ = kf->pose * frontEnd->Tbc_cv.inv();

            // cv::Mat pose = kf->pose.clone();


            gtsam::Pose3 previous_pose = convertPose3_CV2GTSAM(tmp_);
            /// Add a prior on pose x0
            gtsam::noiseModel::Diagonal::shared_ptr poseNoise = gtsam::noiseModel::Diagonal::Sigmas
                    ((gtsam::Vector(6)<< gtsam::Vector3::Constant(0.01),gtsam::Vector3::Constant(0.01)).finished());
            graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', prevStateID), previous_pose, poseNoise));
            VLOG(1)<<"backend: Inserted prior for the state x"<<prevStateID<<endl;

        }


    }
    VLOG(0) << " - - -  CurrKFID - " << frontEnd->currentFrame->kfID << std::endl;
    // addLoopClosureFactor();
    // VLOG(3)<<"Backend Landmarks size: "<<num_lms_filtered<<endl;

    // Are we resetting if we have less landmarks? What does this do?
    if(num_lms_filtered < 15){

        VLOG(0)<<"LESS than 15 Landmarks .Cklearing out everything";
        VLOG(0) << "RESETTING THE BACKEND" << endl;
//        graph.resize(0);
//        initialEstimate.clear();
//        newTimeStamps.clear();
        return false;
    }

    if(frontEnd->useIMU){
        if(prev_Kf!= -1){

            if(frontEnd->useGPS && frontEnd->imu_message_empty==true){
                frontEnd->imu_message_empty = false;
            }
            else{
                addIMUFactor(prev_Kf, current_pose_id);
            }
        }

    }

//    if(frontEnd->useGPS){
//        assert(frontEnd->gps_initialized == true);
//        if(frontEnd->gps_msgs_combined.size() > 0){
//            addGPSFactor(frontEnd->gps_msgs_combined, current_pose_id);
//        }
//    }

    if(num_lms_filtered < 15){

        VLOG(0)<<"LESS than 15 Landmarks .Cklearing out everything";
        VLOG(0) << "RESETTING THE BACKEND" << endl;
//        graph.resize(0);
//        initialEstimate.clear();
//        newTimeStamps.clear();
//        return false;
    }


    // VLOG(3)<<"graph size: "<<graph.size()<<endl;
    if(optimizationMethod != 2){
        if (windowCounter%windowSize != 0)
            ret = false;
    }
    windowCounter++;
    // VLOG(3)<<"add keyframe optimize flag: "<<ret<<endl;

    return ret;

}


tuple<double, double, double> Backend::processGPS_NED(sensor_msgs::NavSatFix gps_msg){


    // convert to ENU frame
    tuple<double, double, double> gps_message_enu = frontEnd->geodetic_to_enu(gps_msg);

    tuple<double, double, double> gps_pose = frontEnd->enu_wrt_origin(gps_message_enu);

    Mat gps_pose_mat = (Mat_<double>(4,1) << get<0>(gps_pose), get<1>(gps_pose), get<2>(gps_pose), 1);

    Mat gps_pose_1_body = frontEnd->T_body_gps * gps_pose_mat;

    tuple<double, double, double> gps_pose_w = make_tuple(gps_pose_1_body.at<double>(0), gps_pose_1_body.at<double>(1), gps_pose_1_body.at<double>(2));


    // convert to NED frame
    double gps_ned_x = get<1>(gps_pose_w);
    double gps_ned_y = get<0>(gps_pose_w);
    double gps_ned_z = -get<2>(gps_pose_w);

    tuple<double, double, double> gps_pose_ned = make_tuple(gps_ned_x, gps_ned_y, gps_ned_z);

    return gps_pose_ned;

}

tuple<double, double, double> Backend::processGPS_ENU(sensor_msgs::NavSatFix gps_msg){


    // convert to ENU frame
    tuple<double, double, double> gps_message_enu = frontEnd->geodetic_to_enu(gps_msg);

    VLOG(0) <<" gps message in ENU: " << get<0>(gps_message_enu) <<" " << get<1>(gps_message_enu) << " " << get<2>(gps_message_enu);

    return gps_message_enu;

}

double Backend::euclidean(tuple<double, double, double> gps_msg1, tuple<double, double, double> gps_msg2){

   return sqrt(pow(get<0>(gps_msg1) - get<0>(gps_msg2), 2) + pow(get<1>(gps_msg1) - get<1>(gps_msg2), 2) + pow(get<2>(gps_msg1) - get<2>(gps_msg2), 2));
}




void Backend::addGPSFactor(std::deque<sensor_msgs::NavSatFix> &gps_msgs_combined, int current_pose_id ){


    /*
     * Store the E_T_V in the backend after the GPS kabsch
     * initialization procedure has been performed
     */
    if(use_E_T_V_init_est == false){
        Tbg = convertPose3_CV2GTSAM(frontEnd->Tbg);
        cv::Mat gps_T_body = (frontEnd->T_body_gps).inv();
        E_T_V = convertPose3_CV2GTSAM(gps_T_body);
        use_E_T_V_init_est = true;
    }

    sensor_msgs::NavSatFix gps_msg = gps_msgs_combined.front();

    // GPS measurement noise
    auto correction_noise = noiseModel::Diagonal::Sigmas(Vector3(1, 1, 1));

    // GPS message in ENU frame
    tuple<double, double, double> gps_message_enu = processGPS_ENU(gps_msg);

    // add a weak prior over the transformation variable
    if(debug_prior){
        graph.addPrior(gtsam::Symbol('t', 0), E_T_V, noiseModel::Diagonal::Sigmas(
                (Vector(6) << 1e-1, 1e-1, 1e5, 1e5, 1e5, 1e5)
                        .finished()));
        debug_prior = false;
        initialEstimate.insert(gtsam::Symbol('t', 0), E_T_V);
    }

    newGPSFactor gps_factor(gtsam::Symbol('x', current_pose_id), gtsam::Point3(get<0>(gps_message_enu),
                                                                               get<1>(gps_message_enu),
                                                                               get<2>(gps_message_enu)), gtsam::Symbol('t', 0), correction_noise, Tbg);
    graph.push_back(gps_factor);
    added_E_T_V_init_est = true;

    VLOG(0) << "!============!" << endl;
    VLOG(0) <<" added gps factor connecting x" << current_pose_id << " and t0";
    VLOG(0) << "!============!" << endl;
    gps_factor_count++;

    VLOG(0) << "GPS factor count: " << gps_factor_count << endl;

    gps_msgs_combined.pop_front();

    return;
}

void Backend::addBetweenFactor_Poses(int prev_kf, int curr_kf){

    VLOG(0) << "Adding between factor between poses x" << prev_Kf <<  " and  x" << curr_kf;

    if(!frontEnd->useGPS){
        VLOG(0) <<"This situation should occur when we use a GPS. GPS flag not turned on";
        exit(0);
    }

    gtsam::noiseModel::Diagonal::shared_ptr between_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << gtsam::Vector3::Constant(0.1),
                    gtsam::Vector3::Constant(0.5)).finished());

    gtsam::BetweenFactor<gtsam::Pose3> pose_between_factor(prev_Kf, curr_kf, Pose3(), between_noise);

    graph.push_back(pose_between_factor);

    VLOG(0) << "Added between factor between poses x" << prev_Kf <<  " and  x" << curr_kf;


}

void Backend::change_vision_kf_info(){

    // change vision kf info to prev gps kf so that landmarks will get attached to that.
    if(frontEnd->gps_initialized == false){
        return;
    }

    if(frontEnd->imu_message_empty == false){
        return;
    }


    // recent vision kf
    MultiCameraFrame *curr_vision_frame = frontEnd->lfFrames[frontEnd->lfFrames.size() - 1];

    // the gps kf before that
    MultiCameraFrame *prev_gps_frame = frontEnd->lfFrames[frontEnd->lfFrames.size() - 2];

    int gps_kfid = prev_gps_frame->kfID;
    int curr_kfid = curr_vision_frame->kfID;

    /*
     * change the gps kf to vision kf
     * so that the tracking can be done later
     */

     curr_vision_frame->kfID = gps_kfid;
//     curr_vision_frame->pose = prev_gps_frame->pose;
     VLOG(0) <<" changed vision kf id from " << curr_kfid <<" -> " << gps_kfid;

     // erase the gps kf from lf frames
     frontEnd->lfFrames.erase(frontEnd->lfFrames.end() - 2);
     delete prev_gps_frame;

     // remove from other places like allposes_dummy, combined_tstamps and combined_poses
    frontEnd->allPoses_dummy.pop_back();
    frontEnd->combined_tStamps.erase(frontEnd->combined_tStamps.end() - 2);
    frontEnd->combined_poses.erase( frontEnd->combined_poses.end() - 2);

    // we are removing this kf in backend. So, it a kf with the currentKFID need not exist anymore.
    frontEnd->currentKFID--;
    return;
}

void Backend::addIMUFactor(int prev_kf, int curr_kf){

    VLOG(0)<<"Adding IMU factor between "<<prev_kf<<" and "<<curr_kf <<endl;
    if(prev_kf == -1){
        VLOG(0)<<"First KF, no IMU factor only adding initial estimates"<<endl;
        if (!initialEstimate.exists(gtsam::Symbol('b', curr_kf))) {
            initialEstimate.insert(gtsam::Symbol('b', curr_kf), prevBias_);
        }
        if (!initialEstimate.exists(gtsam::Symbol('v', curr_kf))) {
            initialEstimate.insert(gtsam::Symbol('v', curr_kf), prevVel_);
        }
        return;
    }

    //combined imu factors
    auto preint_imu_combined = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(*frontEnd->imu_integrator_comb);
    gtsam::CombinedImuFactor imu_factor(gtsam::Symbol('x', prev_kf ), gtsam::Symbol('v', prev_kf ),
                                        gtsam::Symbol('x', curr_kf ), gtsam::Symbol('v', curr_kf ),
                                        gtsam::Symbol('b', prev_kf ), gtsam::Symbol('b', curr_kf ), preint_imu_combined);
    graph.push_back(imu_factor);


    // print the predicted state
    VLOG(0) << "prev Bias while adding imu factor \n" << prevBias_ << endl;
    VLOG(0) << "prev State while adding imu factor for state \n" << prevState_ << endl;

    gtsam::NavState next_state = frontEnd->imu_integrator_comb->predict(prevState_, prevBias_);


    VLOG(0) << "Velocity in Nav frame - " << curr_kf << " is : \n" << next_state.v() << endl;
    VLOG(0) << "Velocity in Body Frame - " << curr_kf << "is :\n" << next_state.bodyVelocity() << endl;
    VLOG(0) << "Initial estimate for bias b" << curr_kf << "is :  \n" << prevBias_ << endl;
    VLOG(0) << "Pose predicted for x" << curr_kf << " is: \n" << next_state.pose() << endl;

    VLOG(0) << "Predicted estimate for x" << curr_kf << " is: \n"<<next_state.pose()<<endl;
    VLOG(0) <<" lf frames pose is : \n" << frontEnd->lfFrames[frontEnd->lfFrames.size() - 1]->pose << endl;

    Mat currentPoseMat = frontEnd->lfFrames[frontEnd->lfFrames.size() - 1]->pose * frontEnd->Tbc_cv.inv();
    gtsam::Pose3 currentPose = convertPose3_CV2GTSAM(currentPoseMat);
    if(!initialEstimate.exists(gtsam::Symbol('x', curr_kf))){
        VLOG(0) << "next estimate sent from IMU!!" ;
//        initialEstimate.insert(gtsam::Symbol('x', curr_kf), next_state.pose());
        initialEstimate.insert(gtsam::Symbol('x', curr_kf), currentPose);
    }

    initialEstimate.insert(gtsam::Symbol('v', curr_kf), next_state.v());
    initialEstimate.insert(gtsam::Symbol('b', curr_kf), prevBias_);

}


/// function for monocular backend
/// Add the factors for the new keyframe and the landmarks
bool Backend::addKeyFrame() {
//Backend::addKeyFrame(cv::Mat &pose, vector<cv::KeyPoint> &keyPoints, vector<cv::Mat> &landmarks, int &frame_id, vector<int>* lids){
    MultiCameraFrame* currentFrame = frontEnd->currentFrame;
    GlobalMap* map = frontEnd->map;
    bool ret = true;

    // VLOG(3)<<"Window counter: "<<windowCounter<<endl;

    vector<int> lids_filtered;
    vector<int> previous_pose_ids;
    int current_pose_id = currentFrame->frameId;

    vector<gtsam::Point3> landmarks;
    vector<gtsam::Point2> current_measurements;
    vector<gtsam::Point2> previous_measurements;

    gtsam::Pose3 current_pose = convertPose3_CV2GTSAM(currentFrame->pose);
    current_pose = calCompCamPose(current_pose);

//    filterLandmarks(currentFrame, map, lids_filtered, landmarks, current_measurements, previous_measurements, previous_pose_ids);
    filterLandmarksStringent(currentFrame, map, lids_filtered, landmarks, current_measurements, previous_measurements, previous_pose_ids);

    // VLOG(3)<<"Backend Landmarks size: "<<landmarks.size()<<endl;
    if(landmarks.size() < 12){
        // VLOG(3)<<"skipping key frame: "<<current_pose_id<<endl<<endl;
        windowCounter--;
        return false;
    }

    /// check if this is the first frame we are inserting
    /// if it is, we have to add prior to the pose and a landmark
    if(currentEstimate.empty() && initialEstimate.empty()) {

        // VLOG(3)<<"Went inside GRAPH==0"<<endl;
        addPosePrior(lids_filtered.at(0), map);
        addLandmarkPrior(lids_filtered.at(0), landmarks[0]);
    }

    initialEstimate.insert(gtsam::Symbol('x', current_pose_id), current_pose);

    /// create graph edges
    /// insert initial values for the variables
    for (int i = 0; i < landmarks.size(); ++i) {
        /// for each measurement add a projection factor

        if(previous_pose_ids[i] != -1){ // Newly tracked landmark

//            if(!(currentEstimate.exists(gtsam::Symbol('x', previous_pose_ids[i])) || initialEstimate.exists(gtsam::Symbol('x', previous_pose_ids[i])))){
//
//                //Landmark's previous frame is not inserted in backend, skip landmark
//                VLOG(3)<<"Skipping landmark lid, frameid: "<<lids_filtered.at(i)<<" "<<previous_pose_ids[i]<<" "<<current_pose_id<<endl;
//                continue;
//            }

            // Insert projection factor with previous kf to factor graph
            // VLOG(3)<<"measurement from prev frame: "<<previous_measurements[i].x()<<" "<<previous_measurements[i].y()<<endl;
            graph.push_back(gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>
                                    (previous_measurements[i], measurementNoise, gtsam::Symbol('x', previous_pose_ids[i]), gtsam::Symbol('l', lids_filtered.at(i)), K[camID]));
            initialEstimate.insert(gtsam::Symbol('l', lids_filtered.at(i)), landmarks[i]);
        }

        // insert projection factor with current frame to factor graph
        graph.push_back(gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>
                                    (current_measurements[i], measurementNoise, gtsam::Symbol('x', current_pose_id), gtsam::Symbol('l', lids_filtered.at(i)), K[camID]));
    }
    // VLOG(3)<<"graph size: "<<graph.size()<<endl;
    if (windowCounter%windowSize != 0)
        ret = false;
    // VLOG(3)<<"add keyframe optimize flag: "<<ret<<endl;
    windowCounter++;
    return ret;
}

bool Backend::addKeyFrameNov(){

    MultiCameraFrame *currentFrame = frontEnd->currentFrame;
    GlobalMap *map = frontEnd->map;
    bool ret = true;

    VLOG(1) << "Window counter: " << windowCounter << endl;

    /// Variable declaration
    int num_lms_filtered=0;
    vector<int> lids_filtered;
    vector<gtsam::Point3> landmarks;
    vector<vector<gtsam::Point2>> current_measurements;
    vector<vector<gtsam::Point2>> previous_measurements;
    vector<vector<int>>  previous_compcam_ids, cur_compcam_ids;
    vector<MultiCameraFrame*> previous_KFs;
    vector<int> prevKFInds;
    vector<bool> new_landmark_flags;

    ///get current frame intramatches and landmarks
    vector<IntraMatch> *intramatches = &currentFrame->intraMatches;
    vector<int> *lids = &currentFrame->lIds;
    int current_pose_id = currentFrame->kfID;

    std::set<MultiCameraFrame*> kfSetOlder;

    /// browse through all intramatches
    for (int i = 0; i < intramatches->size(); ++i) {
        /// if this intra match is a landmark i.e it is tracked in the current frame and is a triangulated inlier
        if (lids->at(i) != -1 ) {

            //get the landmark and it 3D coordinate
            Landmark* l = map->getLandmark(lids->at(i));
            gtsam::Point3 landmark = convertPoint3_CV2GTSAM(l->pt3D);
            ///get the landmark's keyframes
            int numKFs = l->KFs.size();
            ///if the landmark is newly triangulated add factors between the current and previous keyframes
            if(!currentEstimate.exists(gtsam::Symbol('l', lids->at(i))) and
               !initialEstimate.exists(gtsam::Symbol('l', lids->at(i))))
            {
                 if(numKFs <2){
                     VLOG(2)<<"This landmarks has less than 2 keyframes";
//                     continue;
                 }

                /// Get the previous LF frame of the landmark and the observations in the component cameras
                bool firstPair = true;
                for (int pairIdx = (numKFs-1) ; pairIdx< numKFs ; pairIdx++){
                    int prevKFIdx =  pairIdx-1;     //(numKFs-2);
                    int curKFIdx =   pairIdx;       //numKFs-1;
                    MultiCameraFrame* lf1 = l->KFs[prevKFIdx]; // previous KF
                    MultiCameraFrame* lf2 = l->KFs[curKFIdx];  // same as l->KFs[numKFs-1]; // last KF
                    IntraMatch* im1 = &lf1->intraMatches.at(l->featInds[prevKFIdx]);
                    IntraMatch* im2 = &lf2->intraMatches.at(l->featInds[curKFIdx]);


                    ///check all combinations of rays and get the triangulation angles and reprojection errors
                    vector<int> acceptedPrevCamIDs, acceptedCurCamIDs, acceptedPrevKPOctaves, acceptedCurKPOctaves;
                    vector<gtsam::Point2> acceptedPrevMeas, acceptedCurMeas;
                    for (int camID1 = 0; camID1 < lf1->num_cams_ ; camID1++) {
                        int kp_ind1 = im1->matchIndex[camID1];
                        if (kp_ind1 == -1)
                            continue;
                        acceptedPrevCamIDs.push_back(camID1);
                        acceptedPrevMeas.push_back(convertPoint2_CV2GTSAM(lf1->image_kps_undist[camID1][im1->matchIndex[camID1]]));
                        acceptedPrevKPOctaves.push_back(lf1->image_kps_undist[camID1][im1->matchIndex[camID1]].octave);
                    }
                    for (int camID2 = 0; camID2 < lf2->num_cams_ ; camID2++) {
                        int kp_ind2 = im2->matchIndex[camID2];
                        if (kp_ind2 == -1)
                            continue;
                        acceptedCurCamIDs.push_back(camID2);
                        acceptedCurMeas.push_back(convertPoint2_CV2GTSAM(lf2->image_kps_undist[camID2][im2->matchIndex[camID2]]));
                        acceptedCurKPOctaves.push_back(lf2->image_kps_undist[camID2][im2->matchIndex[camID2]].octave);
                    }

                    if(firstPair){
                        firstPair  = false;
                        num_lms_filtered++;
                        if(num_lms_filtered == 1){
                            insertPriors(lids->at(i));
                        }
                        else{
                            kfSetOlder.insert(lf1);
                        }


                        insertLandmarkInGraph(l, prevKFIdx, curKFIdx, true, acceptedPrevCamIDs,
                                              acceptedPrevMeas, acceptedCurCamIDs, acceptedCurMeas, acceptedPrevKPOctaves,
                                              acceptedCurKPOctaves);

                    }
                    else{
                        insertLandmarkInGraph(l, prevKFIdx, curKFIdx, false, acceptedPrevCamIDs,
                                              acceptedPrevMeas, acceptedCurCamIDs, acceptedCurMeas, acceptedPrevKPOctaves,
                                              acceptedCurKPOctaves);
                        VLOG(2)<<"NEVER COMES HERE";

                    }


                }

            }
            else{
                /// this is an existing landmark in the graph.
                /// Grab the last observation
                std::map<int, vector<int>> facs = lmFactorRecordMulti[lids->at(i)];
                VLOG(3)<<"Existing Landmark"<<endl;
                assert(facs.size() != 0);

                /// Get the prev factor's frameID and componenet camera IDs
                vector<int> prevRaysCamIds = facs.rbegin()->second;
                int prevKFIdx = facs.rbegin()->first;
                int curKFIdx = numKFs-1;

                MultiCameraFrame* lf2 = currentFrame;  // same as l->KFs[numKFs-1]; // last KF
                IntraMatch* im2 = &lf2->intraMatches.at(l->featInds.back());

                // compare all the angles and choose the  largest one for insertion
                //first check the diagonal angles i.e intersecting component cameras between two LF frame
                vector<int> acceptedPrevCamIDs, acceptedCurCamIDs, acceptedPrevKPOctaves, acceptedCurKPOctaves;
                vector<gtsam::Point2> acceptedPrevMeas, acceptedCurMeas;
                for (int camID2 = 0; camID2 < lf2->num_cams_ ; camID2++) {
                    int kp_ind2 = im2->matchIndex[camID2];
                    if (kp_ind2 == -1)
                        continue;
                    acceptedCurCamIDs.push_back(camID2);
                    acceptedCurMeas.push_back(convertPoint2_CV2GTSAM(lf2->image_kps_undist[camID2][im2->matchIndex[camID2]]));
                    acceptedCurKPOctaves.push_back(lf2->image_kps_undist[camID2][im2->matchIndex[camID2]].octave);
                }

                num_lms_filtered++;
                insertLandmarkInGraph(l, prevKFIdx, curKFIdx, false, acceptedPrevCamIDs,
                                          acceptedPrevMeas, acceptedCurCamIDs, acceptedCurMeas, acceptedPrevKPOctaves,
                                          acceptedCurKPOctaves);

            }

        }
    }


    if(reinitialized_){
        reinitialized_ = false;
        for(auto kf : kfSetOlder){
            int prevStateID = kf->kfID;
            cv::Mat prev_pose_ = kf->pose * frontEnd->Tbc_cv.inv();
            gtsam::Pose3 previous_pose = convertPose3_CV2GTSAM(prev_pose_);
            /// Add a prior on pose x0
            gtsam::noiseModel::Diagonal::shared_ptr poseNoise = gtsam::noiseModel::Diagonal::Sigmas
                    ((gtsam::Vector(6)<< gtsam::Vector3::Constant(0.01),gtsam::Vector3::Constant(0.01)).finished());
            graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', prevStateID), previous_pose, poseNoise));
            VLOG(3)<<"Inserted prior for the state x"<<prevStateID<<endl;
        }
    }
    if(frontEnd->useIMU){
        if(prev_Kf!= -1){

            if(frontEnd->useGPS && frontEnd->imu_message_empty==true){
                frontEnd->imu_message_empty = false;
            }
            else{
                addIMUFactor(prev_Kf, current_pose_id);
            }
        }

    }


    VLOG(2)<<"Backend Landmarks size: "<<num_lms_filtered<<endl;
    if(num_lms_filtered < 15){
        graph.resize(0);
        initialEstimate.clear();
        newTimeStamps.clear();
        return false;
    }

    VLOG(2)<<"graph size: "<<graph.size()<<endl;
    if (windowCounter%windowSize != 0)
        ret = false;
    VLOG(2)<<"add keyframe optimize flag: "<<ret<<endl;
    windowCounter++;
    VLOG(2)<<"add keyframe optimize flag: "<<ret<<endl;

    return ret;

}



bool Backend::optimizePosesLandmarks() {
    if(optimizationMethod == 0){
        ISAM2 isam2 = ISAM2(isam);
        try {

            VLOG(0) << "isam update";
            VLOG(0) << "Total successful isam updates " << total_succ_isam;
            VLOG(0) << "Total isam ILS errors = " << total_isam_errors << endl;

            // store 3 gps kfs before we do our first isam update
            if(frontEnd->useGPS){

                if(gps_factor_count > 0 && gps_factor_count < 3 && frontEnd->gps_initialized){

                    VLOG(0) << " not performing isam update as GPS factor count is " << gps_factor_count;
                    // wait until the transform variable is constrained by at least 3 poses.
                    prev_Kf = frontEnd->currentKFID - 1; // change the prev kf

                    if(initialEstimate.exists(gtsam::Symbol('x', prev_Kf))){
                        prevBias_ = initialEstimate.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', prev_Kf));
                        prevVel_ = initialEstimate.at<gtsam::Vector3>(gtsam::Symbol('v', prev_Kf));
                    }
                    prevState_ = gtsam::NavState(prevPose_, prevVel_);

                    // reset imu preintegration object
                    frontEnd->imu_integrator_comb->resetIntegrationAndSetBias(prevBias_);
                    return false;
                }
            }

            VLOG(0) << " graph size " << graph.size() << " initial values size " << initialEstimate.size() << endl;
//            VLOG(0) << "initial estimate for x" << frontEnd->currentKFID - 1 << " is : " << initialEstimate.at<gtsam::Pose3>(gtsam::Symbol('x', frontEnd->currentKFID - 1)) << endl;

            auto start = std::chrono::high_resolution_clock::now();


            for(auto &index : toBeRemovedFactorIndices) {
                VLOG(0) << index;

                // check if the index exsists in toBeRemovedFactorIndices_test
                if(std::find(toBeRemovedFactorIndices_test.begin(), toBeRemovedFactorIndices_test.end(), index) != toBeRemovedFactorIndices_test.end()) {
                    VLOG(0) << "Found an index that was deleted previously " << index;
                }


                toBeRemovedFactorIndices_test.push_back(index);
            }


            // update the ISAm parameters to send newly affected keys due to adding measurements
            // to smart factors and also to be removed indices
            ISAM2UpdateParams updateParams;
            updateParams.newAffectedKeys = std::move(factorNewAffectedKeys);
            updateParams.removeFactorIndices = toBeRemovedFactorIndices;

            auto result = isam.update(graph, initialEstimate, updateParams);

            //From the result grab the indices of the newly added factors in the isam's factor graph
            // this will come in handy when we want to refer to them later
            for (const auto &f2l : newFactor_to_lm)
                lm_to_facInds[f2l.second] = result.newFactorsIndices.at(f2l.first);


//            for(auto& lid : deleted_landmarks){
//                smartFactor.erase(lid);
//
//            }
//            deleted_landmarks.clear();


            auto  stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
            VLOG(0) << "isam update time: " << duration.count() << endl;

            graph.resize(0);
            // graph.print("\nFactor Graph:\n");
            initialEstimate.clear();
            toBeRemovedFactorIndices.clear();
            newTimeStamps.clear();
            // Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
            // If accuracy is desired at the expense of time, update(*) can be called additional times
            // to perform multiple optimizer iterations every step.

            auto start2 = std::chrono::high_resolution_clock::now();
            currentEstimate = isam.calculateBestEstimate();
            auto  stop2 = std::chrono::high_resolution_clock::now();
            auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(stop2 - start2);
            VLOG(0) << "isam calculate best estimate time: " << duration2.count() << endl;


            prev_Kf = frontEnd->currentKFID - 1;
            prevPose_ = currentEstimate.at<gtsam::Pose3>(gtsam::Symbol('x', prev_Kf));
            VLOG(0) << "current Estimate for x" << prev_Kf << " is : " << prevPose_ << endl;
            if(frontEnd->useIMU){


                prevBias_ = currentEstimate.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', prev_Kf));
                prevVel_ = currentEstimate.at<gtsam::Vector3>(gtsam::Symbol('v', prev_Kf));
                prevState_ = gtsam::NavState(prevPose_, prevVel_);

                // reset imu preintegration object
                frontEnd->imu_integrator_comb->resetIntegrationAndSetBias(prevBias_);

                VLOG(0) << "current Estimate for v" << prev_Kf << " is : " << prevVel_ << endl;
                VLOG(0) << "current Estimate for b" << prev_Kf << " is : " << prevBias_ << endl;

            }
            if(frontEnd->useGPS){
                // this if condition exists because the transform varible would suddenly not exist if
                // the graph got chopped off.
                if(added_E_T_V_init_est){
                    E_T_V = currentEstimate.at<gtsam::Pose3>(gtsam::Symbol('t', 0));
                    VLOG(0) <<"current Estimate for t " << endl << E_T_V;
                    added_E_T_V_init_est  = false;
                }
            }

            VLOG(0) << "BackEnd Optimization done" << endl;
            total_succ_isam ++;

        }

        catch (IndeterminantLinearSystemException& e)
        {
            VLOG(3)<<"Near by variable: "<<e.nearbyVariable();
            Symbol sym = gtsam::Symbol(e.nearbyVariable());
            if(sym.chr() == 'l'){
                // it is  landmark.
                //extract the edges corresponding to that landmark

                map<int, vector<int>> lmFactorRecord = lmFactorRecordMulti[sym.index()];
                Landmark* l = frontEnd->map->getLandmark(sym.index());

//                Landmark* nearby_l = frontEnd->map->getLandmark(sym.index()-1);
                VLOG(0)<<"lid : "<<sym.index()<<" Pt: "<<l->pt3D;
//                VLOG(0)<<"lid : "<<sym.index()-1<<" Pt: "<<nearby_l->pt3D;
                for(map<int, vector<int>>::iterator itr_l = lmFactorRecord.begin() ; itr_l != lmFactorRecord.end() ; ++itr_l){
                    int iD = (*itr_l).first;
                    VLOG(0)<<"xid : "<<l->KFs[iD]->kfID;
                    vector<int> xids = (*itr_l).second;
                    for(auto& x : xids)
                    {
                        VLOG(0)<<"comp cam id :"<< x;
//                        Mat img ;

//                        if(!l->KFs[iD]->imgs[x].empty()){
//                            cvtColor( l->KFs[iD]->imgs[x],img , COLOR_GRAY2BGR);
//                            IntraMatch im = l->KFs[iD]->intraMatches[l->featInds[iD]];
//                            circle(img, l->KFs[iD]->image_kps_undist[x][im.matchIndex[x]].pt, 3,Scalar(0,200,0), 3);
//                            imshow("error lm", img);
//                            waitKey(0);
//                        }
                    }

                }

            }

            /////////////////////////////////////////////////////////////////////
            //// This is purely damage control as I am not able to figure out
            //// why isam2 is giving indeterminant linea system error
            /////////////////////////////////////////////////////////////////////
            int fr_ind=0;
            for (auto& fr : frontEnd->lfFrames){
                int poseid = fr->kfID;
                if(currentEstimate.exists(gtsam::Symbol('x', poseid))){
                    try{
                       // gtsam::Matrix  covgtsam = isam2.marginalCovariance(gtsam::Symbol('x', poseid));
                       // cv::eigen2cv(covgtsam, fr->cov);
                    }
                    catch (IndeterminantLinearSystemException& e) {
                        VLOG(3)<<"Exception occured in marginal covariance computation:"<<endl;
                        fr->cov = Mat::eye(6,6, CV_64F);
                    }

                    //VLOG(3)<<"Covariance"<<isam.marginalCovariance(gtsam::Symbol('x', poseid));

                }
                fr_ind++;
            }

            //debug
            total_isam_errors ++ ;
            VLOG(0) << "current isam ILS errors = " << total_isam_errors << endl;


            isam.clear();
            lmFactorRecordMulti.clear();
            xFactorRecord.clear();
            currentEstimate.clear();
            toBeRemovedFactorIndices.clear();
            newTimeStamps.clear();
            graph.resize(0);
            initialEstimate.clear();
            currentEstimate.clear();

            isam = gtsam::ISAM2(parameters);
            reinitialized_ = true;
            last_reset_kfid = frontEnd->currentFrame->kfID;
            windowCounter=0;
            debug_prior = true;
            added_E_T_V_init_est  = false;
            gps_factor_count=0;
            VLOG(0) << "Exception caught : " << e.what() << std::endl;
            VLOG(0)<<"Resetting ISAM";
            return false;
        }
    }

    else if(optimizationMethod == 1){
        VLOG(3)<<"LM Optimization"<<endl;
//         graph.print("graph:");
//        initialEstimate.print("initial estimate");
        optimizer = new gtsam::LevenbergMarquardtOptimizer(graph, initialEstimate, params);
        // print the graph error before optimization
        VLOG(0) << "Number of pose nodes in graph : " << poseCounter << endl;
        VLOG(0) << "Initial Error = " << graph.error(initialEstimate) << endl;
        currentEstimate = optimizer->optimize();
        // print the summary of the optimization
//        VLOG(0) << "Optimizer Summary:\n" << optimizer->print() << endl;
        // print the graph error after optimization
        VLOG(0) << "Final Error = " << graph.error(currentEstimate) << endl;
        delete optimizer;

        if (frontEnd->useIMU) {
            // update the previous state
            prev_Kf = frontEnd->currentFrame->kfID;
            if (currentEstimate.exists(gtsam::Symbol('x', prev_Kf))) {
                VLOG(0) << "current Estimate for x" << prev_Kf << " is : "
                        << currentEstimate.at<gtsam::Pose3>(gtsam::Symbol('x', prev_Kf)) << endl;
                prevPose_ = currentEstimate.at<gtsam::Pose3>(gtsam::Symbol('x', prev_Kf));
            }
            else {
                prevPose_ = initialEstimate.at<gtsam::Pose3>(gtsam::Symbol('x', prev_Kf));
            }
            if (currentEstimate.exists(gtsam::Symbol('b', prev_Kf))) {
                VLOG(0) << "current Estimate for b" << prev_Kf << " is : "
                        << currentEstimate.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', prev_Kf)) << endl;
                prevBias_ = currentEstimate.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', prev_Kf));
//                            prevBias_ = currentEstimate.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', prev_Kf));
            }
            else {
                prevBias_ = prevBias_;
            }
            if (currentEstimate.exists(gtsam::Symbol('v', prev_Kf))) {
                VLOG(0) << "current Estimate for v" << prev_Kf << " is : "
                        << currentEstimate.at<gtsam::Vector3>(gtsam::Symbol('v', prev_Kf)) << endl;
                prevVel_ = currentEstimate.at<gtsam::Vector3>(gtsam::Symbol('v', prev_Kf));
//                            prevVel_ = currentEstimate.at<gtsam::Vector3>(gtsam::Symbol('v', prev_Kf));
            }
            else {
                prevVel_ = prevVel_;
            }
            prevState_ = gtsam::NavState(prevPose_, prevVel_);
            // reset imu preintegration object
            frontEnd->imu_integrator_comb->resetIntegrationAndSetBias(prevBias_);
        }
    }
    else if(optimizationMethod == 2){
        VLOG(0) << "Fixed Lag Smoother" << endl;
        // perform fixed lag smoothing only if graph size is greater than 2
        if (frontEnd->lfFrames.size() > 1) {
            if (windowCounter >= 2) {
                try {
//                VLOG(0) << "Fixed Lag Smoother update - Graph initial error " << graph.error(initialEstimate) << endl;
                    fixedLagOptimizer.update(graph, initialEstimate, newTimeStamps);
                    graph.resize(0);
                    initialEstimate.clear();
                    newTimeStamps.clear();
                    VLOG(0) << "Number of factors in graph : " << fixedLagOptimizer.getFactors().size();
                    // Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
                    // If accuracy is desired at the expense of time, update(*) can be called additional times
                    // to perform multiple optimizer iterations every step.
//                isam.update();

//                    currentEstimate = fixedLagOptimizer.calculateEstimate();
//                VLOG(0) << "Fixed Lag Smoother update - Graph final error " << graph.error(currentEstimate) << endl;

                    if (frontEnd->useIMU && prev_Kf > -1) {
                        // reset imu preintegration object
                        frontEnd->imu_integrator_comb->resetIntegrationAndSetBias(prevBias_);
                        // update the previous state
                        prev_Kf = frontEnd->currentFrame->kfID;
//                        if (currentEstimate.exists(gtsam::Symbol('x', prev_Kf)))
                            prevPose_ = fixedLagOptimizer.calculateEstimate<gtsam::Pose3>(gtsam::Symbol('x', frontEnd->currentFrame->kfID));
//                        if (currentEstimate.exists(gtsam::Symbol('b', prev_Kf)))
                            prevBias_ = fixedLagOptimizer.calculateEstimate<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', frontEnd->currentFrame->kfID));
//                            prevBias_ = currentEstimate.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', prev_Kf));
//                        if (currentEstimate.exists(gtsam::Symbol('v', prev_Kf)))
                            prevVel_ = fixedLagOptimizer.calculateEstimate<gtsam::Vector3>(gtsam::Symbol('v', frontEnd->currentFrame->kfID));
//                            prevVel_ = currentEstimate.at<gtsam::Vector3>(gtsam::Symbol('v', prev_Kf));
                        prevState_ = gtsam::NavState(prevPose_, prevVel_);
                    } else if (prev_Kf == -1) {
                        prev_Kf = frontEnd->currentKFID - 1;
                        prevPose_ = currentEstimate.at<gtsam::Pose3>(gtsam::Symbol('x', prev_Kf));
                    }

                }
                catch (IndeterminantLinearSystemException &e) {
                    Symbol sym = gtsam::Symbol(e.nearbyVariable());
                    if (sym.chr() == 'l') {

                        VLOG(0) << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
                        // it is  landmark.
                        //extract the edges corresponding to that landmark

                        map<int, vector<int>> lmFactorRecord = lmFactorRecordMulti[sym.index()];
                        Landmark *l = frontEnd->map->getLandmark(sym.index());
                        VLOG(3) << "lid : " << sym.index() << " Pt: " << l->pt3D;
                        for (map<int, vector<int>>::iterator itr_l = lmFactorRecord.begin();
                             itr_l != lmFactorRecord.end(); ++itr_l) {
                            int iD = (*itr_l).first;
                            VLOG(3) << "xid : " << l->KFs[iD]->kfID;
                            vector<int> xids = (*itr_l).second;
                            for (auto &x: xids) {
                                VLOG(3) << "comp cam id :" << x;
                                Mat img;

                                if (!l->KFs[iD]->imgs[x].empty()) {
                                    cvtColor(l->KFs[iD]->imgs[x], img, COLOR_GRAY2BGR);
                                    IntraMatch im = l->KFs[iD]->intraMatches[l->featInds[iD]];
                                    circle(img, l->KFs[iD]->image_kps_undist[x][im.matchIndex[x]].pt, 3,
                                           Scalar(0, 200, 0), 3);
                                    // imshow("error lm", img);
                                    // waitKey(0);
                                }
                            }

                        }

                    }
                    graph.resize(0);
                    initialEstimate.clear();
//                currentEstimate = isam.calculateBestEstimate();
                    currentEstimate = fixedLagOptimizer.calculateEstimate();
                    VLOG(0) << "Exception caught : " << e.what() << std::endl;


                }

            }
        }
    }

    return true;
}

void Backend::removeVariables( gtsam::KeyVector tobeRemoved){
    if(optimizationMethod == 0){
        const NonlinearFactorGraph& nfg = isam.getFactorsUnsafe();
        set<size_t> removedFactorSlots;
        const VariableIndex variableIndex(nfg);
        for(Key key: tobeRemoved) {
            int lid = symbolIndex(key);
            const auto& slots = variableIndex[key];
            toBeRemovedFactorIndices.insert(toBeRemovedFactorIndices.end(), slots.begin(), slots.end());
//            lmFactorRecordMulti.erase(lid);
//            for(auto fr: frontEnd->map->getLandmark(lid)->KFs){
//                if(xFactorRecord.find(fr->frameId) != xFactorRecord.end()){
//                    vector<int> lms = xFactorRecord[fr->frameId];
//                    for(int i=0; i <lms.size(); i++) {
//                        if (lms[i] == lid)
//                        {
//                            xFactorRecord[fr->frameId].erase(xFactorRecord[fr->frameId].begin() + i);
//                            break;
//                        }
//                    }
//                }
//            }
//            frontEnd->map->deleteLandmark(lid);
        }

      }
    else if(optimizationMethod == 1){
        //Not implemented
    }
    else if(optimizationMethod == 2){
        //Not implemented
    }
}


void Backend::Smartfactor_deleteLandmark(int lid, GlobalMap* map){

    // 0. if it doesnt exist in the map? - shoiuldnt happend

    assert(map->getLandmark(lid) != NULL);

    // 1. delete form map
    map->deleteLandmark(lid);
    // 2. delete from graph and smart factor

//    // get the landmark pointer
//    auto landmark_ptr = smartFactor[lid];
//    const NonlinearFactorGraph& nfg = isam.getFactorsUnsafe();
//
//    auto variable_indices = isam.getVariableIndex();
//
//
//
//    for(int facInd =0 ; facInd < nfg.size(); facInd++){
//        if(nfg.at(facInd)){
//
////            nfg.at(facInd)->print("graph factor ");
////            graph.at(facInd)->
//
//            //RigFactor::shared_ptr fac = static_pointer_cast<RigFactor>(nfg.at(facInd));
//
//            if(nfg.at(facInd) == landmark_ptr){
////                VLOG(0)<<(landmark_ptr);
//
//                // check if it alreadt exists in the toBeRemovedFactorIndices
//                if(std::find(toBeRemovedFactorIndices.begin(), toBeRemovedFactorIndices.end(), facInd) != toBeRemovedFactorIndices.end()) {
//                    VLOG(0) << "Found an index that was already added" << facInd;
//                }
//
//                for(Key j: *nfg[facInd]){
//
//                    gtsam::Symbol sym = gtsam::Symbol(j);
//                    VLOG(0) <<" the key is " << sym.chr() << sym.index();
//                    const FactorIndices& factorEntries = variable_indices[j];
//
//
//
//                    auto entry = std::find(factorEntries.begin(),
//                                           factorEntries.end(), facInd);
//
//                    if (entry == factorEntries.end()) {
//                        VLOG(0) << " did not find the factor index in the variable index";
//                    }
//
//
//                }
//
//
//
//                toBeRemovedFactorIndices.push_back(facInd);
//                // delete from smartfactor
//               // smartFactor.erase(lid);
//                break;
//            }
//
//        }
//
//    }


    // 3. delete from smartpointer


}

void Backend::UpdateVariables_SmartFactors(){


    unique_lock<mutex> lock(frontEnd->mMutexPose);
    std::set<int> retriangulate_lids;
    if (frontEnd->useIMU)
    {
        frontEnd->lastState_ = gtsam::NavState(prevState_);
        frontEnd->lastBias_ = prevBias_;
    }
    if (backendType == MULTI) {
        int fr_ind = 0;
        int dummy_ind = 0;
        int comb_ind = 0;

        //Frames that need traingulation
        vector<MultiCameraFrame *> fms;
        // update poses
        for (auto &fr: frontEnd->lfFrames) {
            int poseid = fr->kfID;
            if (currentEstimate.exists(gtsam::Symbol('x', poseid))) {
                // Twb
                gtsam::Pose3 pose = currentEstimate.at<gtsam::Pose3>(gtsam::Symbol('x', poseid));
                gtsam::Matrix mat = pose.matrix();
                Mat curPos;
                cv::eigen2cv(mat, curPos);
                //Twc =  Twb * Tbc
                Mat Twc_tmp;
                if (fr->dummy_kf) {

                    // TODO: have a seperate vector for dummy poses. maybe its not needed
                    // as we might not need the
                    Twc_tmp = curPos * frontEnd->Tbc_cv;
                    frontEnd->allPoses_dummy[dummy_ind] = Twc_tmp.rowRange(0, 3).clone();
                } else {
                    Twc_tmp = curPos * frontEnd->Tbc_cv;
                    frontEnd->allPoses[fr_ind] = Twc_tmp.rowRange(0, 3).clone();
                }
                Mat diffPose = fr->pose.inv() * Twc_tmp;
                Mat rotvec;
                cv::Rodrigues(diffPose.rowRange(0, 3).colRange(0, 3), rotvec);

                if (cv::norm(diffPose.col(3).rowRange(0,3)) > 0.005 or cv::norm(rotvec) > 1e-06) {
                    VLOG(3) << "diff in pose of x" << poseid << " rot : " << rotvec << " trans: " << diffPose.col(3)
                            << endl;
                    fms.push_back(fr);
                }

                fr->pose = Twc_tmp.clone();

                frontEnd->combined_poses[comb_ind] = Twc_tmp.rowRange(0, 3).clone();
                frontEnd->combined_tStamps[comb_ind] = fr->timeStamp;
            } else {
                VLOG(0) << "Not updates pose : " << poseid;
            }

            if (fr->dummy_kf) {
                dummy_ind++;
            } else {
                fr_ind++;
            }
            comb_ind++;
        }

        // get all the landmarks seen in these frames
        for (auto kf: fms) {
            for (int i = 0; i < kf->lIds.size(); i++) {
                if (kf->lIds[i] != -1) {
                    retriangulate_lids.insert(kf->lIds[i]);
                }
            }

        }
    }


    int succ_lid_upd = 0;
    int fail_lid_upd = 0;

    auto start = std::chrono::high_resolution_clock::now();

    GlobalMap* map = frontEnd->map;
        for(auto lid: retriangulate_lids ){

            if (deleted_landmarks[lid] == 1){
//            VLOG(0) << " landmark " << deleted_landmarks[iter->first] << " has been deleted ";
                continue;
            }
//        VLOG(0) << " landmark " << deleted_landmarks[iter->first] << " has not been deleted ";
            // get the camera
            auto cameras = smartFactor[lid]->cameras(currentEstimate);

            // check if atleast 2 cameras have seen. it should  be atleast 2 but just in case check
            if(cameras.size() < 2){
                VLOG(0) << " not enough cameras have seen this landmark ";
                continue;
            }

            // get the landmark
            auto measurements = smartFactor[lid]->measured();
            TriangulationResult triangulate_check = smartFactor[lid]->triangulateSafe(cameras);
            if(triangulate_check.valid()){

                gtsam::Point3 landmark = gtsam::triangulatePoint3(cameras, measurements);

                cv::Mat landmark_point = (cv::Mat_<double>(3,1) << landmark.x(), landmark.y(), landmark.z());

                // check if the landmark actually exists or has been delted during the prev update
                // variable part

                assert(map->getLandmark(lid) != NULL);
                // update the landmark
                double diff_norm;
                bool success = map->updateLandmark(lid, landmark_point, diff_norm);

                if(success){
                    succ_lid_upd++;
                }
                else{

//                Smartfactor_deleteLandmark(lid, map);
//               deleted_landmarks[lid]=1;

                    fail_lid_upd++;
                }


            }
            else{


                if(triangulate_check.behindCamera()){
                    VLOG(2) << " landmark " << lid << " is behind camera";
                }
                if(triangulate_check.degenerate()){
                    VLOG(2) << " landmark " << lid << " is degenerate";
                }
                if(triangulate_check.outlier()){
                    VLOG(2) << " landmark " << lid << " is outlier";
                }
                if(triangulate_check.farPoint()){
                    VLOG(2) << " landmark " << lid << " is far point";
                }

                VLOG(0) << " triangulation failed for landmark " << lid;

                Smartfactor_deleteLandmark(lid, map);
                deleted_landmarks[lid]=1;

            }

        }


    // update landmarks
//    for(auto iter = smartFactor.begin(); iter!=smartFactor.end(); iter++ ){
//
//        if (deleted_landmarks[iter->first] == 1){
////            VLOG(0) << " landmark " << deleted_landmarks[iter->first] << " has been deleted ";
//            continue;
//        }
////        VLOG(0) << " landmark " << deleted_landmarks[iter->first] << " has not been deleted ";
//        // get the camera
//        auto cameras = smartFactor[iter->first]->cameras(currentEstimate);
//
//        // check if atleast 2 cameras have seen. it should  be atleast 2 but just in case check
//        if(cameras.size() < 2){
//            VLOG(0) << " not enough cameras have seen this landmark ";
//            continue;
//        }
//
//        // get the landmark
//        auto measurements = smartFactor[iter->first]->measured();
//        TriangulationResult triangulate_check = smartFactor[iter->first]->triangulateSafe(cameras);
//        if(triangulate_check.valid()){
//
//            gtsam::Point3 landmark = gtsam::triangulatePoint3(cameras, measurements);
//
//            cv::Mat landmark_point = (cv::Mat_<double>(3,1) << landmark.x(), landmark.y(), landmark.z());
//            int lid = iter->first;
//
//            // check if the landmark actually exists or has been delted during the prev update
//            // variable part
//
//            assert(map->getLandmark(lid) != NULL);
//            // update the landmark
//            double diff_norm;
//            bool success = map->updateLandmark(lid, landmark_point, diff_norm);
//
//            if(success){
//                succ_lid_upd++;
//            }
//            else{
//
////                Smartfactor_deleteLandmark(lid, map);
////               deleted_landmarks[lid]=1;
//
//                fail_lid_upd++;
//            }
//
//
//        }
//        else{
//
//
//            if(triangulate_check.behindCamera()){
//                VLOG(2) << " landmark " << iter->first << " is behind camera";
//            }
//            if(triangulate_check.degenerate()){
//                VLOG(2) << " landmark " << iter->first << " is degenerate";
//            }
//            if(triangulate_check.outlier()){
//                VLOG(2) << " landmark " << iter->first << " is outlier";
//            }
//            if(triangulate_check.farPoint()){
//                VLOG(2) << " landmark " << iter->first << " is far point";
//            }
//
//            VLOG(0) << " triangulation failed for landmark " << iter->first;
//            int lid = iter->first;
//
//            Smartfactor_deleteLandmark(lid, map);
//            deleted_landmarks[lid]=1;
//
//        }
//
//    }

//    for(auto& lid : deleted_landmarks){
//        smartFactor.erase(lid);
//    }
    auto stopT2 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(stopT2 - start);
    VLOG(0)<<"Total time taken for smart factors triangulation: "<<duration2.count()<<endl;
    VLOG(2) << "debug lid size " << debug_lids.size();
    VLOG(2) <<"succ_lid_upd " << succ_lid_upd << " fail_lid_upd " << fail_lid_upd;

}

bool Backend::GlobalRelocalizedPoseNode() {
    // add a node to factor graph for globally relocalized pose
    // if this is the first node in the graph then add the pose as a prior factor and also initial estimate with small
    // noise in pose, since we're certain about the pose.
    // if this is not the first node, then add the IMU factor between previous node and current node and add the globally
    // relocalized pose as a prior factor.
    cv::Mat currentPose = frontEnd->currentFrame->pose * frontEnd->Tbc_cv.inv();
    if (frontEnd->lfFrames.size() == 0)
        return false;
    if (frontEnd->currentFrame->kfID == 0) {
        // add a prior factor
        gtsam::Pose3 pose = convertPose3_CV2GTSAM(currentPose);
        gtsam::noiseModel::Diagonal::shared_ptr poseNoise = gtsam::noiseModel::Diagonal::Sigmas
                ((gtsam::Vector(6) << gtsam::Vector3::Constant(0.01), gtsam::Vector3::Constant(0.01)).finished());
        graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', frontEnd->currentFrame->kfID), pose, poseNoise));
        initialEstimate.insert(gtsam::Symbol('x', frontEnd->currentFrame->kfID), pose);
        VLOG(3) << "Inserted prior for the state x" << frontEnd->currentFrame->kfID << endl;
        return true;
    } else {
        // add IMU factor
        addIMUFactor(frontEnd->currentFrame->kfID - 1, frontEnd->currentFrame->kfID);
        // add prior factor
        gtsam::Pose3 pose = convertPose3_CV2GTSAM(currentPose);
        gtsam::noiseModel::Diagonal::shared_ptr poseNoise = gtsam::noiseModel::Diagonal::Sigmas
                ((gtsam::Vector(6) << gtsam::Vector3::Constant(0.01), gtsam::Vector3::Constant(0.01)).finished());
        graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', frontEnd->currentFrame->kfID), pose, poseNoise));
        initialEstimate.insert(gtsam::Symbol('x', frontEnd->currentFrame->kfID), pose);
        VLOG(3) << "Inserted prior for the state x" << frontEnd->currentFrame->kfID << endl;
        return true;
    }
}

void Backend::UpdateVariablesRelocalization() {
    // update the poses alone for the frames from the graph, no landmark update required
    unique_lock<mutex> lock(frontEnd->mMutexPose);
    if (frontEnd->useIMU)
    {
        frontEnd->lastState_ = gtsam::NavState(prevState_);
        frontEnd->lastBias_ = prevBias_;
    }
    if (backendType == MULTI){
        int fr_ind=0;
        int dummy_ind = 0;
        int comb_ind = 0;

        for (auto& fr : frontEnd->lfFrames) {
            int poseid = fr->kfID;
            if (currentEstimate.exists(gtsam::Symbol('x', poseid))) {
                // Twb
                gtsam::Pose3 pose = currentEstimate.at<gtsam::Pose3>(gtsam::Symbol('x', poseid));
                VLOG(3) << "AFTER OPTIMIZATION, pose of x" << poseid << " : " << pose << endl;
                gtsam::Matrix mat = pose.matrix();
                Mat curPos;
                cv::eigen2cv(mat, curPos);
                //Twc =  Twb * Tbc
                Mat Twc_tmp;
                if (fr->dummy_kf) {
//                    Twc_tmp = curPos * frontEnd->Tbc_cv;
//                    frontEnd->allPoses_dummy[dummy_ind] = Twc_tmp.rowRange(0, 3).clone();
                } else {
                    Twc_tmp = curPos;// * frontEnd->Tbc_cv;
                    frontEnd->allPoses[fr_ind] = Twc_tmp.rowRange(0, 3).clone();
//                    VLOG(0) << "pose of x" << poseid << " : " << Twc_tmp << endl;
                }
                Mat diffPose = fr->pose.inv() * Twc_tmp;
                VLOG(3) << "diff in pose of x" << poseid << " : " << diffPose << endl;
                fr->pose = Twc_tmp.clone();

                frontEnd->combined_poses[comb_ind] = Twc_tmp.rowRange(0, 3).clone();
                frontEnd->combined_tStamps[comb_ind] = fr->timeStamp;

            }
            if (fr->dummy_kf) {
                dummy_ind++;

            } else {
                fr_ind++;
            }
            comb_ind++;
        }
    }
}


void Backend::updateVariables() {
    GlobalMap* map = frontEnd->map;
    double mean_correction=0.0, max_correction=0.0;
    int num_lms = 0;
    gtsam::KeyVector tobeRemoved;
    for (gtsam::Values::iterator it = currentEstimate.begin(); it != currentEstimate.end(); ++it) {
        gtsam::Symbol lid = it->key;

        if (lid.chr() == 'l') {
            num_lms++;
            gtsam::Point3 b = currentEstimate.at<gtsam::Point3>(lid);
            double aa = b.x();
            double bb = b.y();
            double cc = b.z();
            double a[3][1] = {aa, bb, cc};
            cv::Mat point3(3, 1, CV_64F, a);
            double diff_norm;
            bool success = map->updateLandmark(lid.index(), point3, diff_norm);
            // if the update of the landmark is not successful
            //remove the landmark from the graph
            if(!success)
                tobeRemoved.push_back(lid);
            mean_correction += diff_norm;
            if(max_correction < diff_norm)
                max_correction = diff_norm;


        }

    }
    //Remove the following variables from the back-end
    // VLOG(3)<<"To Be Removed landmarks : "<<tobeRemoved.size()<<endl;
    removeVariables( tobeRemoved);
    mean_correction /= num_lms;
    // VLOG(3)<< "Mean correction for landmarks : "<<mean_correction<<", Max Correction : "<<max_correction<<endl;
    //cout<<"Number of LF Frames in front-end:"<<frontEnd->lfFrames.size()<<endl;
    unique_lock<mutex> lock(frontEnd->mMutexPose);
    if (frontEnd->useIMU)
    {
        frontEnd->lastState_ = gtsam::NavState(prevState_);
        frontEnd->lastBias_ = prevBias_;
    }
    if (backendType == MULTI){
        int fr_ind=0;
        int dummy_ind = 0;
        int comb_ind = 0;

        for (auto& fr : frontEnd->lfFrames){
            int poseid = fr->kfID;


            if(currentEstimate.exists(gtsam::Symbol('x', poseid))){
                // Twb
                gtsam::Pose3 pose = currentEstimate.at<gtsam::Pose3>(  gtsam::Symbol('x', poseid));
                gtsam::Matrix mat = pose.matrix();
                Mat curPos;
                cv::eigen2cv(mat, curPos);
                //Twc =  Twb * Tbc
                Mat Twc_tmp;
                if(fr->dummy_kf) {

//                    VLOG(0) << "dummy kf found";
//                    VLOG(0) << "frontEnd->allPoses_dummy size " << frontEnd->allPoses_dummy.size() << " dummy ind " << dummy_ind;
                    Twc_tmp = curPos * frontEnd->Tbc_cv;
                    frontEnd->allPoses_dummy[dummy_ind] = Twc_tmp.rowRange(0,3).clone();
//                    VLOG(0) <<" backend pose " << curPos;
//                    VLOG(0) << "frontEnd->allPoses_dummy at x" <<  poseid << frontEnd->allPoses_dummy[dummy_ind];


                }
                else {
                    Twc_tmp = curPos * frontEnd->Tbc_cv;
                    frontEnd->allPoses[fr_ind] = Twc_tmp.rowRange(0,3).clone();
                }
                Mat diffPose = fr->pose.inv() * Twc_tmp;
                VLOG(3)<<"diff in pose of x"<<poseid<<" : "<<diffPose<<endl;
//                VLOG(0) << "cur pose in backend" << curPos;

                fr->pose = Twc_tmp.clone();

                frontEnd->combined_poses[comb_ind] = Twc_tmp.rowRange(0,3).clone();
                frontEnd->combined_tStamps[comb_ind] = fr->timeStamp;


            }

            if(fr->dummy_kf) {
                dummy_ind++;

            }

            else{
                fr_ind++;
            }
            comb_ind++;
        }
    }


    /*
    deprecated - current version supports backendtype = MULTI
    else if(backendType == MULTI_RIGID ){
        for (auto& fr : frontEnd->lfFrames){
            int poseid = fr->frameId;
            for (int cid =0; cid< 1; cid++){
                int stateid = stoi(to_string(poseid) + to_string(cid));
                if(currentEstimate.exists(gtsam::Symbol('x', stateid))){
                    gtsam::Pose3 pose = currentEstimate.at<gtsam::Pose3>(  gtsam::Symbol('x', stateid));
                    gtsam::Matrix mat = pose.matrix();
                    Mat Pose_wc;
                    cv::eigen2cv(mat, Pose_wc);
                    Mat R = camArrayConfig->R_mats_[cid];/// store the pose of the cam chain
                    Mat t = camArrayConfig->t_mats_[cid];
                    Mat Pose_cb = Mat::eye(4,4, CV_64F);
                    Pose_cb.rowRange(0,3).colRange(0,3) = R;
                    Pose_cb.rowRange(0,3).colRange(3,4) = t;
                    Mat Pose_wb = Pose_wc* Pose_cb;
                    frontEnd->allPoses.push_back(Pose_wb.rowRange(0,3).clone());

                    Mat diffPose = fr->pose.inv() * Pose_wb;
                    VLOG(3)<<"diff in pose of x"<<stateid<<" : "<<diffPose<<endl;
                    fr->pose = Pose_wb.clone();
                    break; // break from this loop and go to next frameID
                }
            }
        }
    } 
    else if (backendType == MONO){
        int fr_ind=0;
        for (auto& fr : frontEnd->lfFrames){
            int poseid = fr->frameId;
            if(currentEstimate.exists(gtsam::Symbol('x', poseid))){

                gtsam::Pose3 pose = currentEstimate.at<gtsam::Pose3>(  gtsam::Symbol('x', poseid));
                gtsam::Matrix mat = pose.matrix();
                Mat curPos;
                cv::eigen2cv(mat, curPos);
                Mat diffPose = fr->pose.inv() * curPos;
                VLOG(3)<<"diff in pose of x"<<poseid<<" : "<<diffPose<<endl;
                //frontEnd->allPoses.push_back(curPos.rowRange(0,3).clone());
                frontEnd->allPoses[fr_ind] = curPos.rowRange(0,3).clone();
                fr->pose = curPos.clone();
            }
            fr_ind++;
        }
    } */

}




