//
// Created by Pushyami Kaveti on 6/6/20.
//

// This is a starter application to run LF -SLAM on the static portions of the scene using multi-view data


#include "MCSlam/FrontEnd.h"
#include "MCSlam/Backend.h"
#include "MCSlam/OpenGlViewer.h"

#include "ParseSettings.h"
#include "MCDataUtils/CamArrayConfig.h"
#include "MCDataUtils/MCDataUtilParams.h"

#include "LiveViewer.h"

#include "MCDataUtils/DatasetReader.h"
#include "MCDataUtils/RosbagParser.h"
#include "MCDataUtils/RosDataReader.h"

#include <chrono>
//// OPENGV ////
#include <Eigen/Eigen>
#include <opengv/types.hpp>
#include <memory>
#include <opengv/relative_pose/NoncentralRelativeAdapter.hpp>
#include <opengv/sac_problems/relative_pose/NoncentralRelativePoseSacProblem.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/relative_pose/methods.hpp>
#include "MCSlam/time_measurement.hpp"
///////
#include <ros/ros.h>
#include <opencv2/core/eigen.hpp>


void handleKeyboardInput(FrontEnd& frontend, Backend& backend, DatasetReaderBase &dataReader, OpenGlViewer &glViewer);

void process_frames(FrontEnd& frontend, Backend& backend);

bool updateData(FrontEnd& frontend,  DatasetReaderBase &dataReader);

DEFINE_bool(fhelp, false, "show config file options");

DEFINE_string(config_file, "", "config file path");

DEFINE_string(log_file, "pose_stats.txt", "log_file file path");
DEFINE_string(traj_file, "Tum_trajectory.txt", "trajectory file file path");

using namespace cv;
using namespace std;
using namespace opengv;
using namespace std::chrono;
float tracking_time=0.0;
float optim_time = 0.0;
float feat_xtract_time = 0.0;
int frame_counter=0;
int optim_cnt=0;

int main(int argc , char** argv){


    // parse arguments
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr=1;

    //parse the settings in config file
    MCDataUtilSettings settings;
    // Program to parse the setting from ParseSettings.cpp
    parse_settings(FLAGS_config_file, settings, FLAGS_fhelp);

    //Initialize get the data from ros bag using ros reader
    //read the dataset and calibration given settings for the refocus object
    DatasetReaderBase* datareader;

    if(settings.parse_bag){

        ros::init(argc, argv, "slam_node");
        ros::NodeHandle nh;

        datareader = new RosbagParser();

    }
    else{
        if (settings.is_ros){
            // if ROS is enabled Initialize the ROS node and
            ros::init(argc, argv, "slam_node"); // Initializing the ros node
            ros::NodeHandle nh;
            datareader = new RosDataReader(nh);
        }

        else{
            ros::init(argc, argv, "slam_node"); // Initializing the ros node
            ros::NodeHandle nh;
            datareader = new DatasetReader();
        }
    }
    datareader->initialize(settings);


   // create a camera configuration object with came intrinisc and extrinsic params
    auto* cam_cfg = new CamArrayConfig(datareader->getKMats(),datareader->getDistMats(),datareader->getRMats(),\
    datareader->getTMats(), datareader->getKalibrRMats(), datareader->getKalibrTMats(), datareader->img_size_, datareader->num_cams_, false);
    VLOG(0) << "datareader initialized" <<endl;
   // create the SLAM front end object
    FrontEnd *slam_frontend;  
//    slam_frontend = new FrontEnd(settings.frontend_params_file, *cam_cfg, settings.debug_mode, settings.calib_file_path,
//                                 settings.imu, settings.gps, settings.relocalization, settings.customVocabFile,
//                                 settings.dataBase, settings.graphLogs, settings.mapLogs, settings.navability);
    slam_frontend = new FrontEnd(settings.frontend_params_file, *cam_cfg, settings.debug_mode, settings.calib_file_path,
                                 settings.imu, settings.gps, settings.relocalization, settings.customVocabFile,
                                 settings.dataBase, settings.graphLogs, settings.mapLogs, settings.navability, settings.ImuMapFrame);
    VLOG(0) << "slam_frontend initialized" <<endl;
    // create the SLAM backend object
    Backend *slam_backend;
   slam_backend = new Backend(settings.backend_params_file, *cam_cfg, slam_frontend);
    VLOG(0) << "slam_backend initialized" <<endl;
   // create a openGlViewer object
   OpenGlViewer* glViewer;
   glViewer = new OpenGlViewer(settings.frontend_params_file, slam_frontend);
   if (!settings.relocalization)
        auto *viewerThread = new thread(&OpenGlViewer::goLive, glViewer);
   else
       auto *viewerThread = new thread([&] () {
           glViewer->goLiveFastTracking(slam_frontend);
       });

   //IN a loop datareader->getNext() or datareader->getNextGPU(), refocus.updateImgs() or refocus.updateImgsGPU()
   // refocus.updateZMap() , optional processing , perform refocus and show in GUI
   ros::AsyncSpinner spinner(0); // Use max cores possible for mt
   spinner.start();

   // This method runs the SLAM program in a loop for each new image.
   // user can give keyboard inputs to control the loop for obtaining next image,
   // visualizing and exiting
   handleKeyboardInput(*slam_frontend, *slam_backend,  *datareader, *glViewer);
   //    slam_frontend->writeLogs(FLAGS_log_file);
   slam_frontend->logFile_.close();
   slam_frontend->logFile2_.close();

   if(slam_frontend->useGPS){
       slam_frontend->writeTrajectoryToFile(FLAGS_traj_file, true);
   }
   else{
       slam_frontend->writeTrajectoryToFile(FLAGS_traj_file, false);
   }

//   slam_frontend->saveORBDatabase();
   slam_frontend->writeTrajectoryToFile(FLAGS_traj_file, false);
   // If not in relocalization mode. then save.
   if(!slam_frontend->relocal)
   {
       slam_frontend->saveORBDatabase();
   }

    return 0;
}

bool updateData(FrontEnd& frontend, DatasetReaderBase &dataReader) {

    vector<Mat> imgs, segMasks;
    std::deque<sensor_msgs::Imu> imu_msgs;
    std::deque<sensor_msgs::NavSatFix> gps_msgs;
    
    vector<string> segmaskNames;
    double timeStamp;


    if(dataReader.settings.gps){

        dataReader.getNext(imgs, timeStamp, imu_msgs, gps_msgs);

        frontend.gps_msgs_combined.insert(frontend.gps_msgs_combined.end(), gps_msgs.begin(), gps_msgs.end());

        VLOG(0) << "gps msgs size " << frontend.gps_msgs_combined.size() << endl;

    }

    if(dataReader.settings.imu){

        if(!dataReader.settings.gps){
            dataReader.getNext(imgs, timeStamp, imu_msgs);
        }
         VLOG(0) << " imu msgs size" << imu_msgs.size() << endl;
        if(!frontend.imu_initialized){
            frontend.imu_initialize(imu_msgs, timeStamp);
        }
        else{
            frontend.imu_msgs_combined.insert(frontend.imu_msgs_combined.end(), imu_msgs.begin(), imu_msgs.end());
        }
   }
    else{
        if(dataReader.settings.read_segmask){
            dataReader.getNext(imgs,segmaskNames, timeStamp);
        }
    else{
        dataReader.getNext(imgs, timeStamp);
        }
    }
   if (imgs.empty()){
       VLOG(0)<<"Reached the end of images";
       return false;
   }

    // uncomment this if we are using segmentation masks to delete features based on the semantics of the scene.
    /*if(dataReader.settings.use_segment){
        if(dataReader.settings.read_segmask) {
            segnet.updateSegmaskImgs(segmaskNames);
            segnet.updateImgs(imgs);
            segMasks = segnet.readSegmentationMasks();
            if (segMasks.size() < dataReader.num_cams_)
                return false;
        }
        else{
            //All image segmnetation
            segnet.updateImgs(imgs);
            segMasks = segnet.computeSegmentation_allimgs();
        }
    }
    else{ */

        for(int i =0; i <dataReader.num_cams_; i++)
            segMasks.push_back(Mat::zeros(dataReader.img_size_, CV_32FC1));
    // }

    std::vector<cv::Mat> copiedImgs;
    for (const auto& img : imgs) {
        copiedImgs.push_back(img.clone());
    }

    std::vector<cv::Mat> copiedSegMasks;
    for (const auto& mask : segMasks) {
        copiedSegMasks.push_back(mask.clone());
    }

    frontend.createFrame(copiedImgs, copiedSegMasks, timeStamp);

    return true;
}

void process_GPS_frames(FrontEnd& frontend, Backend& backend){
    /*
     * to check the validity of the gps messages and add the GPS keyframes.
     * (todo) ideally it would nice to implement a GPS class that can do this entire thing
     * would be easier to add additional stuff
     */

    while (!frontend.gps_msgs_combined.size() == 0) {

        sensor_msgs::NavSatFix gps_msg = frontend.gps_msgs_combined.front();
        // check: redundant - same as geodetic - enu frontend
        tuple<double, double, double> gps_message_enu = backend.processGPS_ENU(gps_msg);
        bool gps_valid = true;
        if (frontend.gps_data_.curr_gps_kf == -1) {
            // first gps message, so dont evaluate the distance, just update the variables
            frontend.gps_data_.curr_gps_kf = frontend.currentKFID;
            frontend.gps_data_.prev_gps_message_enu = gps_message_enu;
            frontend.gps_data_.curr_gps_message_enu = gps_message_enu;
            frontend.gps_msgs_combined.pop_front();
            continue;
        } else {

            // check: redundant: previous if has the same
            frontend.gps_data_.curr_gps_kf = frontend.currentKFID;
            frontend.gps_data_.curr_gps_message_enu = gps_message_enu;

            /*validity of the GPS message
             * this function removes bad GPS data
             * Current criterion - 2 gps messages should be 'x' meters apart
             * can add the covariance condition here
            */
            gps_valid = frontend.validGPSmessage(frontend.gps_data_);

            if (gps_valid) {

                // get the gps message time
                double gps_time = frontend.gps_msgs_combined.front().header.stamp.toSec();
                frontend.gps_data_.tStamp = gps_time;



                /* a. special case
                 * If we have a gps message coming immediately after a keyframe resulting in
                 * zero or 1 imu message between them, then the gps message has to be added to
                 * the previous vision keyframe directly because it essentially means the same
                 * state.
                 */
                double prev_kf_time = frontend.lfFrames.back()->timeStamp;
                // check: its not called dummy kf
                if(gps_time - prev_kf_time < 0.01 && frontend.lfFrames.back()->dummy_kf == false && frontend.initialized_ == INITIALIZED){
                    VLOG(0) << " gps message very close to previous vision kf ";
                    // append gps data
                    frontend.appendGPSValue(frontend.gps_data_, frontend.gps_data_.curr_gps_kf-1);
                    backend.addKeyframeGPS(false);
                }

                else{
                    // perform imu preintegration upto this time
                    frontend.imu_preintegration(gps_time);

                    // add a dummy keyframe in the frontend
                    frontend.insertKeyFrame(backend.prevState_, backend.prevBias_, gps_time);

                    // create a dummy frame in the backend
                    backend.addKeyframeGPS(true);

                    frontend.appendGPSValue(frontend.gps_data_, frontend.gps_data_.curr_gps_kf);

                }

                VLOG(0) << " Added GPS kf to the backend";
                optim_cnt++;

                /* optimize only if the transformation variable is connected
                 * to atleast 3 poses in the backend.
                 */
                bool update = backend.optimizePosesLandmarks();
                if (update) {
                    backend.UpdateVariables_SmartFactors();
                }
                // update last gps message data
                frontend.gps_data_.prev_gps_message_enu = gps_message_enu;
            }
        }
    }
}

void process_frames(FrontEnd& frontend, Backend& backend){

    frame_counter++;
    /// If we are performing VIO, we wait to track frame until the IMU is
    /// initialized. i.e estimate its initial orientation WRT the world
    /// assuming it is starting from rest.
    auto startF = high_resolution_clock::now();
    if(frontend.useIMU){
        if(!frontend.imu_initialized || frontend.imu_msgs_combined.empty()){

            return;
        }
    }


    /* ------TRACKING--------
     * When run in tracking mode this if block is executed and returned,
     * control doesn't proceed further in this function */
    if(frontend.relocal and !frontend.useIMU)
    {
        frontend.processFrame();
        frontend.initialized_ = INITIALIZED;
        if (!frontend.trackingStatus) {
            double image_time = frontend.currentFrame->timeStamp;
            ros::Time time_param2(image_time);
            frontend.imu_preintegration(image_time);
            if(frontend.checkGlobalRelocalization()){
                frontend.trackingStatus = true;
                // add a backend keyframe using the pose of the first keyframe from global relocalization
                backend.prevBias_ = frontend.bias_prior;
                // convert lfframes.back()->pose to gtsam pose
                gtsam::Pose3 pose = convertPose3_CV2GTSAM(frontend.lfFrames.back()->pose);
                backend.prevPose_ = pose;
                backend.prevVel_ = gtsam::Vector3(0,0,0);
                backend.prevState_ = gtsam::NavState(pose, backend.prevVel_);
                backend.prev_Kf = frontend.lfFrames.back()->kfID;
                // add factor in backend for first keyframe
//                backend.GlobalRelocalizedPoseNode();
                backend.poseCounter++;
                // reset IMU preintegration object
//                frontend.imu_integrator_comb->resetIntegrationAndSetBias(backend.prevBias_);
            }
            else{
                frontend.reset();
//                frontend.deleteCurrentFrame();
                return;
            }
        }
        else
        {
            VLOG(0) << "Tracking module started";
            // number of trees in the kd tree vector
            VLOG(0) << "number of trees in the kd tree vector " << frontend.currentFrame->image_kps_kdtree.size() << endl;
            // new frame is created in updateData function and added to frontend
            // preintegrate IMU data upto this time
//            double image_time = frontend.currentFrame->timeStamp;
//            ros::Time time_param2(image_time);
//            frontend.imu_preintegration(image_time);
            // predict the pose of the car upto this time using IMU relative to the previous keyframe
            VLOG(0) << "previous pose before imu prediction " << backend.prevPose_  << endl;
//            VLOG(0) << "previous pose before imu prediction " << backend.prevPose_ * convertPose3_CV2GTSAM(frontend.Tbc_cv).inverse() << endl;
//            gtsam::NavState next_state = frontend.imu_integrator_comb->predict(backend.prevState_, backend.prevBias_);
            gtsam::NavState next_state = backend.prevState_;
            VLOG(0) << "next state's pose predicted " << next_state.pose() << endl;
            // make this pose absolute with respect to the world frame by chaining it with the pose from the previous keyframe
            // find landmark matches for this pose using the tracking module until we loose tracking
            // ignore for now
            bool tracked = frontend.startTrackingModule(next_state);
            // if we loose tracking, reset the frontend and return
            if(!tracked){
                VLOG(0) << "lost tracking" << endl;
                frontend.trackingStatus = false;
                frontend.reset();
//                frontend.deleteCurrentFrame();
                return;
            }
                // if we have tracking, add a new keyframe to the backend
            else{
                VLOG(0) << "tracking successful" << endl;
                // refine the pose using GP3P
                frontend.refinePose();
            }
            // create new next state variable with refined pose
            gtsam::Pose3 refinedPose = convertPose3_CV2GTSAM(frontend.currentFrame->pose);
            VLOG(0) << "velocity in next state - " << next_state.v() << endl;
            gtsam::NavState refinedNextState(refinedPose, next_state.v());

            if (frontend.lfFrames.size() > 1 ) {
                // add node to factor graph and do backend stuff
                backend.RelocalizationGraphConstructor(refinedNextState);
                // optimize the poses in the graph
                bool update = backend.optimizePosesLandmarks();
                if (update) {
//                backend.UpdateVariables_SmartFactors();
                    backend.UpdateVariablesRelocalization();
                }
                VLOG(0) << "Backend optimization done" << endl;
                VLOG(0) << "Latest pose after optimization " << backend.prevPose_ << endl;
                // print the poses updated
//                for (auto &lf : frontend.lfFrames) {
//                    VLOG(0) << "pose of keyframe updated" << lf->kfID << " " << lf->pose << endl;
//                }
            }
        }

        frontend.reset();
        frontend.currentFrame->image_kps_kdtree.clear();
        /// Important to avoid memory leaks ---
//        frontend.deleteCurrentFrame();
        return;
    }

    if(frontend.relocal and frontend.useIMU)
    {
        frontend.processFrame();
        frontend.initialized_ = INITIALIZED;
        if (!frontend.trackingStatus) {
            double image_time = frontend.currentFrame->timeStamp;
            ros::Time time_param2(image_time);
            frontend.imu_preintegration(image_time);
            if(frontend.checkGlobalRelocalization()){
                frontend.trackingStatus = true;
                // add a backend keyframe using the pose of the first keyframe from global relocalization
                backend.prevBias_ = frontend.bias_prior;
                // convert lfframes.back()->pose to gtsam pose
                gtsam::Pose3 pose = convertPose3_CV2GTSAM(frontend.lfFrames.back()->pose);
                backend.prevPose_ = pose;
                backend.prevVel_ = gtsam::Vector3(0,0,0);
                backend.prevState_ = gtsam::NavState(pose, backend.prevVel_);
                backend.prev_Kf = frontend.lfFrames.back()->kfID;
                backend.poseCounter++;
                // reset IMU preintegration object
                frontend.imu_integrator_comb->resetIntegrationAndSetBias(backend.prevBias_);
            }
            else{
                frontend.reset();
                return;
            }
        }
        else
        {
            VLOG(0) << "Tracking module started";
            // number of trees in the kd tree vector
            VLOG(0) << "number of trees in the kd tree vector " << frontend.currentFrame->image_kps_kdtree.size() << endl;
            // new frame is created in updateData function and added to frontend
            // preintegrate IMU data upto this time
            double image_time = frontend.currentFrame->timeStamp;
            ros::Time time_param2(image_time);
            frontend.imu_preintegration(image_time);
            // predict the pose of the car upto this time using IMU relative to the previous keyframe
            VLOG(0) << "previous pose before imu prediction " << backend.prevPose_ << endl;
            gtsam::NavState next_state = frontend.imu_integrator_comb->predict(backend.prevState_, backend.prevBias_);
//            gtsam::NavState next_state = backend.prevState_;
            VLOG(0) << "next state's pose predicted " << next_state.pose() << endl;
            // make this pose absolute with respect to the world frame by chaining it with the pose from the previous keyframe
            // find landmark matches for this pose using the tracking module until we loose tracking
            // ignore for now
            bool tracked = frontend.startTrackingModule(next_state);
            // if we loose tracking, reset the frontend and return
            if(!tracked){
                VLOG(0) << "lost tracking" << endl;
                frontend.trackingStatus = false;
                frontend.reset();
//                frontend.deleteCurrentFrame();
                return;
            }
            // if we have tracking, add a new keyframe to the backend
            else{
                VLOG(0) << "tracking successful" << endl;
            }
            // create new next state variable with refined pose
            gtsam::Pose3 refinedPose = convertPose3_CV2GTSAM(frontend.currentFrame->pose);
            VLOG(0) << "velocity in next state - " << next_state.v() << endl;
            gtsam::NavState refinedNextState(refinedPose, next_state.v());

            if (frontend.lfFrames.size() > 1 ) {
                // add node to factor graph and do backend stuff
                backend.RelocalizationGraphConstructor(refinedNextState);
                // optimize the poses in the graph
                bool update = backend.optimizePosesLandmarks();
                if (update) {
                    backend.UpdateVariablesRelocalization();
                }
                VLOG(0) << "Backend optimization done" << endl;
                VLOG(0) << "Latest pose after optimization " << backend.prevPose_ << endl;
            }
        }

        frontend.reset();
        frontend.currentFrame->image_kps_kdtree.clear();
        // TODO: To avoid flooding memory delete previous keyframe information but hold only pose and kfID
        return;
    }



    /* ------SLAM-------- */
    // we have a custom struct to store the gps message and a kf before and a kf after it so that we can interpolate later for the initial alignment.
    if(!frontend.gps_umeyama_data_vec.empty() && frontend.gps_vins_next){

        cv::Mat vins_pose = frontend.lfFrames.back()->pose;
        gtsam::Pose3 pose = convertPose3_CV2GTSAM(vins_pose);
        // prev vins frame is differnt from teh recent lf frame.
        if(!(pose.equals(convertPose3_CV2GTSAM(frontend.gps_umeyama_data_vec.back().prev_keyframe->pose)))){
            frontend.gps_umeyama_data_vec.back().next_keyframe = frontend.lfFrames.back();
            frontend.gps_vins_next = false;
        }
    }

    // GPS: We perform the GPS kabsch initialization procedure here.
    if(frontend.useGPS && !frontend.gps_initialized) {
        if (frontend.initialized_ == INITIALIZED) {
            // most recent lf frame
            if (frontend.gps_msgs_combined.size() != 0) {
                frontend.gps_initialize_kabsch(frontend.gps_msgs_combined, frontend.lfFrames.back());
            }
        }
        frontend.gps_msgs_combined.clear();
    }
    // everytime we have a gps message, create a GPS keyframe and add it to the backend
    if (frontend.gps_initialized && !frontend.gps_msgs_combined.empty()) {

        //to counter the isam update error that occurs because the Graph is chopped off at a point:
        // check: do we still need this?
        if (!backend.currentEstimate.exists(gtsam::Symbol('b', backend.prev_Kf)) &&
            !backend.initialEstimate.exists(gtsam::Symbol('b', backend.prev_Kf))) {

            VLOG(0) << "isam error and this value doesnt exist in the current estimate";
        } else {
            // check: ideally it would be nice to have a GPS class that can do this entire thing
            process_GPS_frames(frontend, backend);
        }
    }

    // process theMC snapshot. extract features, obtain intra matches and triangulate to get the 3D points
    auto startT = high_resolution_clock::now();

    frontend.processFrame();

    auto stopT = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stopT - startT);
    feat_xtract_time += duration.count();

    VLOG(1)<<"Total time taken for Computing the intramatch features: "<<duration.count()<<" | Average: " <<(feat_xtract_time/frame_counter)<<endl;

    //track thecurrent frame WRT the last keyframe
    startT = high_resolution_clock::now();

    bool new_kf = frontend.trackFrame();

    stopT = high_resolution_clock::now();
    auto stopF = high_resolution_clock::now();
    duration = duration_cast<milliseconds>(stopT - startT);
    tracking_time += duration.count();
    VLOG(1)<<"Total time taken for tracking MC Frame: "<<duration.count()<<" | Average: " << (tracking_time/frame_counter)<< endl;
    VLOG(1)<<"Total time taken for Frontend: "<<duration_cast<milliseconds>(stopF - startF).count()<<" | Average: " << (duration_cast<milliseconds>(stopF - startF).count()/frame_counter)<< endl;


    /* ------BACK_END-------- */
    startT = high_resolution_clock::now();
    bool optimtize_ = false;
    if(frontend.initialized_ == NOT_INITIALIZED){
        frontend.reset();
        return;
    }

    else if(frontend.initialized_ == INITIALIZED and new_kf){

        if(frontend.useIMU){

            //if we get a new keyframe, perform preintegration between 2 keyframes
            if(frontend.first_keyframe) {

                double image_time = frontend.lfFrames[frontend.lfFrames.size() - 2]->timeStamp;

                if(frontend.lfFrames.size() == 2){
                    frontend.imu_values_removal(image_time);
                }
               else{
                    ///Reinitialized
                    ///pre-integrate (already done in the frontend initialization part) and
                    /// add a back-end IMU factor upto the first initialized state.

                    backend.addIMUFactor(frontend.lfFrames[frontend.lfFrames.size() - 3]->kfID, frontend.lfFrames[frontend.lfFrames.size() - 2]->kfID);
                    backend.prev_Kf = frontend.lfFrames[frontend.lfFrames.size() - 2]->kfID;
                    gtsam::NavState next_state = frontend.imu_integrator_comb->predict(backend.prevState_, backend.prevBias_);
                    backend.prevPose_ = next_state.pose();
                    backend.prevBias_ = backend.prevBias_;
                    backend.prevVel_ = next_state.v();
                    backend.prevState_ = next_state;

                    // reset imu preintegration object
                    frontend.imu_integrator_comb->resetIntegrationAndSetBias(backend.prevBias_);
                }
                frontend.first_keyframe = false;
            }
            double image_time = frontend.lfFrames.back()->timeStamp;
            ros::Time time_param2(image_time);
            frontend.imu_preintegration(image_time);

            // gtsam::imuBias::ConstantBias prevBias_ = frontend.bias_prior;
            // VLOG(0) << "prev bias" << prevBias_ << endl;

        }

        if(backend.backendType == MULTI){
            /*
             * Special scenario: If there are not imu messages between the prev gps kf and the current
             * vision kf. at this time make this gps kf a vision kf
             */
            if(frontend.useGPS){
                backend.change_vision_kf_info();
            }
//            optimtize_ = backend.addKeyFrameMulti();
            auto startT1 = high_resolution_clock::now();
            bool loop_closed = false;
            if (frontend.result.detection()){
                loop_closed = true;
            }


            optimtize_ = backend.SmartFactor_backend(loop_closed);
//            optimtize_ = backend.addKeyFrameMulti();
            auto stopT1 = high_resolution_clock::now();
            auto duration1 = duration_cast<milliseconds>(stopT1 - startT1);
            VLOG(0)<<"Total time taken for Back-end optimization: "<<duration1.count()<<endl;
//            optimtize_ = false;
//            if(optimtize_) {
//                frontend.appendLogs(false, backend.prevState_, backend.prevBias_);
//            }

            if(optimtize_){
                optim_cnt++;
                auto startT2 = high_resolution_clock::now();
                bool update = backend.optimizePosesLandmarks();
                auto stopT2 = high_resolution_clock::now();
                auto duration2 = duration_cast<milliseconds>(stopT2 - startT2);
                VLOG(0)<<"Total time taken for Back-end optimization: "<<duration2.count()<<endl;
                if(update) {
//                    backend.updateVariables();
                    auto  startT3 = high_resolution_clock::now();
                    backend.UpdateVariables_SmartFactors();
                    auto stopT3 = high_resolution_clock::now();
                    auto duration3 = duration_cast<milliseconds>(stopT3 - startT3);
                    VLOG(0)<<"Total time taken for updating the variables: "<<duration3.count()<<endl;
                }
            }

            /// If LoopClosure candidate discovered.
            if (frontend.result.detection())
            {
                /// Convert to frameID.
                backend.lvar.loop_candidate_= true;
                backend.lvar.rel_pose_ = frontend.result.relative_pose;
                backend.lvar.best_match_idx_ = (int)frontend.lfFrames[frontend.result.match]->frameId;
                backend.lvar.curr_frame_idx_ = (int)frontend.currentFrame->frameId;
                frontend.appendLogs(true, backend.prevState_, backend.prevBias_);
            }
            else
            {
                frontend.appendLogs(false, backend.prevState_, backend.prevBias_);
            }

        }
        /*
        these modes are deprecated.
        current code supports MULTI mode
        if(backend.backendType == MULTI_RIGID){
            optimtize_ = backend.addKeyFrameMulti();
        }
        else if(backend.backendType == MONO ){
            optimtize_ = backend.addKeyFrame();
        }
        */
        else{
            return;
        }


    }

   stopT = high_resolution_clock::now();
   duration = duration_cast<milliseconds>(stopT - startT);
   optim_time+=duration.count();
   if(optimtize_){
        VLOG(1)<<"Total time taken for Back-end optimization: "<<duration.count()<<" | Average: "<<(optim_time/optim_cnt)<<endl;
   }


   frontend.reset();

}

void handleKeyboardInput(FrontEnd& frontend, Backend& backend, DatasetReaderBase &dataReader, OpenGlViewer &glViewer) {

    int seq = 1;
    //create live-viewer object
    // LiveViewer gui;

    // initialize
    bool initialized = false;
    float time_per_frame = 0.0;

    //need this - vlog values?
    while (true) {
        if (dataReader.settings.is_ros && ! ros::ok())
            return;
        int key = waitKey(1);
        if ((key & 255) != 255)
        {
            int condition = ((key & 255));
            switch (condition) {
                case 46:{

                    bool success = updateData(frontend, dataReader);
                    if(success){
                        process_frames(frontend, backend);

                    } else
                        continue;

                    break;
                }
                case 27: // finishing
                {
                    destroyAllWindows();
                    glViewer.requestFinish();
                    while(!glViewer.isFinished()){
                        usleep(5000);
                    }


                    return ;
                }

                default: // code to be executed if n doesn't match any cases
                    VLOG(0) << "INVALID INPUT" << endl;
            }
        }
        bool live = true; //Hard coded. - need this?
        if(live){
            auto startT = high_resolution_clock::now();
            //update the new Camera data to corresponding objects
            bool success = updateData(frontend, dataReader);

            if(success){
                process_frames(frontend, backend);
            } else{

                //continue;
                destroyAllWindows();
                glViewer.requestFinish();
                while(!glViewer.isFinished()){
                    usleep(5000);
                }
                return ;
            }

            auto stopT = high_resolution_clock::now();
            auto duration = duration_cast<milliseconds>(stopT - startT);
            time_per_frame += duration.count();
            VLOG(1)<<"Time taken per frame : "<<duration.count()<<" | Average: "<<  (time_per_frame/seq)<<endl;
            auto avg_fps = (1/((time_per_frame/seq)/1000));
//            VLOG(0)<<" Average fps: "<<  avg_fps <<endl;
        }
        // Visualize
        seq++;

   }
}

