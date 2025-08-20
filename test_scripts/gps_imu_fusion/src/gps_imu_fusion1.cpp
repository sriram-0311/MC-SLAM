/*

Code to perform GPS + IMU fusion with the zed data
ZED's IMU and BU353 GPS messages are converted into a KITTI based text file.

*/


#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Eigen>
#include<opencv4/opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <yaml-cpp/yaml.h>
#include <glog/logging.h>
#include "newGPSFactor.h"
#include <boost/json.hpp>


using namespace std;
using namespace cv;
using namespace gtsam;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::T; // transform between gps and imu frame

struct ImuMeasurement {
    double time;
    double dt;
    Vector3 accelerometer;
    Vector3 gyroscope;
};

struct GpsMeasurement {
    double time;
    Vector3 position;
};

void loadIMUKittidata( vector<ImuMeasurement>& imu_measurements,  vector<ImuMeasurement>& imu_measurements_rest, vector<ImuMeasurement>& imu_measurements_move, string imu_data_file, double rest_end_time){

    printf("-- Reading IMU measurements from file\n");
    string line;
    {
        ifstream imu_data(imu_data_file.c_str());
        getline(imu_data, line, '\n');  // ignore the first line

        double time = 0, dt = 0, acc_x = 0, acc_y = 0, acc_z = 0, gyro_x = 0,
                gyro_y = 0, gyro_z = 0;
        while (!imu_data.eof()) {

            getline(imu_data, line, '\n');
            sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &time, &dt,
                   &acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z);

            ImuMeasurement measurement;
            measurement.time = time;
            measurement.dt = dt;

            if(measurement.dt < 1){
                measurement.accelerometer = Vector3(acc_x, acc_y, acc_z);
                measurement.gyroscope = Vector3(gyro_x, gyro_y, gyro_z);
                imu_measurements.push_back(measurement);

                if(measurement.time < rest_end_time){
                    imu_measurements_rest.push_back(measurement);
                }
                else{
                    imu_measurements_move.push_back(measurement);
                }
            }
        }
    }

    printf("IMU measurements rest: %d\n", (int)imu_measurements_rest.size());
    printf("IMU measurements move: %d\n", (int)imu_measurements_move.size());

}


void loadGPSKittidata(string gps_data_file, vector<GpsMeasurement>& gps_measurements){

    string line;
    ifstream gps_data(gps_data_file.c_str());
    double time = 0, gps_x = 0, gps_y = 0, gps_z = 0;
    while (!gps_data.eof()) {
        getline(gps_data, line, '\n');
        sscanf(line.c_str(), "%lf %lf %lf %lf", &time, &gps_x, &gps_y, &gps_z);

        GpsMeasurement measurement;
        measurement.time = time;
        measurement.position = Vector3(gps_x, gps_y, gps_z);

        gps_measurements.push_back(measurement);
    }

}

void computeIMUbias(vector<ImuMeasurement>& imu_measurements, Vector3& accelerometer_bias, Vector3& gyroscope_bias, double g){

    accelerometer_bias = Vector3::Zero();
    gyroscope_bias = Vector3::Zero();

    for(int i = 0; i < imu_measurements.size(); i++){

        accelerometer_bias += imu_measurements[i].accelerometer;
        gyroscope_bias += imu_measurements[i].gyroscope;

    }

    accelerometer_bias /= imu_measurements.size();
    gyroscope_bias /= imu_measurements.size();

    accelerometer_bias(2) = accelerometer_bias(2) - g;


    printf("accelerometer bias: %lf %lf %lf\n", accelerometer_bias.x(), accelerometer_bias.y(), accelerometer_bias.z());

    printf("gyroscope bias: %lf %lf %lf\n", gyroscope_bias.x(), gyroscope_bias.y(),
           gyroscope_bias.z());

}

gtsam::Pose3 convert_cv2_to_gtsam_pose3(cv::Mat pose){


    gtsam::Rot3 R(pose.at<double>(0,0), pose.at<double>(0,1), pose.at<double>(0,2),
                  pose.at<double>(1,0), pose.at<double>(1,1), pose.at<double>(1,2),
                  pose.at<double>(2,0), pose.at<double>(2,1), pose.at<double>(2,2));

    gtsam::Point3 t(pose.at<double>(0,3), pose.at<double>(1,3), pose.at<double>(2,3));

    return gtsam::Pose3(R, t);

}

void writeOptimized(gtsam::Values& result,  std::map<int, double>& tStamps, bool multi, const std::string& filename)
{
    std::ofstream fil(filename);
    gtsam::Values pose_values = result.filter<gtsam::Pose3>();
    for(gtsam::Values::iterator it = pose_values.begin(); it != pose_values.end() ; ++it)
    {
        gtsam::Symbol sym = it->key;
        int x_id = (int)sym.index();
        auto pose = pose_values.at<gtsam::Pose3>(sym);
        //Matrix mat = pose.matrix();
        const gtsam::Rot3& rotation_matrix = pose.rotation();
        gtsam::Quaternion quaternion = rotation_matrix.toQuaternion();
        gtsam::Point3 trans_vect = pose.translation();
        int stamp_key=0;
        if (multi)
        {
            std::string s = std::to_string(x_id);
            s.pop_back();
            if(!s.empty())
                stamp_key=stoi(s);
        }
        else
        {
            stamp_key = x_id;
        }
        double stamp = tStamps[stamp_key];
        fil << std::setprecision(5)<< std::fixed << stamp <<" "<< trans_vect.x() << " " <<
            trans_vect.y() << " " << trans_vect.z() << " " << quaternion.x() << " " << quaternion.y() << " "
            << quaternion.z() << " " << quaternion.w() << "\n";

    }
    fil.close();
}

int main() {

    // read from yaml file
    YAML::Node config = YAML::LoadFile("/home/auv/tri_ws/src/TRI-SLAM/test_scripts/gps_imu_fusion/config/fusion_params.yaml");

    // read from config file
    string imu_data_file = config["imu_data_file"].as<string>();
    string gps_data_file = config["gps_data_file"].as<string>();

    // debug files - read from config file
    string imu_log = config["imu_log_file"].as<string>();
    string fusion_log = config["fusion_log_file"].as<string>();
    string E_T_V_log = config["E_T_V_file"].as<string>();

    ofstream imu_log_file;
    ofstream fusion_log_file;
    ofstream E_T_V_log_file;

    imu_log_file.open(imu_log.c_str());
    fusion_log_file.open(fusion_log.c_str());
    E_T_V_log_file.open(E_T_V_log.c_str());

    double g = config["g"].as<double>();
    double accelerometer_sigma = config["accelerometer_sigma"].as<double>();
    double gyroscope_sigma = config["gyroscope_sigma"].as<double>();
    double integration_sigma = 0.000000;
    double accelerometer_bias_sigma = config["accelerometer_bias_sigma"].as<double>();
    double gyroscope_bias_sigma = config["gyroscope_bias_sigma"].as<double>();

    vector<ImuMeasurement> imu_measurements;
    vector<ImuMeasurement> imu_measurements_rest;
    vector<ImuMeasurement> imu_measurements_move;
    vector<GpsMeasurement> gps_measurements;

    double time_rest_end = config["imu_rest_time_max"].as<double>();

    int optim_type = config["optim_type"].as<int>();

    YAML::Node Tbg = config["Tbg"];
    cv::Mat TbgMat(4, 4, CV_64F);

    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            TbgMat.at<double>(i,j) = Tbg[i][j].as<double>();
        }
    }

    gtsam::Pose3 Tbg_gtsam = convert_cv2_to_gtsam_pose3(TbgMat);

    /*step 1: load imu and gps data into custom structs */
    loadIMUKittidata(imu_measurements, imu_measurements_rest, imu_measurements_move, imu_data_file, time_rest_end);
    loadGPSKittidata(gps_data_file, gps_measurements);

    printf("GPS measurements: %d\n", (int)gps_measurements.size());
    printf("IMU measurements: %d\n", (int)imu_measurements.size());


    /* step 2: bias calculation */
    Vector3 accelerometer_bias = Vector3::Zero();
    Vector3 gyroscope_bias = Vector3::Zero();

    computeIMUbias(imu_measurements_rest, accelerometer_bias, gyroscope_bias, g);
    gyroscope_bias = Vector3::Zero();
    //Params based on ZED IMU
    gtsam::PreintegratedCombinedMeasurements *imu_integrator_comb;
    auto w_coriolis = Vector3::Zero();
    auto current_bias = imuBias::ConstantBias(accelerometer_bias, gyroscope_bias);
    auto imu_params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(9.8);
    imu_params->accelerometerCovariance = I_3x3 * pow(accelerometer_sigma, 2);
    imu_params->integrationCovariance = I_3x3 * pow(integration_sigma, 2);
    imu_params->gyroscopeCovariance = I_3x3 * pow(gyroscope_sigma, 2);
    imu_params->biasAccCovariance = I_3x3 * pow(accelerometer_bias_sigma, 2);
    imu_params->biasOmegaCovariance = I_3x3 * pow(gyroscope_bias_sigma, 2);

    imu_integrator_comb = new gtsam::PreintegratedCombinedMeasurements(imu_params, current_bias);

    auto current_pose_world = Pose3();
    Vector3 current_velocity_world = Vector3::Zero();

    auto pose_noise_model = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5)
                    .finished());  // rad,rad,rad,m, m, m
    auto velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);  // m/s
    auto bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);
    auto transform_noise_model = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 0.1, 0.1, 0.1, 0.5, 0.5, 0.5)
                    .finished());


    // step 2a: Set ISAM2 parameters and create ISAM2 solver object
    ISAM2Params isam_params;
    isam_params.relinearizeSkip = 2;
    ISAM2 isam(isam_params);


    /* step 3: Create the factor graph and values object that will store new factors and
     values to add to the graph */

    Values initial_values;
    NonlinearFactorGraph* graph = new NonlinearFactorGraph();
    NavState prev_state;
    NavState prop_state = prev_state;
    imuBias::ConstantBias prev_bias = current_bias;
    double t_prev = gps_measurements[0].time;
    Values result;
    std::map<int, double> timestamps;

    Pose3 E_T_V(Rot3::Rz(M_PI_4), Point3(0.01, 0.01, 0.01));
    VLOG(0) << E_T_V << " E_T_V " << endl;

    for (size_t i = 0; i < gps_measurements.size() - 1; i++) {

        int j = 0;

        auto current_pose_key = X(i);
        auto current_vel_key = V(i);
        auto current_bias_key = B(i);
        auto current_transform_key = T(0);
        double t = gps_measurements[i].time;

        if(i == 0){

            current_pose_world = Pose3(Rot3(), Point3(0, 0, 0));

            // add initial values
            initial_values.insert(current_pose_key, current_pose_world);
            initial_values.insert(current_vel_key, current_velocity_world);
            initial_values.insert(current_bias_key, current_bias);
            initial_values.insert(current_transform_key, E_T_V);

            graph->addPrior(X(i), current_pose_world, pose_noise_model);
            graph->addPrior(V(i), current_velocity_world, velocity_noise_model);
            graph->addPrior(B(i), current_bias, bias_noise_model);

            //add a weak prior
            graph->addPrior(T(i), E_T_V, noiseModel::Diagonal::Sigmas(
                    (Vector(6) << 5, 5, 5, 2, 2, 2)
                            .finished()));

            // gps measurement in gtsam Point3 format
            Point3 gps_position_gtsam(gps_measurements[i].position(0),gps_measurements[i].position(1), gps_measurements[i].position(2));

            auto gps_factor = newGPSFactor(X(i), gps_position_gtsam, T(i), noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.5)), Tbg_gtsam);
            graph->push_back(gps_factor);
        }

        else{

            // preintegrate imu measurements
            while(j < imu_measurements.size()){

                if(imu_measurements[j].time < t_prev){
                    // pop the imu measurements
                    imu_measurements.erase(imu_measurements.begin() + j);
                    continue;
                }

                if(imu_measurements[j].time > t){
                    break;
                }

                else{
                    double dt = imu_measurements[j].dt;
                    imu_integrator_comb->integrateMeasurement(imu_measurements[j].accelerometer, imu_measurements[j].gyroscope, dt);
                    t_prev = imu_measurements[j].time;
                    j++;
                }

            }

            // create imu factor
            auto imu_factor = gtsam::CombinedImuFactor(X(i - 1), V(i - 1), X(i), V(i), B(i - 1), B(i), *imu_integrator_comb);
            graph->push_back(imu_factor);

            // gps measurement in gtsam Point3 format
            Point3 gps_position_gtsam(gps_measurements[i].position(0),gps_measurements[i].position(1), gps_measurements[i].position(2));

            // add gps factor
            auto gps_factor = newGPSFactor(X(i), gps_position_gtsam, T(0), noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.5)), Tbg_gtsam);
            graph->push_back(gps_factor);


            prop_state = imu_integrator_comb->predict(prev_state, prev_bias);
            initial_values.insert(X(i), prop_state.pose());
            initial_values.insert(V(i), prop_state.v());
            initial_values.insert(B(i), prev_bias);

            VLOG(0) << " E_T_V * pose " << endl << E_T_V * prop_state.pose() << endl;
            VLOG(0) << " gps message " << endl << gps_position_gtsam << endl;

            // store this in the imu log file
    //            imu_log_file << t << "," << prop_state.pose().x() << "," << prop_state.pose().y() << "," << prop_state.pose().z() << endl;

            {
    //                 if(i > 5){
    //                     break;
    //                 }
                graph->saveGraph("/home/auv/graph.dot");
                if(optim_type == 2){
                    isam.update(*graph, initial_values);
                    isam.update();
                    result = isam.calculateEstimate();
                    graph->resize(0);
                    initial_values.clear();
                }
                if(optim_type == 1){
                    LevenbergMarquardtOptimizer optimizer(*graph, initial_values);
                    result = optimizer.optimize();
                }

                VLOG(0) << "optimized graph " << endl;

                // update prev state and bias
                prev_state = NavState(result.at<Pose3>(X(i)), result.at<Vector3>(V(i)));
                prev_bias = result.at<imuBias::ConstantBias>(B(i));
                Pose3 prev_E_T_V = E_T_V;
                imu_integrator_comb->resetIntegrationAndSetBias(prev_bias
                );
                t_prev = t;
                timestamps[i] = gps_measurements[i].time;

                VLOG(0) << "Added " << i << " GPS measurements and " << j << " IMU measurements." << std::endl;
                VLOG(0) << "current pose: " << result.at<Pose3>(X(i)) << std::endl;
                VLOG(0) << "predicted pose: " << prop_state.pose().translation().transpose() << std::endl;
                VLOG(0) << "current velocity: " << result.at<Vector3>(V(i)).transpose() << std::endl;
                VLOG(0) << "current bias: " << result.at<imuBias::ConstantBias>(B(i)) << std::endl;

                E_T_V = result.at<Pose3>(T(0));
                VLOG(0) << " Transformation E_T_V " << endl << E_T_V << endl;


            }
        }
    }
    /* step: store the optimized results */
    writeOptimized(result, timestamps, false, fusion_log);

    E_T_V_log_file << std::setprecision(10) <<
                   E_T_V.rotation().matrix()(0,0) << ", " << E_T_V.rotation().matrix()(0,1) << ", " << E_T_V.rotation().matrix()(0,2) << ", " << E_T_V.translation().x() << ", " << endl <<
                   E_T_V.rotation().matrix()(1,0) << ", " << E_T_V.rotation().matrix()(1,1) << ", " << E_T_V.rotation().matrix()(1,2) << ", " << E_T_V.translation().y() << ", " << endl <<
                   E_T_V.rotation().matrix()(2,0) << ", " << E_T_V.rotation().matrix()(2,1) << ", " << E_T_V.rotation().matrix()(2,2) << ", " << E_T_V.translation().z() << ", " << endl <<
                   0 << ", " << 0 << ", " << 0 << ", " << 1 << endl;

    E_T_V_log_file.close();


    return 0;
}

//std::ifstream json_pose_file(
//        "/home/aryaman/catkin_ws/navability_maps/THUNDERHILL_rosbag_2023_17_08_14_25_40_e83/THUNDERHILL_rosbag_2023_17_08_14_25_40_e83_poses.json");
//boost::json::object navabilityJsonPoseParser = boost::json::parse(json_pose_file).as_object();
//
//std::string camera_pose = "cD1l3010";
//for (auto &j: navabilityJsonPoseParser) {
//if (j.key() == camera_pose) {
//VLOG(0) << "Camera pose found in the json file - " << camera_pose << std::endl;
//// get the timestamp
//boost::json::string time_stamp = j.value().at("timestamp").as_string();
//// convert date time format to posix time format
//std::string time_str = time_stamp.c_str();
//VLOG(0) << "Time stamp - " << time_str << std::endl;
//// Parse the input time string
//std::tm timeStruct = {};
//std::istringstream timeStream(time_str);
//timeStream >> std::get_time(&timeStruct, "%Y-%m-%dT%H:%M:%S");
//
//// Adjust the timezone offset
//timeStruct.tm_hour -= timeStruct.tm_gmtoff / 3600;
//timeStruct.tm_min -= (timeStruct.tm_gmtoff % 3600) / 60;
//
//// Convert the time struct to a time_t value
//time_t epochTime = std::mktime(&timeStruct);
//std::string timestamp = std::to_string(epochTime);
//VLOG(0) << "Timestamp - " << timestamp << std::endl;
//}
//
//}