//
// Created by auv on 5/29/24.
//
/*

Code to perform DVL + IMU fusion with the zed data

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
#include "new_DVL_factor.h"
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

struct DvlMeasurement {
    double time;
    Vector3 velocity;
    double velErr;
    double heading;
};

void world_imu_frame(vector<ImuMeasurement>& imu_msgs, double g, Eigen::Matrix3d& rot_bw, Eigen::Vector3d& bias_acc, Eigen::Vector3d& bias_gyro){


    Eigen::MatrixXd acceleration_matrix(imu_msgs.size(), 3);

    for(int i=0; i<imu_msgs.size(); i++){
        acceleration_matrix(i,0) = imu_msgs[i].accelerometer(0);
        acceleration_matrix(i,1) = imu_msgs[i].accelerometer(1);
        acceleration_matrix(i,2) = imu_msgs[i].accelerometer(2);
    }


    Eigen::MatrixXd g_matrix(imu_msgs.size(), 3);
    g_matrix.setZero();
    g_matrix.col(2) = Eigen::VectorXd::Constant(imu_msgs.size(), g);


    // compute mean of the accelerometer measurements
    Eigen::VectorXd EA = acceleration_matrix.colwise().mean();
    // compute the pitch
    double roll = atan(EA(1) / EA(2)) ;
    //compute roll
    double pitch = asin(-1 * EA(0)/g);
    // create the rotation matrix
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
    I(0, 0) = cos(pitch);
    I(0, 1) = 0;
    I(0, 2) = - sin(pitch);
    I(1, 0) = sin(roll) * sin(pitch);
    I(1, 1) = cos(roll);
    I(1, 2) = cos(pitch)*sin(roll);
    I(2, 0) = sin(pitch)*cos(roll);
    I(2, 1) = -sin(roll);
    I(2, 2) = cos(roll)*cos(pitch);

    cout<<"roll: "<<roll* 180/ M_PI<<" pitch: "<<pitch* 180/ M_PI<<endl;
    cout<<I<<endl;
    // return it
    Eigen::MatrixXd bias_meas = acceleration_matrix - (I * g_matrix.transpose()).transpose();
    Eigen::VectorXd bias = bias_meas.colwise().mean();
    cout<<"estimated bias acc: "<<bias<<endl;
    // compute bias
    rot_bw = I;
    bias_acc = bias;
    bias_gyro = Eigen::Vector3d();
    for(int i = 0; i < imu_msgs.size(); i++){

        bias_gyro += imu_msgs[i].gyroscope;

    }

    bias_gyro /= imu_msgs.size();
}



void loadIMUdata(vector<ImuMeasurement>& imu_measurements,  vector<ImuMeasurement>& imu_measurements_rest,\
                 vector<ImuMeasurement>& imu_measurements_move, string imu_data_file, double rest_end_time, bool exact){


    // use the kitti metadata first to get the calibration parameters even for zed data

    string line;
    printf("-- Reading IMU measurements from file\n");
    ImuMeasurement stationarymeasurement;
    {
        ifstream imu_data(imu_data_file.c_str());
        getline(imu_data, line, '\n');  // ignore the first line
        getline(imu_data, line, '\n');  // ignore the first two lines

        double time = 0, dt = 0, acc_x = 0, acc_y = 0, acc_z = 0, gyro_x = 0,
                gyro_y = 0, gyro_z = 0;
        double dum=0;
        while (!imu_data.eof()) {


            getline(imu_data, line, '\n');

            sscanf(line.c_str(), "%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", &dum, &time, &dt,
                       &acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z);


            ImuMeasurement measurement;
            measurement.time = time;

            measurement.dt = dt;

            if(measurement.dt < 1){

                measurement.accelerometer = Vector3(acc_x, acc_y, acc_z);
                measurement.gyroscope = Vector3(gyro_x* M_PI/180.0, gyro_y* M_PI/180.0, gyro_z* M_PI/180.0);
                imu_measurements.push_back(measurement);

                if( measurement.time < rest_end_time){
                    if (exact){
                        continue;
                    }
                   else{
                        imu_measurements_rest.push_back(measurement);
                   }
                }
                else if(measurement.time == rest_end_time){
                    if (exact){
                        cout<<measurement.time<<"=="<<rest_end_time<<endl;
                        stationarymeasurement = measurement;
                        imu_measurements_move.push_back(measurement);

                    }
                }
                else{
                    imu_measurements_move.push_back(measurement);
                }
            }
        }
    }
    imu_measurements_rest.push_back(stationarymeasurement);
    printf("IMU measurements rest: %d\n", (int)imu_measurements_rest.size());
    printf("IMU measurements move: %d\n", (int)imu_measurements_move.size());

}


void loadDVLdata(string dvl_data_file, vector<DvlMeasurement>& dvl_measurements, double rest_end_time){

    string line;
    ifstream dvl_data(dvl_data_file.c_str());
    getline(dvl_data, line, '\n');
    double time = 0, vel_x = 0, vel_y = 0, vel_z = 0;
    double dum=0, dt=0, vel_err=0, depth=0, heading=0;
    while (!dvl_data.eof()) {
        getline(dvl_data, line, '\n');
        //for dvl interface messages
        sscanf(line.c_str(), "%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", &dum, &time, &dt, &vel_x, &vel_y, &vel_z, &vel_err, &depth, &heading );

        DvlMeasurement measurement;
        measurement.time = time;
        if( measurement.time >= rest_end_time){
            measurement.velocity = Vector3(vel_x, vel_y,vel_z);
            measurement.velErr = vel_err;
            if (vel_x > 3.0 or vel_y > 3.0)
                continue;
            dvl_measurements.push_back(measurement);
        }

    }

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
    YAML::Node config = YAML::LoadFile("/home/auv/tri_ws/src/TRI-SLAM/test_scripts/gps_imu_fusion/config/stim_dvl_params.yaml");

    // read from config file
    string imu_data_file = config["imu_data_file"].as<string>();
    string dvl_data_file = config["dvl_data_file"].as<string>();

    string fusion_log = config["fusion_log_file"].as<string>();

    ofstream imu_log_file;
    ofstream fusion_log_file;
    ofstream E_T_V_log_file;

    fusion_log_file.open(fusion_log.c_str());


    double g = config["g"].as<double>();


    vector<ImuMeasurement> imu_measurements_dump;
    vector<ImuMeasurement> imu_measurements_rest;
    vector<ImuMeasurement> imu_measurements;
    vector<DvlMeasurement> dvl_measurements;

    double time_rest_end = config["imu_rest_time_max"].as<double>();

    int optim_type = config["optim_type"].as<int>();

    YAML::Node Tbd = config["Tbd"];
    cv::Mat TbdMat(4, 4, CV_64F);

    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            TbdMat.at<double>(i,j) = Tbd[i][j].as<double>();
        }
    }

    gtsam::Pose3 Tbg_gtsam = convert_cv2_to_gtsam_pose3(TbdMat);

    /*step 1: load imu and gps data into custom structs */
    loadIMUdata(imu_measurements_dump, imu_measurements_rest, imu_measurements, imu_data_file, time_rest_end, true);
    loadDVLdata(dvl_data_file, dvl_measurements, time_rest_end);

    printf("DVL measurements: %d\n", (int)dvl_measurements.size());
    printf("IMU measurements: %d\n", (int)imu_measurements.size());

    Vector3 gravity = Vector3(0, 0, g);
    Eigen::Matrix3d world_imu_matrix(3, 3);
    /* step 2: bias calculation */
    Vector3 accelerometer_bias = Vector3::Zero();
    Vector3 gyroscope_bias = Vector3::Zero();

    //estimate IMU orientation
    world_imu_frame(imu_measurements_rest, g ,world_imu_matrix, accelerometer_bias, gyroscope_bias );
    printf("accelerometer bias: %lf %lf %lf\n", accelerometer_bias.x(),
           accelerometer_bias.y(), accelerometer_bias.z());

    printf("gyroscope bias: %lf %lf %lf\n", gyroscope_bias.x(), gyroscope_bias.y(),
           gyroscope_bias.z());

    accelerometer_bias(2) = 0.0;
    gyroscope_bias = Vector3::Zero();
    //Params based on ZED IMU
    gtsam::PreintegratedCombinedMeasurements *imu_integrator_comb;
    auto w_coriolis = Vector3::Zero();

    double accelerometer_sigma = 0.07 / 60.0;  //0.01;            //config["accelerometer_sigma"].as<double>();
    double gyroscope_sigma = 0.15 / 60.0 * M_PI / 180.0;  //0.01;// //config["gyroscope_sigma"].as<double>();
    double integration_sigma = 0.000000;
    double accelerometer_bias_sigma = 0.000167 ; //0.00001;  //2.0/3600*(0.04 * 0.04); //config["accelerometer_bias_sigma"].as<double>();
    double gyroscope_bias_sigma = 0.001947; //2 / 3600 * 0.3 * 0.3; //config["gyroscope_bias_sigma"].as<double>();

    auto imu_params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(9.81);
    imu_params->accelerometerCovariance =  I_3x3 * pow(accelerometer_sigma, 2);  // acc white noise in continuous
    imu_params->integrationCovariance = I_3x3 * pow(integration_sigma, 2);  // integration uncertainty continuous
    imu_params->gyroscopeCovariance =  I_3x3 * pow(gyroscope_sigma, 2);  // gyro white noise in continuous
    imu_params->omegaCoriolis = w_coriolis;
    imu_params->biasAccCovariance = I_3x3 * accelerometer_bias_sigma;
    imu_params->biasOmegaCovariance = I_3x3 * gyroscope_bias_sigma;
    auto current_bias = imuBias::ConstantBias(accelerometer_bias, gyroscope_bias);
    imu_integrator_comb = new gtsam::PreintegratedCombinedMeasurements(imu_params, current_bias);
    SharedNoiseModel meas_noise = noiseModel::Isotropic::Sigma(3, 0.05);

    // rotaiton is basically identity
    auto current_pose_world = Pose3(Rot3(world_imu_matrix.transpose()), Vector3::Zero());
    // the vehicle is stationary at the beginning at position 0,0,0
    Vector3 current_velocity_world = Vector3::Zero();

    auto pose_noise_model = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1)
                    .finished());  // rad,rad,rad,m, m, m
    auto velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);  // m/s
    auto bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-6); //1e-3


    // step 2a: Set ISAM2 parameters and create ISAM2 solver object
    ISAM2Params isam_params;
    //isam_params.optimizationParams = ISAM2DoglegParams(0.5, 1e-5, DoglegOptimizerImpl::ONE_STEP_PER_ITERATION);
    isam_params.relinearizeSkip = 3;
    isam_params.relinearizeThreshold = 0.1;
    isam_params.factorization = gtsam::ISAM2Params::QR;
//    isam_params.enableDetailedResults = true;  // Increase verbosity
//    isam_params.evaluateNonlinearError = true;
    ISAM2 isam(isam_params);



    /* step 3: Create the factor graph and values object that will store new factors and
     values to add to the graph */
    Values initial_values;
    NonlinearFactorGraph* graph = new NonlinearFactorGraph();


    Values result;
    std::map<int, double> timestamps;

    gtsam::NavState prev_state = NavState(current_pose_world, current_velocity_world);
    NavState prop_state = prev_state;
    imuBias::ConstantBias prev_bias = current_bias;
    int imu_ind = 0;
    for (size_t i = 0; i < dvl_measurements.size() - 1; i++) {

        auto current_pose_key = X(i);
        auto current_vel_key = V(i);
        auto current_bias_key = B(i);
        double dvl_t = dvl_measurements[i].time;

        if(i == 0){

            // add initial values
            initial_values.insert(current_pose_key, current_pose_world);
            initial_values.insert(current_vel_key, current_velocity_world);
            initial_values.insert(current_bias_key, current_bias);


            graph->addPrior(X(i), current_pose_world, pose_noise_model);
            graph->addPrior(V(i), current_velocity_world, velocity_noise_model);
            graph->addPrior(B(i), current_bias, bias_noise_model);

            // dvl factor
            auto dvl_factor = DVLFactor(current_pose_key, current_vel_key, dvl_measurements[i].velocity,  \
                                imu_measurements[imu_ind].gyroscope,  meas_noise, Tbg_gtsam);
            graph->push_back(dvl_factor);


            //imu_ind++;
            timestamps[i] = dvl_measurements[i].time;
        }

        else{

            // preintegrate imu measurements
            while(imu_ind < imu_measurements.size()){

                if(imu_measurements[imu_ind].time <= dvl_t){
                    //VLOG(0)<<std::setprecision(5)<< std::fixed<<imu_measurements[imu_ind].time <<"<="<< dvl_t<<endl;
                    double dt = imu_measurements[imu_ind].dt;
                    imu_integrator_comb->integrateMeasurement(imu_measurements[imu_ind].accelerometer, imu_measurements[imu_ind].gyroscope, dt);
                    imu_ind++;
                }
                else if(imu_measurements[imu_ind].time > dvl_t){
                      break;
                }

            }

            // create imu factor
            auto imu_factor = gtsam::CombinedImuFactor(X(i - 1), V(i - 1), X(i), V(i), B(i - 1), B(i), *imu_integrator_comb);

            graph->push_back(imu_factor);

            auto dvl_factor = DVLFactor(current_pose_key, current_vel_key, dvl_measurements[i].velocity,
                                            imu_measurements[imu_ind-1].gyroscope,  meas_noise, Tbg_gtsam);
            graph->push_back(dvl_factor);


            prop_state = imu_integrator_comb->predict(prev_state, prev_bias);
            initial_values.insert(X(i), prop_state.pose());
            initial_values.insert(V(i), prop_state.v());
            initial_values.insert(B(i), prev_bias);

            graph->saveGraph("/home/auv/graph.dot");
            if(optim_type == 2 and i%3==0 ){
                try{
                    ISAM2Result res = isam.update(*graph, initial_values);
                    isam.update();
                    result = isam.calculateEstimate();
                    graph->resize(0);
                    initial_values.clear();
                    // update prev state and bias
                    prev_state = NavState(result.at<Pose3>(X(i)), result.at<Vector3>(V(i)));
                    prev_bias = result.at<imuBias::ConstantBias>(B(i));
                    VLOG(0) << "current pose: " << result.at<Pose3>(X(i)) << std::endl;
                    VLOG(0) << "current velocity: " << result.at<Vector3>(V(i)).transpose() << std::endl;
                    VLOG(0) << "current bias: " << result.at<imuBias::ConstantBias>(B(i)) << std::endl;
                }
                catch(Exception e){
                    VLOG(0)<<"Eception in ISAM. break";
                    break;
                }

            }
            if(optim_type == 1){
                //graph->print();
                LevenbergMarquardtOptimizer optimizer(*graph, initial_values);
                result = optimizer.optimize();
            }

            VLOG(0) << "optimized graph " << endl;
            imu_integrator_comb->resetIntegrationAndSetBias(prev_bias);
            timestamps[i] = dvl_measurements[i].time;

            VLOG(0) <<std::setprecision(5)<< std::fixed << "Added " << i << " DVL measurements and " << imu_ind << " IMU measurements." <<timestamps[i]<< std::endl;
            VLOG(0)<<"vel x,y,z: "<<dvl_measurements[i].velocity<<endl;
//                VLOG(0) << "current pose: " << result.at<Pose3>(X(i)) << std::endl;
            VLOG(0) << "predicted pose: " << prop_state.pose().translation().transpose() << std::endl;
//                VLOG(0) << "current velocity: " << result.at<Vector3>(V(i)).transpose() << std::endl;
//                VLOG(0) << "current bias: " << result.at<imuBias::ConstantBias>(B(i)) << std::endl;
        }
        if(i ==900) break;
    }
    /* step: store the optimized results */
    writeOptimized(result, timestamps, false, fusion_log);

    return 0;
}

