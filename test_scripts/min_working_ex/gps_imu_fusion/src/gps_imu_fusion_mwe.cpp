/**
 * A Minimum working example code to understand the performance of the new custom GPS
 * factor by performing GPS + IMU fusion.
 *
 * This was built to not compute bias estimation and use simulated data that didnt require these.
 *
 * one issue seen - transformation is not converging to the exact solution expected - we are seeing a differnece of a few degrees
 */


#include <stdio.h>
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
#include <gflags/gflags.h>
#include "newGPSFactor.h"

using namespace std;
using namespace cv;
using namespace gtsam;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::T; // transform between gps and imu frame

struct ImuMeasurement {
    double time;
    Vector3 accelerometer;
    Vector3 gyroscope;
};

struct GpsMeasurement {
    double time;
    Point3 position;
};

string data_file = " ";

std::deque<ImuMeasurement> prev_imu_msgs;


void read_parameter_data(string filename){

    YAML::Node config = YAML::LoadFile(filename);

    data_file = config["imu_gps_datafile"].as<string>();


}

void optimize_data(){

    if(data_file == " "){
        VLOG(0) <<" imu gps datafile not loaded";
        return;
    }

    std::ifstream factor_file(data_file.c_str());
    std::string line;
    VLOG(0) << "Reading the data file " << data_file;
    bool init = false;
    int x_id;


    // initial setup
    gtsam::Pose3 prevPose_;
    gtsam::Vector3  prevVel_(1, 0 ,0);
    gtsam::imuBias::ConstantBias prev_bias; // assume zero initial bias
    gtsam::NavState prevState_(prevPose_, prevVel_);

    // Assume prior noise model and add it the graph.
    noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8).finished()); // rad,rad,rad,m, m, m
    noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3,1e-8); // m/s
    noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6,1e-8);

    // graph related
    NonlinearFactorGraph *graph = new NonlinearFactorGraph();
    Values result;
    Values initial_values;

    // setup isam2 params and create isam2 solver object
    ISAM2 isam;


    // imu
    PreintegratedCombinedMeasurements *imu_preintegrated_;
    boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = PreintegratedCombinedMeasurements::Params::MakeSharedU(
            9.8);

    // We use the sensor specs to build the noise model for the IMU factor.
    double accel_noise_sigma = 1e-3;
    double gyro_noise_sigma =  1e-3;
    double accel_bias_rw_sigma =  1e-3;
    double gyro_bias_rw_sigma =  1e-3;
    Matrix33 measured_acc_cov = Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
    Matrix33 measured_omega_cov = Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
    Matrix33 integration_error_cov = Matrix33::Identity(3,3)*1e-8; // error committed in integrating position from velocities
    Matrix33 bias_acc_cov = Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma,2);
    Matrix33 bias_omega_cov = Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma,2);
    Matrix66 bias_acc_omega_int = Matrix::Identity(6,6)*1e-3; // error in the bias used for preintegration

    p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
    p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
    p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
    p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
    p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
    p->biasAccOmegaInt = bias_acc_omega_int;

    imu_preintegrated_ = new PreintegratedCombinedMeasurements(p, prev_bias);

    // addl variables
    double prev_timestamp = 0;
    double last_imu_time = -1;
    int prev_xid = 0;
    gtsam::Pose3 Tbg;
//    double angle = 0.785101;
//    double angle1 = M_PI_4;
    Pose3 E_T_V(Rot3::Rz(M_PI_4/6), Point3(0.1, 0.1, 0.1));

    while(factor_file >> line){

        if(line == "imu_raw"){
            ImuMeasurement imu;

            factor_file >> imu.time;
            for (int i = 0; i < 3 ; i++){
                factor_file >> imu.accelerometer(i);
            }
            for( int i = 0; i < 3; i++){
                factor_file >> imu.gyroscope(i);
            }

            if(imu.time < prev_timestamp){
                VLOG(0) << " old imu message, ignoring";
                continue;
            }
            prev_imu_msgs.push_back(imu);
        }

        if(line == "g"){

            GpsMeasurement gps;
            factor_file >> x_id;
            factor_file >> gps.time;
            for(int i =0 ; i< 3; i++){
                factor_file >> gps.position(i);
            }
            VLOG(1) << "=================";
            VLOG(1) <<"g" << x_id << ": "  << gps.position(0) <<", " << gps.position(1) << ", " << gps.position(2);

            if(init == false){
                graph->add(PriorFactor<Pose3>(X(x_id), prevPose_, pose_noise_model));
                graph->add(PriorFactor<Vector3>(V(x_id), prevVel_,velocity_noise_model));
                graph->add(PriorFactor<imuBias::ConstantBias>(B(x_id), prev_bias,bias_noise_model));

                VLOG(1) << " adding prior to x" << x_id <<" = " << endl << prevPose_ ;
                VLOG(1) << " adding prior to v" << x_id <<" = " << endl << prevVel_ ;
                VLOG(1) << " adding prior to b" << x_id <<" = " << endl << prev_bias ;

                initial_values.insert(X(x_id), prevPose_);
                initial_values.insert(V(x_id), prevVel_);
                initial_values.insert(B(x_id), prev_bias);
                initial_values.insert(T(0), E_T_V);

                VLOG(1) << " adding initial estimate to x" << x_id <<" = " << endl << prevPose_ ;
                VLOG(1) << " adding initial estimate to v" << x_id <<" = " << endl << prevVel_ ;
                VLOG(1) << " adding initial estimate to b" << x_id <<" = " << endl << prev_bias ;
                VLOG(1) << " adding initial estimate to t0 = " << endl << E_T_V ;

                prev_xid = x_id;
                init = true;

                continue;

            }

            while(!(prev_imu_msgs.empty())){

                ImuMeasurement imu_msg = prev_imu_msgs.front();
//                VLOG(0) << "imu time " << std::setprecision(15) << imu_msg.time;

                if(imu_msg.time <= gps.time){
                    double dt = (last_imu_time < 0) ? (0.1) : (imu_msg.time - last_imu_time);
                    imu_preintegrated_->integrateMeasurement(imu_msg.accelerometer, imu_msg.gyroscope, dt);
//                    VLOG(1) << "dt " << dt;
//                    VLOG(1) << "last_imu time " << std::setprecision(15) << last_imu_time;
//                    VLOG(1) << " imu  mesage acc " << imu_msg.accelerometer(0) << ", " << imu_msg.accelerometer(1) << ", " << imu_msg.accelerometer(2);
//                    VLOG(1) << " imu  mesage gyro " << imu_msg.gyroscope(0) << ", " << imu_msg.gyroscope(1) << ", " << imu_msg.gyroscope(2);
                    last_imu_time = imu_msg.time;
                    prev_imu_msgs.pop_front();

                }
                else{
                    last_imu_time = gps.time;
                    break;
                }
            }
//            VLOG(0) <<"gps time " << std::setprecision(15) << gps.time;

            //add imu factor
            PreintegratedCombinedMeasurements *preint_imu_combined = dynamic_cast<PreintegratedCombinedMeasurements *>(imu_preintegrated_);
            CombinedImuFactor imu_factor(X(prev_xid), V(prev_xid),
                                         X(x_id), V(x_id),
                                         B(prev_xid), B(x_id),
                                         *preint_imu_combined);
            graph->add(imu_factor);

            VLOG(0) <<"Added imu factor between x" << prev_xid <<" and x"<<x_id;

            // add gps factor

            noiseModel::Diagonal::shared_ptr correction_noise = noiseModel::Diagonal::Sigmas(Vector3(1, 1, 1));
            newGPSFactor gps_factor = newGPSFactor(X(x_id), gps.position, T(0), correction_noise
                    ,Tbg);
//            GPSFactor gps_factor(X(x_id), gps.position, correction_noise);
            graph->add(gps_factor);
            VLOG(0) <<"Added gps factor between x" << x_id << " and t0";

            // add initial values
            gtsam::NavState propState = imu_preintegrated_->predict(prevState_, prev_bias);
            initial_values.insert(X(x_id), propState.pose());
            initial_values.insert(V(x_id), propState.v());
            initial_values.insert(B(x_id), prev_bias);

            VLOG(0) <<"previous state" << endl << prevState_;
            VLOG(0) <<"prop state" << endl << propState;

            // optimize the graph
            if(x_id < 2){
                VLOG(0) <<" not performing isam update";
                prev_xid = x_id;
                continue;
            }
            VLOG(0) <<"isam update";
            isam.update(*graph, initial_values);
            result = isam.calculateEstimate();
            initial_values.clear();


//            gtsam::LevenbergMarquardtParams Lparams;
//            Lparams.setVerbosityLM("SUMMARY");
//            gtsam::LevenbergMarquardtOptimizer optimizer(*graph, initial_values, Lparams);
//            Values result = optimizer.optimize();
            graph->resize(0);


            // update the variables for the next update
            prevPose_ = result.at<Pose3>(X(x_id));
            prevVel_ = result.at<Vector3 >(V(x_id));
            prev_bias = result.at<imuBias::ConstantBias>(B(x_id));
            prevState_ = gtsam::NavState(prevPose_, prevVel_);
            E_T_V = result.at<Pose3>(T(0));
            VLOG(0) <<" E_T_V after update " << E_T_V;
            VLOG(0) << "angle measured is " << E_T_V.rotation().yaw();
            VLOG(0) <<"optimized state " << endl << prevState_;
            // reset the preintegration
            imu_preintegrated_->resetIntegrationAndSetBias(prev_bias);

            // update time variable
            prev_timestamp = gps.time;
            prev_xid = x_id;

//            if(x_id > 5){
//                return;
//            }
        }

    }

}

int main(int argc , char** argv){

    string filename = "/home/auv/tri_ws/src/TRI-SLAM/test_scripts/min_working_ex/gps_imu_fusion/config/parameters.yaml";
    google::ParseCommandLineFlags(&argc, &argv, true);

    read_parameter_data(filename);
    optimize_data();
    VLOG(0) <<"done ";



}








