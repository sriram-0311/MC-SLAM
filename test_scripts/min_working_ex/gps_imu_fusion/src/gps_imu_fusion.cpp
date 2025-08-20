/**
 * A Minimum working example code to understand the performance of the new custom GPS
 * factor by performing GPS + IMU fusion.
 * 
 * This involves using the actual data [graph logs] collected when doing SLAM online. 
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

struct Umeyama {
    Vector3 gps_enu;
    Vector3 imu_translation;
};
std::deque<Umeyama> kabsch_data;
std::deque<GpsMeasurement> gps_data;

string data_file = " ";
string calib_file = " ";
string result_file = " ";
string gps_data_gt_file = " ";
double g, accelerometer_sigma, gyroscope_sigma, accelerometer_bias_sigma, gyroscope_bias_sigma, integration_sigma;
gtsam::Matrix Tbg(4,4);
std::deque<ImuMeasurement> prev_imu_msgs;
std::map<int, double> tStamps;

/**
 *
 * Method that reads the calibration parameters
 */
void read_gps_imu_data(string path){

    VLOG(0) << "Reading calibration data" << path << std::endl;

    YAML::Node doc = YAML::LoadFile(path);


    g = doc["imu"]["g_norm"].as<double>();
    accelerometer_sigma = doc["imu"]["acc_noise"].as<double>();
    gyroscope_sigma = doc["imu"]["gyr_noise"].as<double>();
    integration_sigma = 0.000000;
    accelerometer_bias_sigma = doc["imu"]["acc_walk"].as<double>();
    gyroscope_bias_sigma = doc["imu"]["gyr_walk"].as<double>();

    // Extract the Tbg extrinsic data:
    YAML::Node tbgNode = doc["gps"]["Tbg"];
    if (!tbgNode) {
        Tbg = gtsam::I_4x4;
        return;
    }
    // Check the size of Tbc
    if (tbgNode.size() != 4) {
        throw std::runtime_error("Invalid size of 'Tbc' node in YAML file");
    }
    // Fill matrix with Tbc data
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            Tbg(i, j) = tbgNode[i][j].as<double>();
        }
    }
    VLOG(0) << "imu params: " << endl << "acc_sigma " << accelerometer_sigma << endl
            << "gyr_sigma " << gyroscope_sigma << endl << "acc_bias_sigma " << accelerometer_bias_sigma
            << endl << "gyr_bias_sigma "  << gyroscope_bias_sigma << endl <<"g " << g;
    VLOG(0) << " imu- gps extrinsic Tbg = " << endl << Tbg;

}

Eigen::MatrixXd kabsch(Eigen::MatrixXd A, Eigen::MatrixXd B){

    assert(A.rows() == B.rows());

    int n = A.rows();

    Eigen::VectorXd EA = A.colwise().mean();
    Eigen::VectorXd EB = B.colwise().mean();


    Eigen::MatrixXd H = ((A.rowwise() - EA.transpose()).transpose() * (B.rowwise() - EB.transpose())) / n;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd VT = svd.matrixV();
    Eigen::MatrixXd D =  svd.singularValues();

    double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
    if (d > 0)
        d = 1.0;
    else
        d = -1.0;


    Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
    I(2, 2) = d;

    Eigen::Matrix3d R = svd.matrixV() * I * svd.matrixU().transpose();

    VLOG(0) << "Rotation Kabsch: " << R << endl;

    return R;


}

void read_parameter_data(string filename){

    YAML::Node config = YAML::LoadFile(filename);

    data_file = config["imu_gps_datafile"].as<string>();
    calib_file = config["calib_file"].as<string>();
    result_file = config["result_file"].as<string>();
    gps_data_gt_file = config["gps_data_gt_file"].as<string>();

    if(calib_file == " "){
        VLOG(0) <<" calib file not found";
        exit(0);
    }

    //read imu related data
    read_gps_imu_data(calib_file);

}

gtsam::Pose3 gps_initialize_kabsch(){

    Eigen::MatrixXd gps_eigen(kabsch_data.size(), 3);
    Eigen::MatrixXd vins_eigen(kabsch_data.size(), 3);

    for(int i=0;i< (int)kabsch_data.size(); i++){
        for(int j=0; j<3; j++){
            gps_eigen(i ,j) = kabsch_data[i].gps_enu(j);
            vins_eigen(i, j) = kabsch_data[i].imu_translation(j);

        }
    }
    VLOG(0) <<"gps_eigen " << endl << gps_eigen;
    VLOG(0) <<"vins_eigen" << endl << vins_eigen;

    //steps:
    // 1. centroid and subtract from all points
    // 2. get eigen matrices for both points
    // 3. get the rotation matrix from the two eigen matrices
    // 4. get the translation vector from the two centroids
    // 5. create a transformation matrix from the rotation matrix and translation vector

    // 1. centroid and subtract from all points
    Eigen::Vector3d gps_centroid = gps_eigen.colwise().mean();
    Eigen::Vector3d vins_centroid = vins_eigen.colwise().mean();

    Eigen::MatrixXd gps_eigen_centered(gps_eigen.rows(), gps_eigen.cols());
    Eigen::MatrixXd vins_eigen_centered(vins_eigen.rows(), vins_eigen.cols());

    gps_eigen_centered = gps_eigen.rowwise() - gps_centroid.transpose();
    vins_eigen_centered = vins_eigen.rowwise() - vins_centroid.transpose();

    // 2. get eigen matrices for both points - already done

    // 3. get the rotation matrix from the two eigen matrices - rotation from gps -> vins frame
    Eigen::MatrixXd R_vins_gps = kabsch(gps_eigen_centered, vins_eigen_centered);


    // 4. get the translation vector from the two centroids
    Eigen::Vector3d t_vins_gps = vins_centroid - R_vins_gps * gps_centroid;

    // 5. create a transformation matrix from the rotation matrix and translation vector
    gtsam::Rot3 rot(R_vins_gps);
    gtsam::Point3 trans(t_vins_gps(0), t_vins_gps(1), t_vins_gps(2));
    gtsam::Pose3 V_T_E(rot, trans);

    Pose3 Enu_T_Vins = V_T_E.inverse();

    return Enu_T_Vins;

}

void write_optimized(string output_file, string gps_file, Values result, std::map<int, double> tStamps){
    std::ofstream fil(output_file);
    std::ofstream fil2(gps_file);
    gtsam::Values pose_values = result.filter<gtsam::Pose3>();
    int i = 0;
    for(gtsam::Values::iterator it = pose_values.begin(); it != pose_values.end() ; ++it) {
        gtsam::Symbol sym = it->key;
        int x_id = (int) sym.index();

        char ch = (char) sym.chr();

        if (ch == 't') {
            continue;
        }

        auto pose = pose_values.at<gtsam::Pose3>(sym);
        const gtsam::Rot3 &rotation_matrix = pose.rotation();
        gtsam::Quaternion quaternion = rotation_matrix.toQuaternion();
        gtsam::Point3 trans_vect = pose.translation();
        int stamp_key = x_id;
        double stamp = tStamps[stamp_key];

        fil << std::setprecision(5)<< std::fixed << stamp <<" "<< trans_vect.x() << " " <<
            trans_vect.y() << " " << trans_vect.z() << " " << quaternion.x() << " " << quaternion.y() << " "
            << quaternion.z() << " " << quaternion.w() << "\n";

        fil2 << std::setprecision(5)<< std::fixed << stamp <<" "<< gps_data[i].position.x() << " " <<
               gps_data[i].position.y() << " " << gps_data[i].position.z() << " " << 0 << " " << 0 << " "
            << 0 << " " << 1 << "\n";
        i++;


    }
    VLOG(0) <<"saved output";


}

gtsam::imuBias::ConstantBias computeIMUbias(std::deque<ImuMeasurement> imu_rest, double g){

    gtsam::Vector3 accelerometer_bias = Vector3::Zero();
    gtsam::Vector3 gyroscope_bias = Vector3::Zero();

    int size = imu_rest.size();
    for(int i = 0; i < size; i++){
        accelerometer_bias += imu_rest[i].accelerometer;
        gyroscope_bias += imu_rest[i].gyroscope;
    }
    accelerometer_bias /= size;
    gyroscope_bias /= size;

    accelerometer_bias(2) = accelerometer_bias(2) - g;

    VLOG(0) << " acceleromter bias is " << accelerometer_bias.x() << " " <<  accelerometer_bias.y() << " " <<  accelerometer_bias.z();
    VLOG(0) << " gyro bias is " << gyroscope_bias.x()   << " " << gyroscope_bias.y()  << " " <<  gyroscope_bias.z();

    gtsam::imuBias::ConstantBias imu_bias = gtsam::imuBias::ConstantBias(accelerometer_bias, gyroscope_bias);
    return imu_bias;

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
    bool imu_init = false;
    int x_id;


    // initial setup
    gtsam::Pose3 prevPose_;
    gtsam::Vector3  prevVel_;
    gtsam::imuBias::ConstantBias prev_bias; // assume zero initial bias
    gtsam::NavState prevState_(prevPose_, prevVel_);

    // Assume prior noise model and add it the graph.
    noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
    noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);  // m/s
    noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);;

    // graph related
    NonlinearFactorGraph *graph = new NonlinearFactorGraph();
    Values result;
    Values initial_values;

    // setup isam2 params and create isam2 solver object
    ISAM2 isam;


    // imu
    PreintegratedCombinedMeasurements *imu_preintegrated_;
    boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = PreintegratedCombinedMeasurements::Params::MakeSharedU(g);

    // We use the sensor specs to build the noise model for the IMU factor.
    double accel_noise_sigma = accelerometer_sigma;
    double gyro_noise_sigma =  gyroscope_sigma;
    double accel_bias_rw_sigma =  accelerometer_bias_sigma;
    double gyro_bias_rw_sigma =  gyroscope_bias_sigma;
    Matrix33 measured_acc_cov = Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
    Matrix33 measured_omega_cov = Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
    Matrix33 integration_error_cov = Matrix33::Identity(3,3)*1e-8; // error committed in integrating position from velocities
    Matrix33 bias_acc_cov = Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma,2);
    Matrix33 bias_omega_cov = Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma,2);
    Matrix66 bias_acc_omega_int = Matrix::Identity(6,6)*1e-2; // error in the bias used for preintegration

    p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
    p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
    p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
    p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
    p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
    p->biasAccOmegaInt = bias_acc_omega_int;



    // addl variables
    double prev_timestamp = 0;
    double last_imu_time = -1;
    int prev_xid = 0;
    gtsam::Pose3 Tbg;
    std::deque<ImuMeasurement> imu_rest_messages;
    int gps_msg_count = 0;
    bool E_T_V_added = false;
//    double angle = 0.785101;
//    double angle1 = M_PI_4;
    Pose3 E_T_V(Rot3::Rz(- 3 * M_PI_4), Point3(0.0, 0.0, 0.0));

    while(factor_file >> line){

        if(line == "imu_raw"){
            VLOG(0)<<"IMU mesg";
            ImuMeasurement imu;

            factor_file >> imu.time;
            for (int i = 0; i < 3 ; i++){
                factor_file >> imu.accelerometer(i);
            }
            for( int i = 0; i < 3; i++){
                factor_file >> imu.gyroscope(i);
            }

            if(imu_init == false){
                if(imu_rest_messages.size() < 200){
                    imu_rest_messages.push_back(imu);
                }
                else{
                    prev_bias = computeIMUbias(imu_rest_messages, g);
                    VLOG(0) <<"prev bias is "<<prev_bias;
                    imu_preintegrated_ = new PreintegratedCombinedMeasurements(p, prev_bias);
                    imu_init = true;
                    imu_rest_messages.clear();
                }

            }
            if(imu.time < prev_timestamp){
                VLOG(0) << " old imu message, ignoring";
                continue;
            }

            prev_imu_msgs.push_back(imu);
        }

        if(line == "g"){
            VLOG(0)<<"gps message";
            if(imu_init == false){
                VLOG(0) <<"IMU yet to be initialized";
                continue;
            }


            GpsMeasurement gps;
            factor_file >> x_id;
            factor_file >> gps.time;
            for(int i =0 ; i< 3; i++){
                factor_file >> gps.position(i);
            }
            // for jun12 bag
            if(x_id < 10){
                continue;
            }

            VLOG(1) << "=================";
            VLOG(1) <<"g" << x_id << ": "  << gps.position(0) <<", " << gps.position(1) << ", " << gps.position(2);
            gps_data.push_back(gps);
            tStamps[x_id] = gps.time;

            if(init == false){

//                prevPose_ = gtsam::Pose3(Rot3(), Vector3(gps.position(0) , gps.position(1), gps.position(2)));
                prevState_ = gtsam::NavState(prevPose_, prevVel_);

                graph->add(PriorFactor<Pose3>(X(x_id), prevPose_, pose_noise_model));
                graph->add(PriorFactor<Vector3>(V(x_id), prevVel_,velocity_noise_model));
                graph->add(PriorFactor<imuBias::ConstantBias>(B(x_id), prev_bias,bias_noise_model));



                VLOG(1) << " adding prior to x" << x_id <<" = " << endl << prevPose_ ;
                VLOG(1) << " adding prior to v" << x_id <<" = " << endl << prevVel_ ;
                VLOG(1) << " adding prior to b" << x_id <<" = " << endl << prev_bias ;

                initial_values.insert(X(x_id), prevPose_);
                initial_values.insert(V(x_id), prevVel_);
                initial_values.insert(B(x_id), prev_bias);


                VLOG(1) << " adding initial estimate to x" << x_id <<" = " << endl << prevPose_ ;
                VLOG(1) << " adding initial estimate to v" << x_id <<" = " << endl << prevVel_ ;
                VLOG(1) << " adding initial estimate to b" << x_id <<" = " << endl << prev_bias ;


                prev_xid = x_id;
                init = true;
                prev_imu_msgs.clear();
                gps_msg_count++;

                Umeyama msg;
                msg.gps_enu = gtsam::Vector3(gps.position(0), gps.position(1),0);
                msg.imu_translation = gtsam::Vector3(prevPose_.translation().x(), prevPose_.translation().y(), prevPose_.translation().z());
                kabsch_data.push_back(msg);

                // to debug
                VLOG(0) <<"gps time " << std::setprecision(15) << gps.time;

                continue;

            }
            //to debug
            int imu_count = 0;
            while(!(prev_imu_msgs.empty())){

                ImuMeasurement imu_msg = prev_imu_msgs.front();
                if(imu_msg.time <= gps.time){
                    double dt = (last_imu_time < 0) ? (0.0050) : (imu_msg.time - last_imu_time);
                    imu_preintegrated_->integrateMeasurement(imu_msg.accelerometer, imu_msg.gyroscope, dt);
                    gtsam::NavState propState = imu_preintegrated_->predict(prevState_, prev_bias);

//                    VLOG(1) << "dt " << dt;
//                    VLOG(1) << "last_imu time " << std::setprecision(15) << last_imu_time;
//                    VLOG(0) << "imu time " << std::setprecision(15) << imu_msg.time;
//                    VLOG(0) <<"prop state" << endl << propState;
//                    VLOG(1) << " imu  mesage acc " << imu_msg.accelerometer(0) << ", " << imu_msg.accelerometer(1) << ", " << imu_msg.accelerometer(2);
//                    VLOG(1) << " imu  mesage gyro " << imu_msg.gyroscope(0) << ", " << imu_msg.gyroscope(1) << ", " << imu_msg.gyroscope(2);
                    last_imu_time = imu_msg.time;
                    prev_imu_msgs.pop_front();

                    //to debug
                    imu_count++;


                }
                else{
                    last_imu_time = gps.time;
                    break;
                }

            }
            VLOG(0) <<"gps time " << std::setprecision(15) << gps.time;
            VLOG(0) << "imu_count " << imu_count;
            //add imu factor
            PreintegratedCombinedMeasurements *preint_imu_combined = dynamic_cast<PreintegratedCombinedMeasurements *>(imu_preintegrated_);
            CombinedImuFactor imu_factor(X(prev_xid), V(prev_xid),
                                         X(x_id), V(x_id),
                                         B(prev_xid), B(x_id),
                                         *preint_imu_combined);
            graph->add(imu_factor);

            VLOG(0) <<"Added imu factor between x" << prev_xid <<" and x"<<x_id;

            // add gps factor

            noiseModel::Diagonal::shared_ptr correction_noise = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 1));
            newGPSFactor gps_factor = newGPSFactor(X(x_id), gps.position, T(0), correction_noise
                                    ,Tbg);
//            GPSFactor gps_factor(X(x_id), gps.position, correction_noise);
            graph->add(gps_factor);
            VLOG(0) <<"Added gps factor between x" << x_id << " and t0";
            gps_msg_count++;

            // add initial values
            gtsam::NavState propState = imu_preintegrated_->predict(prevState_, prev_bias);
            initial_values.insert(X(x_id), propState.pose());
            initial_values.insert(V(x_id), propState.v());
            initial_values.insert(B(x_id), prev_bias);
            VLOG(1) << " adding initial estimate to x" << x_id <<" = " << endl << propState.pose() ;
            VLOG(1) << " adding initial estimate to v" << x_id <<" = " << endl << propState.v() ;
            VLOG(1) << " adding initial estimate to b" << x_id <<" = " << endl << prev_bias ;

            VLOG(0) <<"previous state" << endl << prevState_;
            VLOG(0) <<"prop state" << endl << propState;
            VLOG(0) <<" body vel prop " << endl << propState.bodyVelocity();

            // optimize the graph
            if(gps_msg_count < 4){
                VLOG(0) <<" not performing isam update";
                prev_xid = x_id;

                Umeyama msg;
                msg.gps_enu = gtsam::Vector3(gps.position(0), gps.position(1),0);
                msg.imu_translation = gtsam::Vector3(propState.pose().translation().x(), propState.pose().translation().y(), 0);
                kabsch_data.push_back(msg);
                continue;
            }
            else{
                if(E_T_V_added == false){

                    E_T_V = gps_initialize_kabsch();

                    graph->addPrior(gtsam::Symbol('t', 0), E_T_V, noiseModel::Diagonal::Sigmas(
                            (Vector(6) << 1e-2, 1e-2, 1e2, 1e2, 1e2, 1e2)
                                    .finished()));
                    initial_values.insert(T(0), E_T_V);
                    VLOG(1) << " adding initial estimate to t0 = " << endl << E_T_V ;
                    VLOG(0) << "angle measured is " << E_T_V.rotation().yaw() * 180.0 / M_PI << " degrees";
                    E_T_V_added = true;

                }

            }

            VLOG(0) <<"isam update";
            isam.update(*graph, initial_values);
            result = isam.calculateEstimate();
            initial_values.clear();
            graph->resize(0);

//            gtsam::LevenbergMarquardtParams Lparams;
//            Lparams.setVerbosityLM("SUMMARY");
//            gtsam::LevenbergMarquardtOptimizer optimizer(*graph, init  ial_values, Lparams);
//            Values result = optimizer.optimize();


            // update the variables for the next update
            prevPose_ = result.at<Pose3>(X(x_id));
            prevVel_ = result.at<Vector3 >(V(x_id));
            prev_bias = result.at<imuBias::ConstantBias>(B(x_id));
            prevState_ = gtsam::NavState(prevPose_, prevVel_);
            E_T_V = result.at<Pose3>(T(0));

            VLOG(0) <<"optimized state at x" << x_id << endl << prevState_;
            VLOG(0) <<" E_T_V after update " << E_T_V;
            VLOG(0) << "angle measured is " << E_T_V.rotation().yaw() * 180.0 / M_PI << " degrees";
            // reset the preintegration
            imu_preintegrated_->resetIntegrationAndSetBias(prev_bias);

            // update time variable
            prev_timestamp = gps.time;
            prev_xid = x_id;

        }
    }

    write_optimized(result_file, gps_data_gt_file ,result, tStamps);
}

int main(int argc , char** argv){

    string filename = "/home/auv/tri_ws/src/TRI-SLAM/test_scripts/min_working_ex/gps_imu_fusion/config/parameters.yaml";
    google::ParseCommandLineFlags(&argc, &argv, true);
    read_parameter_data(filename);
    optimize_data();

    VLOG(0) <<"done ";

}








