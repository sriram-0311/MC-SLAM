/**
 * A Minimum working example code to understand the performance of the new custom GPS
 * factor by performing GPS + IMU fusion.
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

using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::T; // transform between gps and imu frame

struct GpsMeasurement {
    double time;
    Point3 position;
};

struct PoseMeasurement {
    double time;
    Pose3 pose;
};

string data_file = " ";
string output_file = " ";

std::map<int, int> tStamps;


void read_parameter_data(string filename){

    YAML::Node config = YAML::LoadFile(filename);

    data_file = config["gps_datafile"].as<string>();
    output_file = config["output_file"].as<string>();

}

void write_optimized(string filename, Values result, std::map<int, int> tStamps){
    std::ofstream fil(filename);
    gtsam::Values pose_values = result.filter<gtsam::Pose3>();

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

        fil << "o" << " " << std::setprecision(5)<< std::fixed << stamp <<" "<< trans_vect.x() << " " <<
           trans_vect.y() << " " << trans_vect.z() << " " << quaternion.x() << " " << quaternion.y() << " "
           << quaternion.z() << " " << quaternion.w() << "\n";


    }


}

void optimize_data(){

    if(data_file == " "){
        VLOG(0) <<"gps datafile not loaded";
        return;
    }

    std::ifstream factor_file(data_file.c_str());
    std::string line;
    VLOG(0) << "Reading the data file " << data_file;
    bool init = false;
    int x_id;
    bool added_transform = false;


    // initial setup
    gtsam::Pose3 prevPose_;

    // Assume prior noise model and add it the graph.
    noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8).finished()); // rad,rad,rad,m, m, m

    // graph related
    NonlinearFactorGraph *graph = new NonlinearFactorGraph();
    Values result;
    Values initial_values;

    // setup isam2 params and create isam2 solver object
    ISAM2 isam;

    // addl variables
    int poses_count = 0;
    gtsam::Pose3 prev_local_pose;
    int prev_xid = 0;
    gtsam::Pose3 Tbg;
//    double angle = 0.785101;
//    double angle1 = M_PI_4;
    Pose3 E_T_V(Rot3::Rz(M_PI_4), Point3(0.0, 0.0, 0.0));

    while(factor_file >> line){

        if(line == "x"){

            PoseMeasurement local_frame_pose;
            factor_file >> x_id;
            factor_file >> local_frame_pose.time;
            gtsam::Point3 position;
            for(int i =0 ; i< 3; i++){
                factor_file >>  position(i);
            }
            local_frame_pose.pose = gtsam::Pose3(Rot3(), gtsam::Vector3(position(0), position(1), position(2)));

            if(poses_count == 0){
                // fix initial pose as the first gps location
                prevPose_ = local_frame_pose.pose;

                // add the first local frame pose as the prior over the first pose
                graph->add(PriorFactor<Pose3>(X(x_id), prevPose_, pose_noise_model));

                // add initial estimates
                initial_values.insert(X(x_id), prevPose_);

                prev_xid = x_id;
                prev_local_pose = prevPose_;
                poses_count++;

                continue;

            }

            // add a between factor between the prev x_id and curr x_id
            gtsam::Pose3 currPose_ = local_frame_pose.pose;
            gtsam::Pose3 measured = (prev_local_pose.inverse().compose(currPose_));
            noiseModel::Diagonal::shared_ptr odom_noise = noiseModel::Diagonal::Sigmas(
                    (Vector(6) << 1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1).finished()); // rad,rad,rad,m, m, m
            graph->add(gtsam::BetweenFactor<Pose3>(X(prev_xid), X(x_id),
                                                   measured, odom_noise));
            VLOG(0) <<"Added a between factor between x" << prev_xid <<" and x"<<x_id;
            VLOG(1) <<"between pose " << measured;

            // add initial estimates for the local pose
            initial_values.insert(X(x_id), currPose_);
            VLOG(1) <<"curr Pose " << currPose_;


            prev_xid = x_id;
            prev_local_pose = currPose_;
            continue;
        }


        if(line == "g"){

            GpsMeasurement gps;
            factor_file >> x_id;
            factor_file >> gps.time;
            for(int i =0 ; i< 3; i++){
                factor_file >> gps.position(i);
            }
            tStamps[x_id] = gps.time;
            VLOG(1) << "=================";
            VLOG(1) <<"g" << x_id << ": "  << gps.position(0) <<", " << gps.position(1) << ", " << gps.position(2);

            // if this is the first time we are adding the gps factor, then add the transform variable too, for the
            // custom factor scenario

            // add initial estiamte for the transform - custom factor case
            if(added_transform == false){
                initial_values.insert(T(0), E_T_V);
                //add a weak prior
                graph->addPrior(T(0), E_T_V, noiseModel::Diagonal::Sigmas(
                        (Vector(6) << 1e-1, 1e-1, 1e5, 1e5, 1e5, 1e5)
                                .finished()));
                VLOG(0) << "initial yaw angle that E_T_V has " << E_T_V.rotation().yaw() * 180.0/3.141592653589793238463;
                added_transform = true;
            }

            noiseModel::Diagonal::shared_ptr correction_noise = noiseModel::Diagonal::Sigmas(
                    Vector3(1, 1, 0.01));
            newGPSFactor gps_factor = newGPSFactor(X(x_id), gps.position, T(0), correction_noise, Tbg);
            VLOG(0) <<"Added gps factor between x" << x_id << " and t0";

            // gtsam gps factor scenario
//            GPSFactor gps_factor(X(x_id), gps.position, correction_noise);
            graph->add(gps_factor);
//            VLOG(0) <<"Added gps factor at x" << x_id << " " << gps.position(0) << " " << gps.position(1) <<" " << gps.position(2);


            // optimize the graph
            if(x_id < 2){
                VLOG(0) <<" not performing isam update";
                prev_xid = x_id;
                continue;
            }

//            boost::shared_ptr<gtsam::GaussianFactorGraph> linearized_graph_init = graph->linearize(initial_values);
//            gtsam::Matrix jacobian_graph_init = (linearized_graph_init->jacobian()).first;
//            gtsam::Matrix U_, V_;
//            gtsam::Vector  S_;
//            gtsam::svd(jacobian_graph_init, U_, S_, V_);
//
//            VLOG(0) <<" Singular values are " << endl << S_;


            VLOG(0) <<"isam update";
            isam.update(*graph, initial_values);
            result = isam.calculateEstimate();
            initial_values.clear();


//            gtsam::LevenbergMarquardtParams Lparams;
//            Lparams.setVerbosityLM("SUMMARY");
//            gtsam::LevenbergMarquardtOptimizer optimizer(*graph, initial_values, Lparams);
//            Values result = optimizer.optimize();
//            graph->resize(0);


            // update the variables for the next update
            prevPose_ = result.at<Pose3>(X(x_id));
            E_T_V = result.at<Pose3>(T(0));
            VLOG(0) <<" E_T_V after update " << E_T_V;
            VLOG(0) <<"optimized state " << endl << prevPose_;
            VLOG(0) << "yaw angle that E_T_V computes " << E_T_V.rotation().yaw() * 180.0/3.141592653589793238463;


            // to debug
            // Linearize the graph.
//            VLOG(0) <<" linearize the graph and get the jacobians ";
//            boost::shared_ptr<gtsam::GaussianFactorGraph> linearized_graph = graph->linearize(result);
//            gtsam::Matrix jacobian_graph = (linearized_graph->jacobian()).first;
//            gtsam::Matrix U, V;
//            gtsam::Vector  S;
//            gtsam::svd(jacobian_graph, U, S, V);

//            VLOG(0) <<" Singular values are " << endl << S;
//            if(x_id > 5){
//                return;
//            }
        }

    }

    write_optimized(output_file, result, tStamps);

}

int main(int argc , char** argv){

    string filename = "/home/neural/catkin_ws/src/TRI-SLAM/test_scripts/min_working_ex/gps_tests/config/parameters.yaml";
    google::ParseCommandLineFlags(&argc, &argv, true);

    read_parameter_data(filename);
    optimize_data();
    VLOG(0) <<"done ";

}








