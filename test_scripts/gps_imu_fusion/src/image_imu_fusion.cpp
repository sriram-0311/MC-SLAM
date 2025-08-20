//
// Created by aryaman on 3/1/24.
//

// like the gps_imu_fusion1.cpp but with image and imu data fusion

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
#include <opencv4/opencv2/opencv_modules.hpp>
#include <boost/json.hpp>
//#include <opencv2/core/eigen.hpp>
#include <yaml-cpp/yaml.h>
#include <glog/logging.h>
#include "newGPSFactor.h"
#ifndef __has_include
static_assert(false, "__has_include not supported");
#else
#  if __cplusplus >= 201703L && __has_include(<filesystem>)
#    include <filesystem>
namespace fs = std::filesystem;
#  elif __has_include(<experimental/filesystem>)
#    include <experimental/filesystem>
     namespace fs = std::experimental::filesystem;
#  elif __has_include(<boost/filesystem.hpp>)
#    include <boost/filesystem.hpp>
     namespace fs = boost::filesystem;
#  endif
#endif



using namespace std;
//using namespace cv;
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

struct ImageMeasurement {
    double time;
//    Mat image;
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

void loadImageKittydata(vector<ImageMeasurement>& image_measurements, string image_data_file){
    //read all images in the folder
    // get all the images in the folder
    vector<string> fn;
    for(const auto & entry : fs::directory_iterator(image_data_file)){
        fn.push_back(entry.path());
    }

    // sort the images in ascending order
    sort(fn.begin(), fn.end());

    size_t count = fn.size(); //number of png files in images folder

    for (size_t i=0; i<count; i++){
        ImageMeasurement measurement;
        // strip the name of the file of the path and .png
        string name = fn[i].substr(fn[i].find_last_of("/\\") + 1);
        name = name.substr(0, name.find_last_of("."));
        // put a decimal point after the first 10 characters
        name.insert(10, ".");
        measurement.time = stod(name);
//        measurement.image = imread(fn[i]);
        image_measurements.push_back(measurement);
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


//gtsam::Pose3 convert_cv2_to_gtsam_pose3(cv::Mat pose){
//
//
//    gtsam::Rot3 R(pose.at<double>(0,0), pose.at<double>(0,1), pose.at<double>(0,2),
//                  pose.at<double>(1,0), pose.at<double>(1,1), pose.at<double>(1,2),
//                  pose.at<double>(2,0), pose.at<double>(2,1), pose.at<double>(2,2));
//
//    gtsam::Point3 t(pose.at<double>(0,3), pose.at<double>(1,3), pose.at<double>(2,3));
//
//    return gtsam::Pose3(R, t);
//
//}

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

int main(){

   // read from yaml file
   YAML::Node config = YAML::LoadFile("/home/aryaman/catkin_ws/src/TRI-SLAM/test_scripts/gps_imu_fusion/config/fusion_params.yaml");

   // read from config file
   string imu_data_file = config["imu_data_file"].as<string>();
   string image_data_file = config["image_data_file"].as<string>();
   double time_rest_end = config["imu_rest_time_max"].as<double>();
   double accelerometer_sigma = config["accelerometer_sigma"].as<double>();
   double gyroscope_sigma = config["gyroscope_sigma"].as<double>();
   double integration_sigma = 0.000000;
   double accelerometer_bias_sigma = config["accelerometer_bias_sigma"].as<double>();
   double gyroscope_bias_sigma = config["gyroscope_bias_sigma"].as<double>();
   double g = config["g"].as<double>();
   string output_file = config["imu_log_file"].as<string>();

   vector<ImuMeasurement> imu_measurements;
   vector<ImuMeasurement> imu_measurements_rest;
   vector<ImuMeasurement> imu_measurements_move;

   loadIMUKittidata(imu_measurements, imu_measurements_rest, imu_measurements_move, imu_data_file, time_rest_end);

   printf("IMU measurements: %d\n", (int)imu_measurements.size());

   // image measurements
   vector<ImageMeasurement> image_measurements;
   loadImageKittydata(image_measurements, image_data_file);

   printf("Image measurements: %d\n", (int)image_measurements.size());

   // IMU bias
   Vector3 accelerometer_bias = Vector3::Zero();
   Vector3 gyroscope_bias = Vector3::Zero();

   computeIMUbias(imu_measurements_rest, accelerometer_bias, gyroscope_bias, g);

   // make this the first pose
//    R: [
//    0.0617805, 0.997993, -0.01392;
//    -0.0047887, 0.0142429, 0.999887;
//    0.998078, -0.0617069, 0.00565902
//    ]
//    t: 002.24244 -0.776562 0017.9509

   gtsam::Pose3 pose = gtsam::Pose3(gtsam::Rot3(0.0617805, 0.997993, -0.01392, -0.0047887, 0.0142429, 0.999887, 0.998078, -0.0617069, 0.00565902), gtsam::Point3(2.24244, -0.776562, 17.9509));

   // make identity the starting pose state
//    gtsam::NavState prev_state = gtsam::NavState();
   gtsam::NavState prev_state = gtsam::NavState(pose, gtsam::Vector3(0, 0, 0));
   gtsam::imuBias::ConstantBias prev_bias  = gtsam::imuBias::ConstantBias(accelerometer_bias, gyroscope_bias);
   gtsam::Values initial_values;
   gtsam::NavState prop_state;
   gtsam::Pose3 currentPose;

   auto imu_params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(9.8);
   imu_params->accelerometerCovariance = I_3x3 * pow(accelerometer_sigma, 2);
   imu_params->integrationCovariance = I_3x3 * pow(integration_sigma, 2);
   imu_params->gyroscopeCovariance = I_3x3 * pow(gyroscope_sigma, 2);
   imu_params->biasAccCovariance = I_3x3 * pow(accelerometer_bias_sigma, 2);
   imu_params->biasOmegaCovariance = I_3x3 * pow(gyroscope_bias_sigma, 2);

   gtsam::PreintegratedCombinedMeasurements *imu_integrator_comb;

   imu_integrator_comb = new gtsam::PreintegratedCombinedMeasurements(imu_params, prev_bias);

   auto t_prev = image_measurements[0].time;

   std::map<int, double> timestamps;

   for (int i = 0; i < image_measurements.size(); i++){

       printf("Image time: %lf\n", image_measurements[i].time);
       int j = 0;

       double t = image_measurements[i].time;

       if (i == 0) {
//            currentPose = gtsam::Pose3();
           currentPose = pose;
           initial_values.insert(X(i), currentPose);
       }
       else {
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
       }

       cout<<"Previous pose before prediction: "<<prev_state.pose()<<endl;
       prop_state = imu_integrator_comb->predict(prev_state, prev_bias);
       cout<<"Pose after prediction: "<<prop_state.pose()<<endl;

       // update the state
       prev_state = prop_state;
       prev_bias = imu_integrator_comb->biasHat();

//        cout << "prop_state: " << prop_state << endl;

       // add the state to the initial values
       initial_values.insert(X(i+1), prop_state.pose());

       timestamps[i] = image_measurements[i].time;

       imu_integrator_comb->resetIntegrationAndSetBias(prev_bias);

       if (i > 100){
           break;
       }
   }

   writeOptimized(initial_values, timestamps, false, output_file);
}


