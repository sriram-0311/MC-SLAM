// A code to rewrite IMU preintegration with the zed data converted to KITTI format

 #include <gtsam/inference/Symbol.h>
 #include <gtsam/navigation/CombinedImuFactor.h>
 #include <gtsam/navigation/GPSFactor.h>
 #include <gtsam/navigation/ImuFactor.h>
 #include <gtsam/nonlinear/ISAM2.h>
 #include <gtsam/nonlinear/ISAM2Params.h>
 #include <gtsam/nonlinear/NonlinearFactorGraph.h>
 #include <gtsam/slam/BetweenFactor.h>
 #include <gtsam/slam/PriorFactor.h>
 #include <gtsam/slam/dataset.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Eigen>



using namespace std;

using namespace gtsam;

 struct ImuMeasurement {
   double time;
   double dt;
   Vector3 accelerometer;
   Vector3 gyroscope;  // omega
 };

struct KittiCalibration {
   double body_ptx;
   double body_pty;
   double body_ptz;
   double body_prx;
   double body_pry;
   double body_prz;
   double accelerometer_sigma;
   double gyroscope_sigma;
   double integration_sigma;
   double accelerometer_bias_sigma;
   double gyroscope_bias_sigma;
   double average_delta_t;
 };

Eigen::MatrixXd kabsch(Eigen::MatrixXd A, Eigen::MatrixXd B){

   assert(A.rows() == B.rows());

   int n = A.rows();
   int m = A.cols();

   Eigen::VectorXd EA = A.colwise().mean();
   Eigen::VectorXd EB = B.colwise().mean();


   Eigen::MatrixXd H = (A.rowwise() - EA.transpose()).transpose() * (B.rowwise() - EB.transpose()) / n;
   //Eigen::MatrixXd H = A.transpose() * B/ n;
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

    // VLOG(0) << "Rotation: " << R << endl;
    // VLOG(0) << "!!!!!!!!!" << endl;

   // R = I;

   return R;

}
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
                 vector<ImuMeasurement>& imu_measurements_move, string imu_data_file, double rest_end_time, bool ntnu){


    // use the kitti metadata first to get the calibration parameters even for zed data

    string line;
    printf("-- Reading IMU measurements from file\n");

    {
     ifstream imu_data(imu_data_file.c_str());
     getline(imu_data, line, '\n');  // ignore the first line
     getline(imu_data, line, '\n');  // ignore the first two lines
 
     double time = 0, dt = 0, acc_x = 0, acc_y = 0, acc_z = 0, gyro_x = 0,
            gyro_y = 0, gyro_z = 0;
     double dum=0;
     while (!imu_data.eof()) {


       getline(imu_data, line, '\n');
       if (ntnu){
           sscanf(line.c_str(), "%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", &dum, &time, &dt,
                  &acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z);
       }
       else{
           sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &time, &dt,
                  &acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z);
       }

       ImuMeasurement measurement;
       measurement.time = time;

       measurement.dt = dt;

    if(measurement.dt < 1){

       measurement.accelerometer = Vector3(acc_x, acc_y, acc_z);
       measurement.gyroscope = Vector3(gyro_x* M_PI/180.0, gyro_y* M_PI/180.0, gyro_z* M_PI/180.0);
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


int main(){

    string imu_data_file;
    string filename;
    string filename_all;
    double t_previous;


    filename = "/data/NTNU/data/dvl_imu_december/txtfiles/imu_dvl_pipeline_following_last/df_imu.csv";

    t_previous = 1702406509.217640; //for zed 1916822112.2972548


    imu_data_file = "/home/auv/ros_ws/stim_flat_move.csv";

    // for vector nav it is z down
    double g = 9.81;

    KittiCalibration kitti_calibration;
    vector<ImuMeasurement> imu_measurements;
    vector<ImuMeasurement> imu_measurements_rest;
    vector<ImuMeasurement> imu_measurements_move;

    double time_rest_end = t_previous;


    loadIMUdata(imu_measurements, imu_measurements_rest, imu_measurements_move, imu_data_file, time_rest_end, true);

    printf("IMU measurements: %d\n", (int)imu_measurements.size());

    Vector3 gravity = Vector3(0, 0, g);


    Eigen::Matrix3d world_imu_matrix(3, 3);
    /* step 2: bias calculation */
    Vector3 accelerometer_bias = Vector3::Zero();
    Vector3 gyroscope_bias = Vector3::Zero();

    //estimate IMU orientation
    world_imu_frame(imu_measurements_rest, g,world_imu_matrix, accelerometer_bias, gyroscope_bias );
    printf("accelerometer bias: %lf %lf %lf\n", accelerometer_bias.x(),
           accelerometer_bias.y(), accelerometer_bias.z());

    printf("gyroscope bias: %lf %lf %lf\n", gyroscope_bias.x(), gyroscope_bias.y(),
           gyroscope_bias.z());


    //IMU parameters
    auto w_coriolis = Vector3::Zero();
    double accelerometer_sigma = 0.07 / 60.0;
    double gyroscope_sigma = 0.15 / 60.0 * M_PI / 180.0;
    double integration_sigma = 0.000000;
    double accelerometer_bias_sigma = 2.0/3600*(0.04 * 0.04);
    double gyroscope_bias_sigma = 2 / 3600 * 0.3 * 0.3 ;


    double grav = gravity(2);
    printf("grav: %lf\n", grav);
    auto imu_params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(grav);
   imu_params->accelerometerCovariance =  I_3x3 * pow(accelerometer_sigma, 2);  // acc white noise in continuous
   imu_params->integrationCovariance = I_3x3 * pow(integration_sigma, 2);  // integration uncertainty continuous
   imu_params->gyroscopeCovariance =  I_3x3 * pow(gyroscope_sigma, 2);  // gyro white noise in continuous
   imu_params->omegaCoriolis = w_coriolis;
   imu_params->biasAccCovariance = I_3x3 * accelerometer_bias_sigma;
   imu_params->biasOmegaCovariance = I_3x3 * gyroscope_bias_sigma;

   auto current_bias = imuBias::ConstantBias(accelerometer_bias, gyroscope_bias);
   gtsam::PreintegratedCombinedMeasurements *imu_integrator_comb;
   imu_integrator_comb = new gtsam::PreintegratedCombinedMeasurements(imu_params, current_bias);


    // rotaiton is basically identity
    auto current_pose_world = Pose3(Rot3(world_imu_matrix.transpose()), Vector3::Zero());
    // the vehicle is stationary at the beginning at position 0,0,0
    Vector3 current_velocity_world = Vector3::Zero();
    // print the bias
    printf("current_bias: %lf %lf %lf %lf %lf %lf\n", current_bias.accelerometer().x(),
           current_bias.accelerometer().y(), current_bias.accelerometer().z(),
           current_bias.gyroscope().x(), current_bias.gyroscope().y(),
           current_bias.gyroscope().z());
    auto pose_noise_model = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5)
                    .finished());  // rad,rad,rad,m, m, m
    auto velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);  // m/s
    auto bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-2);


    // Set ISAM2 parameters and create ISAM2 solver object
   ISAM2Params isam_params;
   isam_params.factorization = ISAM2Params::CHOLESKY;
   isam_params.relinearizeSkip = 10;
 
   ISAM2 isam(isam_params);
 
   // Create the factor graph and values object that will store new factors and
   // values to add to the incremental graph
   NonlinearFactorGraph new_factors;
   Values new_values;  // values storing the initial estimates of new nodes in
                       // the factor graph


    printf(
       "Starting IMU preintegration \n");

    ofstream myfile;
   
    myfile.open(filename);

    gtsam::NavState prevState_ = NavState(current_pose_world, current_velocity_world);

    size_t j = 0;

    double pose_z = 0.0;
    while (j < imu_measurements_move.size()){
    
        // printf(" time: %f\n", imu_measurements_rest[j].time);

        if(imu_measurements_move[j].time > t_previous) {


            //printf("imu_measurements[j].accelerometer: %f\n, %f\n, %f\n", imu_measurements_move[j].accelerometer.x(), imu_measurements_move[j].accelerometer.y(), imu_measurements_move[j].accelerometer.z());
           // printf("imu_measurements_rest[j].gyroscope: %f\n, %f\n, %f\n", imu_measurements_move[j].gyroscope.x(), imu_measurements_move[j].gyroscope.y(), imu_measurements_move[j].gyroscope.z());
            //printf("imu_measurements_rest[j].dt: %f\n", imu_measurements_move[j].dt);
            imu_integrator_comb->integrateMeasurement(imu_measurements_move[j].accelerometer, imu_measurements_move[j].gyroscope, imu_measurements_move[j].dt);

            gtsam::NavState next_state = imu_integrator_comb->predict(prevState_, current_bias);

            pose_z += next_state.v()(2) * imu_measurements_move[j].dt;
            myfile << std::setprecision(5)<< std::fixed << imu_measurements_move[j].time<<" "<<next_state.pose().x() << " " << next_state.pose().y() << " " << next_state.pose().z() << " " << \
            next_state.quaternion().x()<<" "<<next_state.quaternion().y()<<" "<<next_state.quaternion().z()<<" "<< \
            next_state.quaternion().w()<<endl; //<<"," <<next_state.velocity().x() << "," << next_state.velocity().y() << "," << next_state.velocity().z() << endl;
            printf("next_state x: %f, y: %f, z:%f, vel x:%f, vel y:%f, vel z:%f, pose_z : %f, dt : %f \n", next_state.pose().x(), next_state.pose().y(),next_state.pose().z(), \
                                                       next_state.v()(0), next_state.v()(1), next_state.v()(2), pose_z, imu_measurements_move[j].dt);
            }
        j++;
    }

}