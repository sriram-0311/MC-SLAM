// A code to rewrite IMU preintegration with the VectorNav data converted to KITTI format

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

    R = I;

   return R;

}

Eigen::Quaterniond  world_imu_frame(vector<ImuMeasurement>& imu_msgs, double g){


   Eigen::MatrixXd acceleration_matrix(imu_msgs.size(), 3);

   for(int i=0; i<imu_msgs.size(); i++){
       acceleration_matrix(i,0) = imu_msgs[i].accelerometer(0);
       acceleration_matrix(i,1) = imu_msgs[i].accelerometer(1);
       acceleration_matrix(i,2) = imu_msgs[i].accelerometer(2);
   }

   Eigen::MatrixXd g_matrix(imu_msgs.size(), 3);
   g_matrix.setZero();
   g_matrix.col(2) = Eigen::VectorXd::Constant(imu_msgs.size(), g);

   Eigen::MatrixXd R = kabsch(acceleration_matrix, g_matrix);
   Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::Matrix3d(R)).normalized();

   return q;

}





void loadIMUKittidata(KittiCalibration& kitti_calibration, vector<ImuMeasurement>& imu_measurements,  vector<ImuMeasurement>& imu_measurements_rest, vector<ImuMeasurement>& imu_measurements_move, string imu_data_file, double rest_end_time){


    // use the kitti metadata first to get the calibration parameters even for zed data

    string line;
    string imu_metadata_file = "/home/marley/neu_ws/Third_party/gtsam/examples/Data/KittiEquivBiasedImu_metadata.txt";
    ifstream imu_metadata(imu_metadata_file.c_str());

    printf("-- Reading sensor metadata\n");
    
    getline(imu_metadata, line, '\n');  // ignore the first line

    // Load Kitti calibration
   getline(imu_metadata, line, '\n');
   sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
          &kitti_calibration.body_ptx, &kitti_calibration.body_pty,
          &kitti_calibration.body_ptz, &kitti_calibration.body_prx,
          &kitti_calibration.body_pry, &kitti_calibration.body_prz,
          &kitti_calibration.accelerometer_sigma,
          &kitti_calibration.gyroscope_sigma,
          &kitti_calibration.integration_sigma,
          &kitti_calibration.accelerometer_bias_sigma,
          &kitti_calibration.gyroscope_bias_sigma,
          &kitti_calibration.average_delta_t);
   printf("IMU metadata: %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
          kitti_calibration.body_ptx, kitti_calibration.body_pty,
          kitti_calibration.body_ptz, kitti_calibration.body_prx,
          kitti_calibration.body_pry, kitti_calibration.body_prz,
          kitti_calibration.accelerometer_sigma,
          kitti_calibration.gyroscope_sigma, kitti_calibration.integration_sigma,
          kitti_calibration.accelerometer_bias_sigma,
          kitti_calibration.gyroscope_bias_sigma,
          kitti_calibration.average_delta_t);


    printf("-- Reading IMU measurements from file\n");



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


        if(measurement.dt < 2){
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

int main(){

    // true if rest data or false if move data
    bool rest = true;
    string imu_data_file;
    string filename;
    string filename_all;
    double t_previous;
    double t_current;

    // true if bias is being computed or false if bias is not being computed
    bool bias = false;

    // change the file name here 
    filename = "/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/vector_nav_rest_no_bias.txt";

    filename_all = "/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/vector_nav_rest_no_bias_5.txt";
    t_previous = 1682709193.0810857;
    t_current = 1682709198.665393;
    imu_data_file = "/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/vector_nav_combined.txt";

    // for vector nav it is z down
    double g = -9.8;

    KittiCalibration kitti_calibration;
    vector<ImuMeasurement> imu_measurements;
    vector<ImuMeasurement> imu_measurements_rest;
    vector<ImuMeasurement> imu_measurements_move;
    double time_rest_end = t_previous;
    loadIMUKittidata(kitti_calibration, imu_measurements, imu_measurements_rest, imu_measurements_move, imu_data_file, time_rest_end);
    
    Vector3 accelerometer_bias = Vector3::Zero();
    Vector3 gyroscope_bias = Vector3::Zero();
    // initialize gravitu vector to 0,0, 9.81
    Vector3 gravity = Vector3(0, 0, g);


   
    if (bias){


        for(int i = 0; i < imu_measurements_rest.size(); i++){

            accelerometer_bias += imu_measurements_rest[i].accelerometer;
            gyroscope_bias += imu_measurements_rest[i].gyroscope;

        }

        accelerometer_bias /= imu_measurements_rest.size();
        gyroscope_bias /= imu_measurements_rest.size();

        printf("accelerometer bias: %lf %lf %lf\n", accelerometer_bias.x(),
                accelerometer_bias.y(), accelerometer_bias.z());

        printf("gyroscope bias: %lf %lf %lf\n", gyroscope_bias.x(), gyroscope_bias.y(),
                gyroscope_bias.z());    

        gravity = Vector3::Zero();
        for (int i = 0; i < imu_measurements_rest.size(); i++) {
            gravity += imu_measurements_rest[i].accelerometer;
        }
        gravity /= imu_measurements_rest.size();

        printf("gravity: %lf %lf %lf\n", gravity.x(), gravity.y(), gravity.z());

    }

    else {

        accelerometer_bias = Vector3(0.000000, 0.000000, 0.000000);
        gyroscope_bias = Vector3(0.000000, 0.000000, 0.000000);

    }



    // [[ 1.00000000e+00  0.00000000e+00  2.88184436e-05]
//  [-3.32201077e-09 -9.99999993e-01  1.15273774e-04]
//  [ 2.88184434e-05 -1.15273774e-04 -9.99999993e-01]] - convert to eigen matrix

    Eigen::Matrix3d world_imu_matrix(3, 3);
    world_imu_matrix << 1.00000000e+00, 0.00000000e+00, 2.88184436e-05,
                        -3.32201077e-09, -9.99999993e-01, 1.15273774e-04,
                        2.88184434e-05, -1.15273774e-04, -9.99999993e-01;

    auto w_coriolis = Vector3::Zero();

    // convert world_imu_matrix to quaternion
    Eigen::Quaterniond q_w_i = Quaternion(world_imu_matrix);

     gtsam::Rot3 prior_rotation = gtsam::Rot3::Quaternion(q_w_i.w(), q_w_i.x(), q_w_i.y(), q_w_i.z());

    auto current_pose_global = Pose3(prior_rotation, Point3(0, 0, 0));

   // the vehicle is stationary at the beginning at position 0,0,0
   Vector3 current_velocity_global = Vector3::Zero();
   auto current_bias = imuBias::ConstantBias(); 

//    if its in rest then use this as the bias
    // let z of acceleormeter bias be 0 

    accelerometer_bias(2) = 0;

    current_bias = imuBias::ConstantBias(accelerometer_bias, gyroscope_bias);
    
    // print the bias
    printf("current_bias: %lf %lf %lf %lf %lf %lf\n", current_bias.accelerometer().x(),
        current_bias.accelerometer().y(), current_bias.accelerometer().z(),
        current_bias.gyroscope().x(), current_bias.gyroscope().y(),
        current_bias.gyroscope().z());


   auto sigma_init_x = noiseModel::Diagonal::Precisions(
       (Vector6() << Vector3::Constant(0), Vector3::Constant(1.0)).finished());
   auto sigma_init_v = noiseModel::Diagonal::Sigmas(Vector3::Constant(1000.0));
   auto sigma_init_b = noiseModel::Diagonal::Sigmas(
       (Vector6() << Vector3::Constant(0.100), Vector3::Constant(5.00e-05))
           .finished());


     // print kitti calibration values
    printf("kitti_calibration.accelerometer_sigma: %lf\n", kitti_calibration.accelerometer_sigma);
    printf("kitti_calibration.gyroscope_sigma: %lf\n", kitti_calibration.gyroscope_sigma);
    printf("kitti_calibration.integration_sigma: %lf\n", kitti_calibration.integration_sigma);
    printf("kitti_calibration.accelerometer_bias_sigma: %lf\n", kitti_calibration.accelerometer_bias_sigma);
    printf("kitti_calibration.gyroscope_bias_sigma: %lf\n", kitti_calibration.gyroscope_bias_sigma);

    
    double accelerometer_sigma = 0.0022;
    double gyroscope_sigma = 0.0066;
    double integration_sigma = 0.000;
    double accelerometer_bias_sigma = 0.0000322;
    double gyroscope_bias_sigma = 0.000262;

    // Set IMU preintegration parameters
   Matrix33 measured_acc_cov =
       I_3x3 * pow(accelerometer_sigma, 2);
   Matrix33 measured_omega_cov =
       I_3x3 * pow(gyroscope_sigma, 2);
   // error committed in integrating position from velocities
   Matrix33 integration_error_cov =
       I_3x3 * pow(integration_sigma, 2);
 
   double grav = gravity(2);

   printf("grav: %lf\n", grav);

   auto imu_params = PreintegratedImuMeasurements::Params::MakeSharedU(-grav);

   imu_params->accelerometerCovariance =
       measured_acc_cov;  // acc white noise in continuous
   imu_params->integrationCovariance =
       integration_error_cov;  // integration uncertainty continuous
   imu_params->gyroscopeCovariance =
       measured_omega_cov;  // gyro white noise in continuous
   imu_params->omegaCoriolis = w_coriolis;
 
   std::shared_ptr<PreintegratedImuMeasurements> current_summarized_measurement =
       nullptr;
 
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

    size_t j = 0;
   size_t included_imu_measurement_count = 0;


    printf(
       "Starting IMU preintegration \n");

    ofstream myfile;
    ofstream myfile1;
    myfile.open(filename);
    myfile1.open(filename_all);
    gtsam::NavState prevState_ = NavState(current_pose_global, current_velocity_global);


    current_summarized_measurement =
           std::make_shared<PreintegratedImuMeasurements>(imu_params,
                                                          current_bias);


    if(!rest){

    while (j < imu_measurements_move.size()){
    
        if(imu_measurements_move[j].time <= t_current && imu_measurements_move[j].time > t_previous) {

        
            printf("imu_measurements[j].accelerometer: %f\n, %f\n, %f\n", imu_measurements_move[j].accelerometer.x(), imu_measurements_move[j].accelerometer.y(), imu_measurements_move[j].accelerometer.z());
            printf("imu_measurements_move[j].gyroscope: %f\n, %f\n, %f\n", imu_measurements_move[j].gyroscope.x(), imu_measurements_move[j].gyroscope.y(), imu_measurements_move[j].gyroscope.z());
            printf("imu_measurements_move[j].dt: %f\n", imu_measurements_move[j].dt);


            current_summarized_measurement->integrateMeasurement(
                imu_measurements_move[j].accelerometer, imu_measurements_move[j].gyroscope,
                imu_measurements_move[j].dt);
            included_imu_measurement_count++;

            gtsam::NavState next_state = current_summarized_measurement->predict(prevState_, current_bias);
            myfile1 << next_state.pose().x() << "," << next_state.pose().y() << "," << next_state.pose().z() << "," << next_state.velocity().x() << "," << next_state.velocity().y() << "," << next_state.velocity().z() << endl;
        }
        j++;
    }

    }


    else{

        while (j < imu_measurements_rest.size()){
    
            if(imu_measurements_rest[j].time <= t_previous) {

            
                printf("imu_measurements[j].accelerometer: %f\n, %f\n, %f\n", imu_measurements_rest[j].accelerometer.x(), imu_measurements_rest[j].accelerometer.y(), imu_measurements_rest[j].accelerometer.z());
                printf("imu_measurements_rest[j].gyroscope: %f\n, %f\n, %f\n", imu_measurements_rest[j].gyroscope.x(), imu_measurements_rest[j].gyroscope.y(), imu_measurements_rest[j].gyroscope.z());
                printf("imu_measurements_rest[j].dt: %f\n", imu_measurements_rest[j].dt);


                current_summarized_measurement->integrateMeasurement(
                    imu_measurements_rest[j].accelerometer, imu_measurements_rest[j].gyroscope,
                    imu_measurements_rest[j].dt);
                included_imu_measurement_count++;

                gtsam::NavState next_state = current_summarized_measurement->predict(prevState_, current_bias);
                myfile1 << next_state.pose().x() << "," << next_state.pose().y() << "," << next_state.pose().z() << "," << next_state.velocity().x() << "," << next_state.velocity().y() << "," << next_state.velocity().z() << endl;
                printf("next_state.pose().x(): %f\n", next_state.pose().x());
                
                }
            j++;
        }

    }

    printf(" total imu measurements: %d\n", included_imu_measurement_count);
    printf(" Saving the state \n");
    myfile1.close();


    NavState next_state = current_summarized_measurement->predict(prevState_, current_bias);

    myfile << next_state.pose().x() << "," << next_state.pose().y() << "," << next_state.pose().z() << "," << next_state.velocity().x() << "," << next_state.velocity().y() << "," << next_state.velocity().z() << endl;
        

    printf(" Saving the state \n");
    myfile.close();


}