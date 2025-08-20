//#include <iostream>
//#include <vector>
//#include "MCSlam/Preintegration.h"
//
//
//
//void IMUPreintegration::imu_initialize_noise(gtsam::imuBias::ConstantBias prior_imu_bias){
//
//
//    bias_prior = prior_imu_bias;
//
//    std::cout << "bias prior: " << bias_prior << endl;
//
//    auto w_coriolis = gtsam::Vector3(0, 0, 0);
//
//    // Create GTSAM preintegration params
//    params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(g_norm);
//
//   // // acc noise
//    params->setAccelerometerCovariance(gtsam::I_3x3 * pow(acc_n, 2));
//   // // gyro noise
//    params->setGyroscopeCovariance(gtsam::I_3x3 * pow(gyr_n, 2));
//   // // bias acc
//    params->biasAccCovariance = gtsam::I_3x3 * pow(acc_w, 2);
//   // // bias gyro
//    params->biasOmegaCovariance = gtsam::I_3x3 * pow(gyr_w, 2);
//    params->omegaCoriolis = w_coriolis;
//
//    double integration_sigma = 1e-8;
//    params->setIntegrationCovariance( gtsam::I_3x3 * pow(integration_sigma, 2) );  //1e-8
//
//   // //error in bias
//    params->biasAccOmegaInt = gtsam::I_6x6 * 1e0;// 1e-5;
//
//    priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
//    priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 0.1); // m/s
//    bias_noise_model = gtsam::noiseModel::Isotropic::Sigma(6,1e-3);
//
//   // Create GTSAM preintegration object
//    this->imu_integrator_comb = new gtsam::PreintegratedCombinedMeasurements(params, bias_prior);
//
//   return;
//
//}
//
//
//void IMUPreintegration::imu_initialize(std::deque<sensor_msgs::Imu> imu_msgs, double pose_time){
//
//
//
//
//    if(imu_msgs_initialized.size() < 200){
//            VLOG(0) << "IMU NOT INITIALIZED" << endl;
//            for(int i = 0; i < imu_msgs.size(); i++){
//                imu_msgs_initialized.push_back(imu_msgs[i]);
//            }
//            return;
//        }
//    Eigen::Quaterniond q = world_imu_frame(imu_msgs_initialized);
//
//
//    // calculate bias:
//        double acc_x_sum = 0;
//        double acc_y_sum = 0;
//        double acc_z_sum = 0;
//        double gyr_x_sum = 0;
//        double gyr_y_sum = 0;
//        double gyr_z_sum = 0;
//
//        for (int i = 0; i< imu_msgs_initialized.size(); i++){
//            double acc_x = imu_msgs_initialized[i].linear_acceleration.x;
//            double acc_y = imu_msgs_initialized[i].linear_acceleration.y;
//            double acc_z = imu_msgs_initialized[i].linear_acceleration.z;
//            double gyr_x = imu_msgs_initialized[i].angular_velocity.x;
//            double gyr_y = imu_msgs_initialized[i].angular_velocity.y;
//            double gyr_z = imu_msgs_initialized[i].angular_velocity.z;
//
//            acc_x_sum += acc_x;
//            acc_y_sum += acc_y;
//            acc_z_sum += acc_z;
//            gyr_x_sum += gyr_x;
//            gyr_y_sum += gyr_y;
//            gyr_z_sum += gyr_z;
//
//        }
//        bias_acc(0) = acc_x_sum / imu_msgs_initialized.size();
//        bias_acc(1) = acc_y_sum / imu_msgs_initialized.size();
//        bias_acc(2) = acc_z_sum / imu_msgs_initialized.size();
//
//        bias_gyr(0) = gyr_x_sum / imu_msgs_initialized.size();
//        bias_gyr(1) = gyr_y_sum / imu_msgs_initialized.size();
//        bias_gyr(2) = gyr_z_sum / imu_msgs_initialized.size();
//
//
//        Eigen::Vector4d q_coeffs(q.w(), q.x(), q.y(), q.z());
//
//        gtsam::Rot3 prior_rotation = gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z());
//        gtsam::Point3 prior_point(0.0, 0.0, 0.0);
//
//        gtsam::Pose3 prior_pose(prior_rotation, prior_point);
//        gtsam::Vector3 prior_velocity(0.0, 0.0, 0.0);
//
//        // correct bias_Acc w.r.t g_norm
//        bias_acc(2) = bias_acc(2) - g_norm;
//
//        // set bias_prior to be bias_acc and bias_gyr
//        gtsam::imuBias::ConstantBias prior_imu_bias(gtsam::Vector3(bias_acc(0), bias_acc(1), bias_acc(2)), gtsam::Vector3(bias_gyr(0), bias_gyr(1), bias_gyr(2)));
//
//        imu_initialize_noise(prior_imu_bias);
//
//        imu_initialized = true;
//        imu_msgs_initialized.clear();
//        VLOG(0) << "IMU INITIALIZED" << endl;
//        return;
//
//}
//
//
//Eigen::Quaterniond IMUPreintegration::world_imu_frame(std::deque<sensor_msgs::Imu>& imu_msgs){
//
//
//    Eigen::MatrixXd acceleration_matrix(imu_msgs.size(), 3);
//
//   for(int i=0; i<imu_msgs.size(); i++){
//       acceleration_matrix(i,0) = imu_msgs[i].linear_acceleration.x;
//       acceleration_matrix(i,1) = imu_msgs[i].linear_acceleration.y;
//       acceleration_matrix(i,2) = imu_msgs[i].linear_acceleration.z;
//   }
//
//   Eigen::MatrixXd g_matrix(imu_msgs.size(), 3);
//   g_matrix.setZero();
//
//   g_matrix.col(2) = Eigen::VectorXd::Constant(imu_msgs.size(), 9.81);
//   Eigen::MatrixXd R = kabsch(acceleration_matrix, g_matrix);
//   world_imu_rotation = R;
//   Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::Matrix3d(R)).normalized();
//
//   return q;
//
//
//
//}
//
//
//Eigen::MatrixXd IMUPreintegration::kabsch(Eigen::MatrixXd A, Eigen::MatrixXd B){
//
//   assert(A.rows() == B.rows());
//
//   int n = A.rows();
//   int m = A.cols();
//
//   Eigen::VectorXd EA = A.colwise().mean();
//   Eigen::VectorXd EB = B.colwise().mean();
//
//
//   Eigen::MatrixXd H = (A.rowwise() - EA.transpose()).transpose() * (B.rowwise() - EB.transpose()) / n;
//   Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
//
//   Eigen::MatrixXd U = svd.matrixU();
//   Eigen::MatrixXd VT = svd.matrixV();
//   Eigen::MatrixXd D =  svd.singularValues();
//
//   double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
//   if (d > 0)
//       d = 1.0;
//   else
//       d = -1.0;
//
//
//   Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
//   I(2, 2) = d;
//
//   Eigen::Matrix3d R = svd.matrixV() * I * svd.matrixU().transpose();
//
//    VLOG(0) << "Rotation Kabsch: " << R << endl;
//
//   return R;
//
//
//}
//
//
//void IMUPreintegration::imu_values_removal(double pose_time){
//
//    //unique mutex lock
//    unique_lock<mutex> lock(mMutexImu);
//    // remove messages that came before the first keyframe
//    while(!imu_msgs_combined.empty()){
//        double imuTime = imu_msgs_combined.front().header.stamp.toSec();
//        if(imuTime > pose_time){
//            break;
//        }
//        imu_msgs_combined.pop_front();
//    }
//
//}
//
//void IMUPreintegration::imu_preintegration(double image_time){
//
//    // preintegration
//    unique_lock<mutex> lock(mMutexImu);
//    ros::Time time_param(image_time);
//    ros::Time time_param2(image_time - prev_image_time);
//
//    VLOG(0) << "Time difference between images: " <<   image_time - prev_image_time << endl;
//    VLOG(0) << "IMU messages in this time interval: " <<  (image_time - prev_image_time) * 200<< endl;
//    ros::Time time_imu(imu_msgs_combined.front().header.stamp.sec);
//    ros::Time time_imu1(imu_msgs_combined.front().header.stamp.nsec);
//    int initial_size = imu_msgs_combined.size();
//    VLOG(0) << "mumber of imu messages before preintegration: " << imu_msgs_combined.size() << endl;
//    VLOG(0) << "image time: " << time_param.sec << "." << time_param.nsec<< endl;
//    while(!imu_msgs_combined.empty()){
//        double imuTime = imu_msgs_combined.front().header.stamp.toSec();
//
//        // assert (imuTime < image_time) ;
//        if(imuTime > image_time){
//            break;
//        }
//
//        double dt = (last_imu_time < 0) ? (1.0 / 200.0) : (imuTime - last_imu_time);
//
//
//        imu_integrator_comb->integrateMeasurement(gtsam::Vector3(imu_msgs_combined.front().linear_acceleration.x, imu_msgs_combined.front().linear_acceleration.y, imu_msgs_combined.front().linear_acceleration.z),
//                                                 gtsam::Vector3(imu_msgs_combined.front().angular_velocity.x, imu_msgs_combined.front().angular_velocity.y, imu_msgs_combined.front().angular_velocity.z),
//                                                 dt);
//
//
//        last_imu_time = imu_msgs_combined.front().header.stamp.toSec();
//        imu_msgs_combined.pop_front();
//    }
//
//    int final_size = imu_msgs_combined.size();
//    VLOG(0) << "imu messges removed: " << initial_size - final_size << endl;
//    prev_image_time = image_time;
//    VLOG(0) << "mumber of imu messages after preintegration: " << imu_msgs_combined.size() << endl;
//
//
//}
//
//
