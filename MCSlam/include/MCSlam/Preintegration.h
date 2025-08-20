#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/inference/Symbol.h>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"





class IMUPreintegration {

    public:

        IMUPreintegration();
        ~IMUPreintegration();

        void imu_initialize_noise(gtsam::imuBias::ConstantBias prior_imu_bias);
        void imu_initialize(std::deque<sensor_msgs::Imu> imu_msgs, double image_time);
        void imu_values_removal(double image_time);
        void imu_preintegration(double image_time);
        Eigen::Quaterniond  world_imu_frame(std::deque<sensor_msgs::Imu>& imu_msgs);
        Eigen::MatrixXd kabsch(Eigen::MatrixXd A, Eigen::MatrixXd B);

        gtsam::PreintegratedCombinedMeasurements *imu_integrator_comb;
        double last_imu_time = -1;
        gtsam::imuBias::ConstantBias prior_imu_bias;
        Eigen::Matrix3d world_imu_rotation = Eigen::Matrix3d::Zero();


        gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
        gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
        gtsam::noiseModel::Diagonal::shared_ptr bias_noise_model;


        gtsam::imuBias::ConstantBias bias_prior, lastBias_;
        boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> params;

        struct IMUData{
            double time;
            Eigen::Vector3d acc;
            Eigen::Vector3d gyr;
        };


        std::deque<IMUData> imu_msgs_initialized;

};