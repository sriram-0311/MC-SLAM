// A code to rewrite IMU preintegration with the KITTI data available in the KittiIMUGPS code

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




void loadIMUKittidata(KittiCalibration& kitti_calibration, vector<ImuMeasurement>& imu_measurements){



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






	string imu_data_file = "/home/marley/neu_ws/Third_party/gtsam/examples/Data/KittiEquivBiasedImu.txt";
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

		// printf("time: %lf\n", measurement.time);

	measurement.dt = dt;
	measurement.accelerometer = Vector3(acc_x, acc_y, acc_z);
	measurement.gyroscope = Vector3(gyro_x, gyro_y, gyro_z);
	imu_measurements.push_back(measurement);
	}
}

printf("IMU measurements: %d\n", (int)imu_measurements.size());

}


int main(){


	KittiCalibration kitti_calibration;
	vector<ImuMeasurement> imu_measurements;
	loadIMUKittidata(kitti_calibration, imu_measurements);

	// using kitti values:
	double g = 9.8;
	auto w_coriolis = Vector3::Zero();

	auto current_pose_global = 	Pose3();

	// the vehicle is stationary at the beginning at position 0,0,0
	Vector3 current_velocity_global = Vector3::Zero();
	auto current_bias = imuBias::ConstantBias(); 


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

	



	// Set IMU preintegration parameters
	Matrix33 measured_acc_cov =
		I_3x3 * pow(kitti_calibration.accelerometer_sigma, 2);
	Matrix33 measured_omega_cov =
		I_3x3 * pow(kitti_calibration.gyroscope_sigma, 2);
	// error committed in integrating position from velocities
	Matrix33 integration_error_cov =
		I_3x3 * pow(kitti_calibration.integration_sigma, 2);

	auto imu_params = PreintegratedImuMeasurements::Params::MakeSharedU(g);
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
	// EDIT: file where the predicted values are stored
	string filename = "/home/marley/tri_ws/src/TRI-SLAM/t8/state_kitti_t1.txt";
	string filename_all = "/home/marley/tri_ws/src/TRI-SLAM/t8/state_kitti_t1_5.txt";
	myfile.open(filename);
	myfile1.open(filename_all);
	gtsam::NavState prevState_;


	// MAnually set this time based on kitti gps txt. in our case taking between 46540 and 46545 because x,y, and z are asymptotically increasing and it hleps to understand the movement.

	double t_previous = 46540.387861;
	double t_current = 46545.387071;

	// in this time interval: gps has moved: 16.916268020014168627,32.965313321824140758,0.17036437987999875077 to 32.999440612836473008,64.756789569115412064,0.36680603026999847316
	//  along x = 16.083172592822332381, y = 31.791476247291271306, z = 0.19644165038999972299


	current_summarized_measurement =
		std::make_shared<PreintegratedImuMeasurements>(imu_params,
														current_bias);


	while (j < imu_measurements.size()){
		
		if(imu_measurements[j].time <= t_current && imu_measurements[j].time > t_previous) {

			
			printf("imu_measurements[j].accelerometer: %f\n, %f\n, %f\n", imu_measurements[j].accelerometer.x(), imu_measurements[j].accelerometer.y(), imu_measurements[j].accelerometer.z());
			printf("imu_measurements[j].gyroscope: %f\n, %f\n, %f\n", imu_measurements[j].gyroscope.x(), imu_measurements[j].gyroscope.y(), imu_measurements[j].gyroscope.z());
			printf("imu_measurements[j].dt: %f\n", imu_measurements[j].dt);


			current_summarized_measurement->integrateMeasurement(
				imu_measurements[j].accelerometer, imu_measurements[j].gyroscope,
				imu_measurements[j].dt);
			included_imu_measurement_count++;

			gtsam::NavState next_state = current_summarized_measurement->predict(prevState_, current_bias);
			myfile1 << next_state.pose().x() << "," << next_state.pose().y() << "," << next_state.pose().z() << "," << next_state.velocity().x() << "," << next_state.velocity().y() << "," << next_state.velocity().z() << endl;
		}
			j++;
	}
	myfile1.close();


	NavState next_state = current_summarized_measurement->predict(prevState_, current_bias);

	myfile << next_state.pose().x() << "," << next_state.pose().y() << "," << next_state.pose().z() << "," << next_state.velocity().x() << "," << next_state.velocity().y() << "," << next_state.velocity().z() << endl;
	myfile.close();


}