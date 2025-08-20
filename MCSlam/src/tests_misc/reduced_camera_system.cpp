//
// Created by Pushyami Kaveti on 11/1/21.
//
#include <gflags/gflags.h>
#include <fstream>
#include <map>
#include <glog/logging.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/geometry/CameraSet.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <yaml-cpp/yaml.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/RegularImplicitSchurFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/triangulation.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <set>
#include <sstream>
#include <string>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/SmartProjectionRigFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <filesystem>
#include <boost/serialization/export.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/navigation/GPSFactor.h>
#include </home/neural/catkin_ws/src/TRI-SLAM/test_scripts/new_gps_factor/src/newGPSFactor.h>


BOOST_CLASS_EXPORT(gtsam::PreintegratedCombinedMeasurements);
//BOOST_CLASS_EXPORT(gtsam::PreintegrationCombinedParams);


using namespace std;
using namespace cv;
using namespace gtsam;

double FOCAL_LENGTH_X = 0.0;
double FOCAL_LENGTH_Y = 0.0;
double CENTER_X = 0.0;
double CENTER_Y = 0.0;

DEFINE_string(calib_file_mono, "", "A text file with calibration params for a single camera");
//DEFINE_string(calib_file, "/home/neural/catkin_ws/src/TRI-SLAM/MCApps/params/zed_calib/calib_zed.yaml", "A text file with calibration params");
//DEFINE_string(factor_file, "/home/neural/catkin_ws/src/TRI-SLAM/test_scripts/smart_factors/jul_17_bag2/graph_logs.txt", "A text file with poses and landmark initial values");
//DEFINE_string(parameter_file, "/home/neural/catkin_ws/src/TRI-SLAM/MCSlam/src/tests_misc/parameters.yaml", "Path to parameter file.");
//////
DEFINE_string(calib_file, "/home/neural/catkin_ws/src/TRI-SLAM/MCApps/params/nuance_calib/nuance.yaml", "A text file with calibration params");
DEFINE_string(factor_file, "/home/neural/catkin_ws/src/TRI-SLAM/test_scripts/smart_factors/kri/graph_logs.txt", "A text file with poses and landmark initial values");
DEFINE_string(parameter_file, "/home/neural/catkin_ws/src/TRI-SLAM/MCSlam/src/tests_misc/parameters.yaml", "Path to parameter file.");

vector<Matrix> K_mats_;
vector<Matrix> Rt_mats_;
vector<Matrix> Rt_mats_kalib_;
vector<Mat> t_vecs_ ;
vector<Mat> dist_coeffs_;
vector<Mat> R_mats_;
gtsam::Matrix Tbc(4,4);
gtsam::Matrix Tbg(4,4);


struct imu_xid{

    int x_id;
    int prev_xid;
    int next_xid = -1;
    double prev_timestamp;
    double curr_timestamp;
    double next_timestamp;
    bool gps;
    gtsam::Point3 gps_enu;

};
std::deque<imu_xid> xid_data;
struct ImuMeasurement {
    double time;
    Vector3 accelerometer;
    Vector3 gyroscope;
};
struct Umeyama {
    Vector3 gps_enu;
    Vector3 vins_translation;
};
std::deque<Umeyama> kabsch_data;
int kabsch_data_max = 50;

std::vector<ImuMeasurement> imu_rest;
double g, accelerometer_sigma, gyroscope_sigma, accelerometer_bias_sigma, gyroscope_bias_sigma, integration_sigma;

int gps;
int imu;
int first_xid;
Pose3 E_T_V(Rot3::Rz(M_PI_4 * 2.5), Point3(0.01, 0.01, 0.01));
int total_skipped_messages = 0;

gtsam::NonlinearFactorGraph graph;
gtsam::Values initialValues;

std::unordered_map<int, int> dropped_cam_poses;
std::map<int, int> cam_poses;
std::map<int, int> gps_poses;


Mat build_Rt(Mat R, Mat t) {

    Mat_<double> Rt = Mat_<double>::zeros(3,4);
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            Rt(i,j) = R.at<double>(i,j);
        }
        Rt(i,3) = t.at<double>(0,i);
    }
    return Rt;
}


/**
 *
 * Method that reads the calibration parameters
 */
void read_kalibr_chain(string path){

    VLOG(0) << "Reading calibration data" << path << std::endl;
    bool radtan = true;

    FileStorage fs(path, FileStorage::READ);
    FileNode fn = fs.root();

    FileNodeIterator fi = fn.begin(), fi_end = fn.end();
    int i=0;
    Size calib_img_size_;
    for (; fi != fi_end; ++fi, i++) {

        FileNode f = *fi;
        if (f.name().find("cam",0) == string::npos)
            break;

        // READING CAMERA PARAMETERS from here coz its only one time now due to static camera array
        // in future will have to subscribe from camera_info topic
        // Reading distortion coefficients
        vector<double> dc;
        Mat_<double> dist_coeff = Mat_<double>::zeros(1,5);
        f["distortion_coeffs"] >> dc;
        if(radtan){
            for (int j=0; j < dc.size(); j++)
                dist_coeff(0,j) = (double)dc[j];
        }else{

            for (int j=0; j < 3; j++){
                if( j < 2)
                    dist_coeff(0,j) = (double)dc[j];
                else
                    dist_coeff(0,j+2) = (double)dc[j];
            }
        }

        vector<int> ims;
        f["resolution"] >> ims;
        if (i>0) {
            if (((int)ims[0] != calib_img_size_.width) || ((int)ims[1] != calib_img_size_.height))
                LOG(FATAL) <<  "Resolution of all images is not the same!";
        } else {
            calib_img_size_ = Size((int)ims[0], (int)ims[1]);
        }

        // Reading K (camera matrix)
        vector<double> intr;
        f["intrinsics"] >> intr;
        Mat_<double> K_mat = Mat_<double>::zeros(3,3);
        K_mat(0,0) = (double)intr[0]; K_mat(1,1) = (double)intr[1];
        K_mat(0,2) = (double)intr[2]; K_mat(1,2) = (double)intr[3];
        K_mat(2,2) = 1.0;

        // Reading R and t matrices
        Mat_<double> R = Mat_<double>::zeros(3,3);
        Mat_<double> t = Mat_<double>::zeros(3,1);
        FileNode tn = f["T_cn_cnm1"];
        if (tn.empty()) {
            R(0,0) = 1.0; R(1,1) = 1.0; R(2,2) = 1.0;
            t(0,0) = 0.0; t(1,0) = 0.0; t(2,0) = 0.0;
        } else {
            FileNodeIterator fi2 = tn.begin(), fi2_end = tn.end();
            int r = 0;
            for (; fi2 != fi2_end; ++fi2, r++) {
                if (r==3)
                    continue;
                FileNode f2 = *fi2;
                R(r,0) = (double)f2[0]; R(r,1) = (double)f2[1]; R(r,2) = (double)f2[2];
                t(r,0) = (double)f2[3];  // * 1000; //. 1000 for real rig we donot have a scale yet.. not sure what metrics this will be
            }
        }
        /// store the pose of the cam chain
        Mat kalibrPose = Mat::eye(4,4, CV_64F);
        kalibrPose.rowRange(0,3).colRange(0,3) = R.t();
        kalibrPose.rowRange(0,3).colRange(3,4) = -1* R.t()*t;
        Matrix eigenRt_kalib;
        cv2eigen(kalibrPose, eigenRt_kalib);
        Rt_mats_kalib_.push_back(eigenRt_kalib);
        // Converting R and t matrices to be relative to world coordinates
        if (i>0) {
            Mat R3 = R.clone()*R_mats_[i-1].clone();
            Mat t3 = R.clone()*t_vecs_[i-1].clone() + t.clone();
            R = R3.clone(); t = t3.clone();
        }
        Mat Rt = build_Rt(R, t);
        Mat P = K_mat*Rt;
        std::string topic_name;
        Mat camPose = Mat::eye(4,4, CV_64F);
        camPose.rowRange(0,3).colRange(0,3) = R.t();
        camPose.rowRange(0,3).colRange(3,4) = -1* R.t()*t;
        Matrix eigenRt;
        cv2eigen(camPose, eigenRt);
        Rt_mats_.push_back(eigenRt);
        dist_coeffs_.push_back(dist_coeff);
        Matrix eigenK;
        cv2eigen(K_mat, eigenK);
        K_mats_.push_back(eigenK);
        R_mats_.push_back(R);
        t_vecs_.push_back(t);

    }

    YAML::Node doc = YAML::LoadFile(path);


    g = doc["imu"]["g_norm"].as<double>();
    accelerometer_sigma = doc["imu"]["acc_noise"].as<double>();
    gyroscope_sigma = doc["imu"]["gyr_noise"].as<double>();
    integration_sigma = 0.000000;
    accelerometer_bias_sigma = doc["imu"]["acc_walk"].as<double>();
    gyroscope_bias_sigma = doc["imu"]["gyr_walk"].as<double>();

    // Extract the data:
    YAML::Node tbcNode = doc["imu"]["Tbc"];
    if (!tbcNode) {
        Tbc = gtsam::I_4x4;
        return;
    }
    // Check the size of Tbc
    if (tbcNode.size() != 4) {
        throw std::runtime_error("Invalid size of 'Tbc' node in YAML file");
    }
    // Fill matrix with Tbc data
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            Tbc(i, j) = tbcNode[i][j].as<double>();
        }
    }
    VLOG(0) << Tbc << "  " << accelerometer_sigma << " " << gyroscope_sigma << " " << accelerometer_bias_sigma << " "  << gyroscope_bias_sigma;
//    Tbc = gtsam::Matrix::Identity(4,4);
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
    VLOG(0) << Tbc << "  " << accelerometer_sigma << " " << gyroscope_sigma << " " << accelerometer_bias_sigma << " "  << gyroscope_bias_sigma;
    VLOG(0) << " imu- gps extrinsic Tbg = " << endl << Tbg;


}

/**
 * Write the 3D points to the txt file.
 * @param output
 * @param filename
 */
void writeLandmarkPointCloud(const std::map<int, gtsam::Point3>& output, const std::string& filename)
{
//    gtsam::Values point_cloud = output.filter<gtsam::Point3>();
//    // Write the point cloud to a file :
//    std::ofstream file(filename);
//    for(gtsam::Values::iterator it = point_cloud.begin(); it != point_cloud.end(); it++)
//    {
//        auto point = point_cloud.at<gtsam::Point3>(it->key);
//        file << point.x() << " " << point.y() << " " << point.z() << "\n";
//    }
//    file.close();
    std::ofstream file(filename);
    for(const auto& entry : output)
    {
        const gtsam::Point3& point = entry.second;
        file << point.x() << " " << point.y() << " " << point.z() << "\n";
    }
    file.close();
}

/**
 * Writed the results of the optimization to a txt file in TUM format for eval.
 * @param result
 * @param tStamps
 * @param multi
 */
void writeOptimized(gtsam::Values& result,  std::map<int, double>& tStamps, bool multi, const std::string& filename)
{


    std::ofstream fil(filename);
    gtsam::Values pose_values = result.filter<gtsam::Pose3>();
    for(gtsam::Values::iterator it = pose_values.begin(); it != pose_values.end() ; ++it)
    {
        gtsam::Symbol sym = it->key;
        int x_id = (int)sym.index();

        char ch =  (char) sym.chr();

        if(ch == 't'){
            continue;
        }

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
//        VLOG(0) << "xid" << x_id << std::setprecision(15) << " time: " <<stamp;
                fil << std::setprecision(5)<< std::fixed << stamp <<" "<< trans_vect.x() << " " <<
            trans_vect.y() << " " << trans_vect.z() << " " << quaternion.x() << " " << quaternion.y() << " "
            << quaternion.z() << " " << quaternion.w() << "\n";

    }
    fil.close();
}


void store_gps_cam_pose_data(gtsam::Values& result, std::map<int, double>& tStamps, std::map<int, int> gps_poses, std::map<int, int> cam_poses, string filename ){

    std::ofstream fil(filename);
    gtsam::Values pose_values = result.filter<gtsam::Pose3>();
    int total_cam_poses = 0;
    int total_gps_poses = 0;
    int total_poses = 0;

    VLOG(0) <<" storing cam and gps data for plotting .. ";
    for(gtsam::Values::iterator it = pose_values.begin(); it != pose_values.end() ; ++it)
    {
        gtsam::Symbol sym = it->key;
        int x_id = (int)sym.index();

        char ch =  (char) sym.chr();

        if(ch == 't'){
            continue;
        }
        total_poses++;
        auto pose = pose_values.at<gtsam::Pose3>(sym);
        const gtsam::Rot3& rotation_matrix = pose.rotation();
        gtsam::Quaternion quaternion = rotation_matrix.toQuaternion();
        gtsam::Point3 trans_vect = pose.translation();

        int stamp_key = x_id;

        double stamp = tStamps[stamp_key];
        if(gps_poses[stamp_key] > 0){
            fil << "g" << " " << std::setprecision(5)<< std::fixed << stamp <<" "<< trans_vect.x() << " " <<
                trans_vect.y() << " " << trans_vect.z() << " " << quaternion.x() << " " << quaternion.y() << " "
                << quaternion.z() << " " << quaternion.w() << "\n";
            total_gps_poses++;
        }

        else if(cam_poses[stamp_key] > 0){
            fil << "c" << " " << std::setprecision(5)<< std::fixed << stamp <<" "<< trans_vect.x() << " " <<
                trans_vect.y() << " " << trans_vect.z() << " " << quaternion.x() << " " << quaternion.y() << " "
                << quaternion.z() << " " << quaternion.w() << "\n";
            total_cam_poses++;
        }
    }

    VLOG(0) <<" stored gps and cam data";
    VLOG(0) <<" total cam poses " << total_cam_poses;
    VLOG(0) <<" total gps poses " << total_gps_poses;
    VLOG(0) <<" total poses " << total_poses;
}


Eigen::MatrixXd kabsch(Eigen::MatrixXd A, Eigen::MatrixXd B){

    assert(A.rows() == B.rows());

    int n = A.rows();
    int m = A.cols();

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


gtsam::Pose3 gps_initialize_kabsch(){

    Eigen::MatrixXd gps_eigen(kabsch_data.size(), 3);
    Eigen::MatrixXd vins_eigen(kabsch_data.size(), 3);

    for(int i=0;i<kabsch_data.size(); i++){
        for(int j=0; j<3; j++){
            gps_eigen(i ,j) = kabsch_data[i].gps_enu(j);
            vins_eigen(i, j) = kabsch_data[i].vins_translation(j);

        }
    }


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


/**
 * Use the below structure if we are using serialized factors
 */
void addImuPreint(std::string folder_name, NonlinearFactorGraph& graph){



    namespace fs = std::filesystem;
    fs::path folder = folder_name;

    for(const auto& entry : fs::directory_iterator(folder)) {

        std::string filename = entry.path().filename();
        if(filename.substr(filename.find_last_of(".") + 1) == "bin") {

            size_t bin_pos = filename.find(".bin");
            size_t split_pos = filename.find('_');

            std::string kf1_str = filename.substr(0, split_pos);
            int kf1 = std::stoi(kf1_str);

            std::string kf2_str = filename.substr(split_pos+1, bin_pos-split_pos-1);
            int kf2 = std::stoi(kf2_str);

            gtsam::PreintegratedCombinedMeasurements preint_imu_combined;
            string file = folder_name + "/" + filename;

            std::ifstream ifs(file);
            boost::archive::binary_iarchive ia(ifs);

            VLOG(0)  << " adding imu factor between " << kf1 << " and " << kf2;
            ia >> preint_imu_combined;


            gtsam::CombinedImuFactor imu_factor(gtsam::Symbol('x', kf1 ), gtsam::Symbol('v', kf1), gtsam::Symbol('x', kf2), gtsam::Symbol('v', kf2), gtsam::Symbol('b', kf1), gtsam::Symbol('b', kf2), preint_imu_combined);
            graph.push_back(imu_factor);

        }

    }
}

void debug_factors(gtsam::Values initialValues, NonlinearFactorGraph graph){

    gtsam::Values pose_values = initialValues.filter<gtsam::Pose3>();
    gtsam::Values vel_values = initialValues.filter<gtsam::Vector3>();
    gtsam::Values bias_values = initialValues.filter<gtsam::imuBias::ConstantBias>();

    VLOG(0) << " pose values size " << pose_values.size();
    VLOG(0) << " vel values size " << vel_values.size();
    VLOG(0) << " bias values size " << bias_values.size();

    for(gtsam::Values::iterator it = pose_values.begin(); it != pose_values.end() ; ++it) {
        gtsam::Symbol sym = it->key;
        int x_id = (int) sym.index();
        char ch =  (char) sym.chr();

        if(ch == 't'){
            VLOG(0) << "transfomr " << sym;
        }
        else{
//            VLOG(0) << sym;

            if(!(initialValues.exists(gtsam::Symbol('v', x_id)))){
                VLOG(0) << " v" << x_id << " doesnt exist!!";
            }
            if(!(initialValues.exists(gtsam::Symbol('b', x_id)))){
                VLOG(0) << " b" << x_id << " doesnt exist!!";
            }

        }
    }

//    for (size_t i = 0; i < graph.size(); ++i) {
//        gtsam::NonlinearFactor::shared_ptr factor = graph.at(i);
//        const gtsam::CombinedImuFactor* CombinedImuFactors = dynamic_cast<const gtsam::CombinedImuFactor*>(graph.at(i).get());
//        if(CombinedImuFactors){
//            CombinedImuFactors->printKeys();
//        }
//
//        const newGPSFactor* newGPSFactors = dynamic_cast<const  newGPSFactor*>(graph.at(i).get());
//        if(newGPSFactors){
//            newGPSFactors->printKeys();
//        }
//
//        auto *priors = dynamic_cast<const gtsam::PriorFactor<gtsam::Pose3>*>(graph.at(i).get());
//        if(priors){
//            priors->print("Prior Factor");
//        }
//
////        factor->print("Factor");
//    }



}


gtsam::imuBias::ConstantBias computeIMUbias(std::vector<ImuMeasurement> imu_rest, double g){

    gtsam::Vector3 accelerometer_bias = Vector3::Zero();
    gtsam::Vector3 gyroscope_bias = Vector3::Zero();

    int size = imu_rest.size();
    if( imu_rest.size() > 200 ){
        size = 200;
    }

    for(int i = 0; i < size; i++){

        accelerometer_bias += imu_rest[i].accelerometer;
//        VLOG(0) << "accelerometer_bias " << accelerometer_bias;
        gyroscope_bias += imu_rest[i].gyroscope;
//        VLOG(0) << "gyroscope_bias " << gyroscope_bias;

    }
    accelerometer_bias /= size;
    gyroscope_bias /= size;

//    VLOG(0) << " imu rest size " << imu_rest.size();
//    VLOG(0) << " acceleromter bias is " << accelerometer_bias.x() << " " <<  accelerometer_bias.y() << " " <<  accelerometer_bias.z();

    accelerometer_bias(2) = accelerometer_bias(2) - g;

    VLOG(0) << " acceleromter bias is " << accelerometer_bias.x() << " " <<  accelerometer_bias.y() << " " <<  accelerometer_bias.z();
    VLOG(0) << " gyro bias is " << gyroscope_bias.x()   << " " << gyroscope_bias.y()  << " " <<  gyroscope_bias.z();

    gtsam::imuBias::ConstantBias imu_bias = gtsam::imuBias::ConstantBias(accelerometer_bias, gyroscope_bias);
    return imu_bias;

}

void addImufactors(NonlinearFactorGraph &graph, gtsam::Values &initialValues) {

    std::string line;
    std::ifstream factor_file(FLAGS_factor_file.c_str());
    double imu_tStamp;
    double last_imu_tStamp = -1;
    gtsam::imuBias::ConstantBias imu_bias;

    PreintegratedCombinedMeasurements *imu_preintegrated_;
    boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = PreintegratedCombinedMeasurements::Params::MakeSharedU(
            g);

    int total_imu_count = 0;
    std::deque<ImuMeasurement> prev_imu_msgs;
    int last_1 = 0; int last = 0;
    int prev_kf, curr_kf;
    imu_xid prev_info;
    bool addfactor_imu = true;
    // to debug
    int i = 0;
    int total_gps_messages = 0;
    int total_cam_gps_msgs = 0;


    gtsam::Rot3 rot(Tbg.block<3,3>(0, 0));
    gtsam::Point3 trans = Tbg.col(3).head<3>();
    gtsam::Pose3 Tbg_gtsam(rot, trans);

    while (factor_file >> line) {

        if (xid_data.empty()) {

            VLOG(0) << " xid is empty " << endl;
            break;
        }
        imu_xid recent_info = xid_data.front();
        if(xid_data.size() == 1){
            last_1 = recent_info.prev_xid;
            last = recent_info.x_id;
        }

        if (line == "imu_raw") {
            ImuMeasurement imu;
            factor_file >> imu_tStamp;
            for (int i = 0; i < 3; i++) {
                factor_file >> imu.accelerometer(i);
            }
            for (int i = 0; i < 3; i++) {
                factor_file >> imu.gyroscope(i);
            }
            imu.time = imu_tStamp;


            if (recent_info.prev_timestamp == -1) {
                if (imu_tStamp < recent_info.curr_timestamp) {
                    imu_rest.push_back(imu);
                } else {
                    imu_bias = computeIMUbias(imu_rest, g);
                    VLOG(0) << " imu bias " << imu_bias;

                    Matrix33 measured_acc_cov = Matrix33::Identity(3, 3) * pow(accelerometer_sigma, 2);
                    Matrix33 measured_omega_cov = Matrix33::Identity(3, 3) * pow(gyroscope_sigma, 2);
                    Matrix33 integration_error_cov =
                            Matrix33::Identity(3, 3) * 1e-8; // error committed in integrating position from velocities
                    Matrix33 bias_acc_cov = Matrix33::Identity(3, 3) * pow(accelerometer_bias_sigma, 2);
                    Matrix33 bias_omega_cov = Matrix33::Identity(3, 3) * pow(gyroscope_bias_sigma, 2);
                    Matrix66 bias_acc_omega_int =
                            Matrix::Identity(6, 6) * 1e-2; // error in the bias used for preintegration

                    p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
                    p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
                    p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
                    p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
                    p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
                    p->biasAccOmegaInt = bias_acc_omega_int;

                    imu_preintegrated_ = new PreintegratedCombinedMeasurements(p, imu_bias);

                    last_imu_tStamp = imu_tStamp;
                    prev_info = recent_info;
                    xid_data.pop_front();
                }
            } else {

                if (imu.time < recent_info.prev_timestamp) {
                    VLOG(0) << " old imu message - not using (shouldnt come here?)";
                }

                while (!prev_imu_msgs.empty()) {
                    ImuMeasurement imu_msg = prev_imu_msgs.front();
                    if (imu_msg.time < recent_info.prev_timestamp) {
                        prev_imu_msgs.pop_front();
                        continue;
                    } else if (imu_msg.time <= recent_info.curr_timestamp) {
                        double dt = (last_imu_tStamp < 0) ? (1.0 / 400.0) : (imu_msg.time - last_imu_tStamp);
                        imu_preintegrated_->integrateMeasurement(imu_msg.accelerometer, imu_msg.gyroscope, dt);
                        last_imu_tStamp = imu_msg.time;
                        total_imu_count++;
                        prev_imu_msgs.pop_front();

                    } else {
                        break;
                    }

                }

                // perform preintegration and add the combined imu factors
                if (imu.time <= recent_info.curr_timestamp) {
                    double dt = (last_imu_tStamp <= 0) ? (1.0 / 200.0) : (imu.time - last_imu_tStamp);
                    imu_preintegrated_->integrateMeasurement(imu.accelerometer, imu.gyroscope, dt);
                    last_imu_tStamp = imu.time;
                    total_imu_count++;
                }

                else if(imu.time > recent_info.curr_timestamp)
                {
                    // to debug
                    gtsam::NavState prevState = gtsam::NavState(
                            initialValues.at<Pose3>(gtsam::Symbol('x', recent_info.prev_xid)),
                            initialValues.at<Vector3>(gtsam::Symbol('v', recent_info.prev_xid)));
                    gtsam::NavState propState = imu_preintegrated_->predict(prevState, imu_bias);
//                    VLOG(0) << " Prev State at " << recent_info.prev_xid << " is " << endl << prevState;
//                    VLOG(0) << " Predicted State at " << recent_info.x_id << " is " << endl << propState;
                    prev_imu_msgs.push_back(imu);
                    PreintegratedCombinedMeasurements *preint_imu_combined = dynamic_cast<PreintegratedCombinedMeasurements *>(imu_preintegrated_);




                    if(recent_info.gps == true){

                        // if it's a gps, add a gps factor
                        if(!(initialValues.exists(gtsam::Symbol('t', 0)))){
                            // add the transformation E_T_V to the initialvalues and add a weak prior
                            //add a weak prior
//                            graph.addPrior(gtsam::Symbol('t', 0), E_T_V, noiseModel::Diagonal::Sigmas(
//                                    (Vector(6) << 2, 2, 2, 2, 2, 2)
//                                            .finished()));
                            initialValues.insert(gtsam::Symbol('t', 0), E_T_V);
                        }

                        /* a. there are scenarios where the xid and gps is very very close. kinda synced and hence 0 or 1 imu message present. this is also affecting the
                         * optimization. In this condition, we can directly add the gps onto the 'x' rather than use a propagated state to create a dummy keyframe.
                        */

                        if( (recent_info.next_xid == recent_info.x_id) && recent_info.next_xid!=-1){

                            prev_kf = recent_info.prev_xid;
                            curr_kf = recent_info.next_xid;

                            gtsam::noiseModel::Diagonal::shared_ptr  gpsNoise = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.5));
                            VLOG(0) << " gps very close to next xid " << recent_info.next_xid;
                            VLOG(0) << recent_info.prev_xid <<" -> " << recent_info.x_id <<" -> " << recent_info.next_xid;
                            VLOG(0) <<" gps factor between x" << recent_info.next_xid << " -> t" << 0;
                            auto gps_factor = newGPSFactor(gtsam::Symbol('x', recent_info.next_xid), recent_info.gps_enu, gtsam::Symbol('t', 0), gpsNoise, Tbg_gtsam);
                            graph.push_back(gps_factor);
                            total_gps_messages++;
                            total_cam_gps_msgs++;
                            gps_poses[recent_info.next_xid]++;
                            cam_poses[recent_info.next_xid]--;
//                            if( recent_info.curr_timestamp > 1689805034 &&  recent_info.curr_timestamp < 1689805052){
//                                total_skipped_messages--;
//                            }

                            // update the xid_data variable so that the prev xid info is updated to consider this sync.
                            VLOG(1) << "next state before upd" << xid_data[1].prev_xid << " -> " << xid_data[1].x_id << " -> " << xid_data[1].next_xid;

                            xid_data[1].prev_xid = prev_info.x_id;
                            xid_data[1].prev_timestamp = prev_info.curr_timestamp;

                            VLOG(1) << "next state after upd" << xid_data[1].prev_xid << " -> " << xid_data[1].x_id << " -> " << xid_data[1].next_xid;

                            VLOG(1) <<" recent info here is "<< recent_info.prev_xid <<" -> " << recent_info.x_id <<" -> " << recent_info.next_xid;
                            VLOG(1) <<" prev kf here is " << prev_info.prev_xid << " -> " << prev_info.x_id << " -> " << prev_info.next_xid;

                            addfactor_imu = false;

                        }
                        else if( recent_info.prev_xid == recent_info.x_id ) { // || recent_info.curr_timestamp - recent_info.prev_timestamp < 0.01){

                            // lets attach gps onto the prev x. and dont add imu factor as it already exists for the prev kf.
                            addfactor_imu = false;
                            gtsam::noiseModel::Diagonal::shared_ptr  gpsNoise = noiseModel::Diagonal::Sigmas(Vector3(1, 1, 1));
                            VLOG(0) << " gps very close to prev xid " << recent_info.prev_xid;
                            VLOG(1) << recent_info.prev_xid <<" -> " << recent_info.x_id <<" -> " << recent_info.next_xid;
                            VLOG(0) <<" gps factor between x" << recent_info.prev_xid << " -> t" << 0;
                            auto gps_factor = newGPSFactor(gtsam::Symbol('x', recent_info.prev_xid), recent_info.gps_enu, gtsam::Symbol('t', 0), gpsNoise, Tbg_gtsam);
                            graph.push_back(gps_factor);
                            total_gps_messages++;
                            total_cam_gps_msgs++;
                            gps_poses[recent_info.prev_xid]++;
                            cam_poses[recent_info.prev_xid]--;
                            if( recent_info.curr_timestamp > 1689805034 &&  recent_info.curr_timestamp < 1689805052){
                                total_skipped_messages--;
                            }

                            // update the xid_data variable so that the prev xid info is updated to consider this sync.
                            VLOG(1) << "next state before upd" << xid_data[1].prev_xid << " -> " << xid_data[1].x_id << " -> " << xid_data[1].next_xid;
                            xid_data[1].prev_xid = prev_info.x_id;
                            xid_data[1].prev_timestamp = prev_info.curr_timestamp;

                            VLOG(1) <<" recent info here is "<< recent_info.prev_xid <<" -> " << recent_info.x_id <<" -> " << recent_info.next_xid;
                            VLOG(1) <<" prev kf here is " << prev_info.prev_xid << " -> " << prev_info.x_id << " -> " << prev_info.next_xid;

//                            prev_info = recent_info;
                            // but recent info's next xid will be wrong
//                            prev_info.next_xid = recent_info.next_xid
//                            prev_info.next_timestamp = recent_info.next_timestamp;

                            VLOG(0) << "next state after upd" << xid_data[1].prev_xid << " -> " << xid_data[1].x_id << " -> " << xid_data[1].next_xid;

                        }

                        else {

                            prev_kf = recent_info.prev_xid;
                            curr_kf = recent_info.x_id;

//                            VLOG(0) <<" GPS data being added at " << recent_info.x_id ;
                            initialValues.insert(gtsam::Symbol('x', recent_info.x_id), propState.pose());
                            initialValues.insert(gtsam::Symbol('v', recent_info.x_id), propState.v());
                            initialValues.insert(gtsam::Symbol('b', recent_info.x_id), imu_bias);

                            //add the gps factor - custom vs original
                            gtsam::noiseModel::Diagonal::shared_ptr  gpsNoise = noiseModel::Diagonal::Sigmas(Vector3(1, 1, 1));
                            VLOG(0) <<" gps factor between x" << recent_info.x_id << " -> t" << 0;
                            auto gps_factor = newGPSFactor(gtsam::Symbol('x', recent_info.x_id), recent_info.gps_enu, gtsam::Symbol('t', 0), gpsNoise, Tbg_gtsam);
                            graph.push_back(gps_factor);

                            // update the bias
                            imu_bias = initialValues.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', recent_info.x_id));

                            gps_poses[recent_info.x_id]++;
                            total_gps_messages++;

                        }




                    }

                    else
                    {
                        // no gps. so imu factors between all states in the same order
                        prev_kf = recent_info.prev_xid;
                        curr_kf = recent_info.x_id;

                    }



                    if(addfactor_imu == false){
                        // dont add imu factor since it was added in the p[rev iterations
                        addfactor_imu = true;
                    }
                    else{
                        CombinedImuFactor imu_factor(gtsam::Symbol('x', prev_kf),
                                                     gtsam::Symbol('v', prev_kf),
                                                     gtsam::Symbol('x', curr_kf),
                                                     gtsam::Symbol('v', curr_kf),
                                                     gtsam::Symbol('b', prev_kf),
                                                     gtsam::Symbol('b', curr_kf),
                                                     *preint_imu_combined);
                        graph.add(imu_factor);

                        VLOG(0) << " adding imu factor between " << prev_kf<< " and " << curr_kf;


                        // update the bias
                        imu_bias = initialValues.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', curr_kf));
                        // Reset the preintegration object.
                        imu_preintegrated_->resetIntegrationAndSetBias(imu_bias);


                    }


                    if(!recent_info.gps){
                        gtsam::NavState nextState = gtsam::NavState(
                                initialValues.at<Pose3>(gtsam::Symbol('x', recent_info.x_id)),
                                initialValues.at<Vector3>(gtsam::Symbol('v', recent_info.x_id)));
//                        VLOG(0) << " Actual State at " << recent_info.x_id << " is " << endl << nextState;

                    }

                    // update to teh next recent info

                    prev_info = recent_info;
                    xid_data.pop_front();
                    total_imu_count = 0;

                }

            }

        }

    }
    if(xid_data.size() == 1){

        // add the last imu factor - a bug in the code wherein the last imu factor deosnt get added as there is no more data coming from the
        // factor file and it doesnt enter the else condition and add the factor. this resolves it.

        PreintegratedCombinedMeasurements *preint_imu_combined = dynamic_cast<PreintegratedCombinedMeasurements *>(imu_preintegrated_);
        CombinedImuFactor imu_factor(gtsam::Symbol('x', last_1),
                                     gtsam::Symbol('v', last_1),
                                     gtsam::Symbol('x', last),
                                     gtsam::Symbol('v', last),
                                     gtsam::Symbol('b', last_1),
                                     gtsam::Symbol('b', last),
                                     *preint_imu_combined);
        graph.add(imu_factor);
        VLOG(0) << "added imu factor = " << last_1 << " -> " << last;
    }
    VLOG(0) <<" total gps messages added as factors " << total_gps_messages;
    VLOG(0) <<" total gps messages added to cam poses " << total_cam_gps_msgs;

}

void SmartFactorsBA()
{
    // Read the factor file and the calibration parameters:
    std::ifstream calib_file(FLAGS_calib_file.c_str());
    std::ifstream factor_file(FLAGS_factor_file.c_str());
    typedef gtsam::SmartProjectionRigFactor<PinholePose<Cal3_S2>> RigFactor;
    // Safety:
    if(!calib_file || !factor_file) {
        std::cerr << "Error : Check paths to the files." << std::endl;
        exit(0);
    }
    YAML::Node config = YAML::LoadFile(FLAGS_parameter_file);
    auto pixel_noise = config["measurementNoise"].as<double>();
    gps = config["useGPS"].as<int>();
    imu = config["useIMU"].as<int>();
    int x_id, l_id, cam_id;
    bool init_prior_pose = true;
    int count_poses = 0;
    bool init_prior_landmark = true;
    double stamp, X, Y, Z, u, v;
    double angle_thresh = 0.1;
    std::string line;
    std::vector<double> u_coord, v_coord;
    std::vector<int> pose_ids, cam_ids;
    std::map<int, gtsam::MatrixRowMajor> chosen_poses;
    gtsam::MatrixRowMajor m(4,4);

    /// ISAM
    gtsam::ISAM2Params isam2Params;
    isam2Params.relinearizeThreshold = 0.1;
    isam2Params.factorization = ISAM2Params::QR;
    gtsam::ISAM2 isam(isam2Params);

    // Loop closure parameters:
    int curr_match;
    int matched_lid;
    int cam_Idx = 0;
    std::map<int, double> time_stamps;

    // Noise Model for measurement noise -
    static gtsam::SharedIsotropic meas_model(noiseModel::Isotropic::Sigma(2, pixel_noise));

    gtsam::Point2 meas;
    // Define the camera rig -
    // TODO: Can be made more generalized based on the types of cameras in the rig.
    boost::shared_ptr<CameraSet<PinholePose<Cal3_S2>>> cameraRig(new CameraSet<PinholePose<Cal3_S2>>());
    // Read the calibrations.
    read_kalibr_chain(FLAGS_calib_file);
    // Define the calibration parameters -
    int iter = 0;
    for (auto& item : K_mats_)
    {
        // For each camera get the K matrix(intrinsic), get the R matrix(extrinsic) and add to the cameraSet.
        gtsam::Cal3_S2::shared_ptr K(new Cal3_S2(item(0,0), item(1,1), 0, item(0,2), item(1,2)));
        VLOG(0) << "K - \n" << *K << std::endl;
        gtsam::PinholePose<Cal3_S2> camera;
        camera = gtsam::PinholePose<Cal3_S2>(gtsam::Pose3(Tbc).compose(gtsam::Pose3(Rt_mats_[iter])), K);

        VLOG(0) << "R - \n" << Rt_mats_[iter] << std::endl;
        cameraRig->push_back(camera);
        iter++;
    }
    std::string p;
    cameraRig->print(p);


    // Parameters -
    gtsam::SmartProjectionParams params(gtsam::HESSIAN,
                                        gtsam::ZERO_ON_DEGENERACY);
    std::map<int, RigFactor::shared_ptr> smartFactor;


    //imu related
    gtsam::Vector3 velocity;
    gtsam::Vector3 acc_bias;
    gtsam::Vector3 gyr_bias;
    int prev_xid = -1;
    double prev_timestamp = -1;
    bool imu_init = true;

    //to debug
    bool gps_added = false;
    int gps_count = 0;
    bool dropped_camera = false;
    int total_gps_messages = 0;
    int total_cam_messages = 0;

    while(factor_file >> line)
    {
        if(line == "x")
        {

            factor_file >> x_id;
            factor_file >> stamp;

//            if(x_id > break_point){
//                VLOG(0) << " break here at x_id " << x_id;
//                break;
//            }

//            if(stamp > 1689805034 && stamp < 1689805052){
//                VLOG(0) << " Ignoring x_id " << x_id;
//                dropped_cam_poses[x_id]++;
//                dropped_camera = true;
//                total_skipped_messages++;
//                continue;
//            }
            dropped_camera = false;
            // Get the pose of the state
            for (int i = 0; i < 16; i++) {
                factor_file >> m.data()[i];
            }
            gtsam::Pose3 pose(m);

            gtsam::Pose3 body_pose = pose.compose(gtsam::Pose3(Tbc.inverse()));


            if(init_prior_pose){
                init_prior_pose = false;
                gtsam::noiseModel::Diagonal::shared_ptr  poseNoise = gtsam::noiseModel::Diagonal::Sigmas(
                        (gtsam::Vector(6)<<gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.1)).finished());
                graph.addPrior(gtsam::Symbol('x', x_id), body_pose, poseNoise);
                first_xid = x_id;
            }

            //insert the initial value
            initialValues.insert(gtsam::Symbol('x', x_id), body_pose);
            cam_poses[x_id]++;
            chosen_poses[x_id] = m; // * Tbc.inverse();
            time_stamps[x_id] = stamp;
            count_poses++;

            // store the x_id, prev_xid, curr_tstamp, prev_timetamp - to assist with imu preintegration
            imu_xid xid_info;
            xid_info.x_id = x_id;
            xid_info.prev_xid = prev_xid;
            xid_info.curr_timestamp = stamp;
            xid_info.prev_timestamp = prev_timestamp;
            xid_info.gps = false;
            xid_info.next_xid = -1;
            xid_info.next_timestamp = -1.0;

            if(xid_data.size() > 0){
                xid_data.back().next_xid  = x_id;
                xid_data.back().next_timestamp = stamp;
            }

            xid_data.push_back(xid_info);
            total_cam_messages++;
            // update the variables
            prev_xid = x_id;
            prev_timestamp = stamp;

            continue;
        }
        else if(line == "g"){

            // to debug
            if(gps_added){
                continue;
            }

            if(gps == 0){
                continue;
            }
            if(imu == 0){
                VLOG(0) << " gps is swithced on but imu is off" << endl;
                exit(0);

            }
            double tStamp;
            factor_file >> x_id;
            factor_file >> tStamp;
            gtsam::Point3 gps_msg;
            for(int i = 0; i < 3; i++){
                factor_file >> gps_msg(i);
            }


            imu_xid xid_info;
            xid_info.x_id = x_id;
            xid_info.prev_xid = prev_xid;
            xid_info.curr_timestamp = tStamp;
            xid_info.prev_timestamp = prev_timestamp;
            xid_info.gps = true;
            xid_info.gps_enu = gps_msg;


            // update the variables
            prev_xid = x_id;

            if(!(prev_timestamp == -1)) {
                if (xid_data.size() > 0) {
                    // update the prev state's next xids.
                    xid_data.back().next_xid = x_id;
                    xid_data.back().next_timestamp = tStamp;
                }

                xid_data.push_back(xid_info);
//                gps_poses[x_id]++;
                total_gps_messages++;
                prev_timestamp = tStamp;
                time_stamps[x_id] = tStamp;
                gps_count++;

                if(kabsch_data.size() <= kabsch_data_max){
                    Umeyama data;
                    data.gps_enu = gps_msg;
                    if(!initialValues.exists(gtsam::Symbol('x', xid_info.prev_xid))){
                        continue;
                    }
                    data.vins_translation = initialValues.at<Pose3>(gtsam::Symbol('x', xid_info.prev_xid)).translation();
//                    VLOG(0) << "gps " << data.gps_enu;
//                    VLOG(0) << "data.vins_translation " << data.vins_translation;
                    kabsch_data.push_back(data);
//                    VLOG(0) << "kabsch size " << kabsch_data.size();
                }

            }

            continue;
        }


        else if(line == "imu_est")
        {
            if(imu == 0){
                continue;
            }

            factor_file >> x_id;
            if(dropped_cam_poses[x_id]>0){
                continue;
            }

            for(int i =0 ; i < 9; i++){
                if(i < 3){
                    factor_file >> velocity[i];
                }
                else if( i >= 3 && i< 6){
                    factor_file >> acc_bias[i-3];
                }
                else if( i>=6){
                    factor_file >> gyr_bias[i-6];
                }
            }
//            VLOG(0) <<" adding imu for xid " << x_id << " velocity " << velocity;
//            VLOG(0) << " bias " << x_id << " acc " << acc_bias << " ::  " << gyr_bias;
            initialValues.insert(gtsam::Symbol('v', x_id), velocity);
            initialValues.insert(gtsam::Symbol('b', x_id), gtsam::imuBias::ConstantBias(acc_bias, gyr_bias));

            if(imu_init){

                imu_init = false;
                gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
                gtsam::noiseModel::Diagonal::shared_ptr bias_noise_model = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);
                graph.addPrior(gtsam::Symbol('v', x_id), velocity);
                graph.addPrior(gtsam::Symbol('b', x_id), gtsam::imuBias::ConstantBias(acc_bias, gyr_bias));
            }
        }
        else if(line == "l")
        {   if(dropped_camera){
                continue;
                }

            if(u_coord.size() >= 2)
            {  // if the landmark is at least seen from two LF frames add the factor.
                gtsam::Point3 lm(X,Y,Z);
                // If new Landmark -
                if(smartFactor.count(l_id) == 0)
                {
                    smartFactor[l_id] = RigFactor::shared_ptr(new RigFactor (meas_model, cameraRig, params));
                    graph.push_back(smartFactor[l_id]);
                }
                for(int idx = 0 ; idx < u_coord.size() ; idx++ )
                {
                    gtsam::Point2 measurement(u_coord[idx], v_coord[idx]);
                    // Add to the smart factor
                    smartFactor[l_id]->add(measurement, gtsam::Symbol('x', pose_ids[idx]), cam_ids[idx]);
                }
                u_coord.clear();
                v_coord.clear();
                pose_ids.clear();
                cam_ids.clear();
            }
            else
            {
                u_coord.clear();
                v_coord.clear();
                pose_ids.clear();
                cam_ids.clear();
            }
            factor_file >> l_id >> X >> Y >> Z;
            continue;
        }
        else if (line == "e")
        {

            if(dropped_camera){
                continue;
            }

            factor_file >> x_id >> cam_id >> u >> v;
            // Don't add the negative measurements.
            if (u < 0.0 || v < 0.0)
            {
                continue;
            }
            if (chosen_poses.find(x_id) != chosen_poses.end()){
                // make sure that the angle between the rays is not acute.
                if(u_coord.size() >= 1){
                    // Check parallax
                    // Get the landmarks coordinates, prev pose id and u, v coordinates
                    gtsam::Vector3 pt(X,Y,Z);
                    int x_id_p = pose_ids[u_coord.size()-1];

                    // Get the Ray1 between landmark and the previous pose
                    gtsam::Vector3 ray1 = pt - chosen_poses[x_id_p].block(0,3,3,1);
                    double norm1 = ray1.norm();

                    // Get the Ray2 between landmark and the current pose
                    gtsam::Vector3 ray2 = pt - chosen_poses[x_id].block(0,3,3,1);
                    double norm2 = ray2.norm();

                    //  Compute the angle between the two rays
                    double cosTheta = ray1.dot(ray2)/(norm1*norm2);
                    double angle_deg = acos(cosTheta)*(180.0/3.141592653589793238463);
                    if(angle_deg > angle_thresh){
                        u_coord.push_back(u);
                        v_coord.push_back(v);
                        pose_ids.push_back(x_id);
                        cam_ids.push_back(cam_id);
                    }
                }
                else{
                    u_coord.push_back(u);
                    v_coord.push_back(v);
                    pose_ids.push_back(x_id);
                    cam_ids.push_back(cam_id);
                }

            }
            continue;
        }
        else if (line == "m")
        {
            factor_file >> curr_match >> matched_lid;
            if(smartFactor.count(matched_lid))
            {
                std::string point_str;
                std::getline(factor_file, point_str);
                std::istringstream iss(point_str);
                while (iss >> cam_Idx >> meas.x() >> meas.y())
                {
                    if (meas.x() < 0.0 || meas.y() < 0.0) {
                        continue;
                    }
                    // Add the measurement to the matched_lid
                    smartFactor[matched_lid]->add(meas, gtsam::Symbol('x', curr_match), cam_Idx);
                }
            }
            else
            {
                VLOG(0) << "L_ID does not exit - " << matched_lid << std::endl;
                continue;
            }
        }

    }
//    Values imu_prev = initialValues;
    if(gps == 1){

        E_T_V = gps_initialize_kabsch();
        VLOG(0) <<" check";
        kabsch_data.clear();
        VLOG(0) <<" total gps messages " << total_gps_messages;
        VLOG(0) <<" total cam messages " << total_cam_messages;

    }

    if(imu == 1){
        VLOG(0) << std::setprecision(15) << "Initial graph error before adding imu factors = " << graph.error(initialValues) << std::endl;
        addImufactors(graph, initialValues);

    }

    VLOG(0) <<" Total messages skipped = " << total_skipped_messages;
//    addImuPreint("/home/neural/catkin_ws/src/TRI-SLAM/test_scripts/smart_factors/imu_pre", graph);
    writeOptimized(initialValues, time_stamps, false, "/home/neural/traj.txt");
    if(imu == 1){
        debug_factors(initialValues, graph);
    }


    gtsam::LevenbergMarquardtParams Lparams;
    Lparams.setVerbosityLM("SUMMARY");
    Lparams.setOrderingType("COLAMD");
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initialValues, Lparams);
    VLOG(0) << "Optimizing the graph . . ." << std::endl;


    boost::shared_ptr<GaussianFactorGraph> ptr = graph.linearize(initialValues);
    GaussianFactorGraph graph1 = *ptr;
    VectorValues grad1 =   graph1.gradientAtZero();

    gtsam::Values result = optimizer.optimize();

    if(initialValues.exists(gtsam::Symbol('t', 0))){
        VLOG(0) <<" original E_T_V " << endl << E_T_V;
        VLOG(0) << " E_T_V optim " << endl << result.at<gtsam::Pose3>(gtsam::Symbol('t', 0));
    }


    boost::shared_ptr<GaussianFactorGraph> ptr2 = graph.linearize(result);
    GaussianFactorGraph graph2 = *ptr2;

    VectorValues grad2 =   graph2.gradientAtZero();
//    VLOG(0) << grad2 << endl;

//    VLOG(0) << std::setprecision(15) << " norm grad1 " << grad1.norm();
//    VLOG(0) << std::setprecision(15) << " norm grad2 " << grad2.norm();
    VLOG(0) << std::setprecision(15) << "Initial graph error = " << graph.error(initialValues) << std::endl;
    VLOG(0) << std::setprecision(15) << "Final graph error = " << graph.error(result) << std::endl;

    store_gps_cam_pose_data(result, time_stamps, gps_poses, cam_poses, "/home/neural/data_cam_gps.txt");

    // Triangulate the landmarks.
    std::map<int, gtsam::Point3> landmarks;
    for(auto iter = smartFactor.begin(); iter != smartFactor.end(); iter++)
    {
        // Get the camera/obs points from the results.
        auto cameras = smartFactor[iter->first]->cameras(result);
        // Gets the measurements of the observation points.
        auto measurements = smartFactor[iter->first]->measured();
        if(cameras.size() < 2)
        {
            VLOG(0) << "Degenerate case!" << std::endl;
            continue;
        }
        // get re-triangulated landmarks.
        TriangulationResult check_if_valid = smartFactor[iter->first]->triangulateSafe(cameras);
        if(check_if_valid.valid())
        {
            landmarks[iter->first] = gtsam::triangulatePoint3(cameras, measurements);
        }
        else
        {
//            VLOG(0) << "Not Valid" << std::endl;
        }
        // TODO: Use triangulateSafe to check the triangulated point. - DONE
    }
    writeOptimized(result, time_stamps, false, config["pathInitialTraj"].as<std::string>());
    writeLandmarkPointCloud(landmarks, "/home/neural/landmarks.txt");


}

int main(int argc, char** argv){

    //google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    //gtsamLMOptimization();
    SmartFactorsBA();
}