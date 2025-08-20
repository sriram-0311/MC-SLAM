//
// Created by Pushyami Kaveti on 9/16/19.
//
#pragma once
#include "DatasetReaderBase.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Imu.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/NavSatFix.h>


using namespace cv;
using namespace std;


class RosDataReader : public DatasetReaderBase{

public:
    //constructor and destructor
    RosDataReader(ros::NodeHandle nh): nh_(nh) , it_(nh_){

    }
    ~RosDataReader(){

    }

    bool CAMCHAIN;
    //Inner class
    class callBackFunctor
    {

    private:
        int counter;
        int cam_ind;
        RosDataReader *re;

    public:
        bool got_frame;
        callBackFunctor(RosDataReader &obj, int in){
            this->re = &obj;
            this->counter=0;
            this->cam_ind= in;
            this->got_frame = false;
            VLOG(2)<<" camera index in callbackfunctor : "<<this->cam_ind<<"\n";
        }

        int getcam_ind(){ return this->cam_ind ;}
        int getcounter(){ return this->counter ;}
        void CB(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
    };


    vector<cv::Mat> ros_imgs;

    // imu data
    std::deque<sensor_msgs::Imu> imu_msgs;
    std::deque<sensor_msgs::NavSatFix> gps_msgs;
    //todo: only required temporarily to save and read segmentation results. We should do this in a better way
    vector<string> ros_img_seq_nums;
    bool grab_frame;
    ros::Time tStamp = ros::Time();

    void initialize(MCDataUtilSettings refocus_set);
    void loadNext(vector<cv::Mat>& imgs);
    void getNext(vector<cv::Mat>& imgs, double& timeStamp);
    void getNext(vector<cv::Mat>& imgs , vector<string>& segmaskImgs,double& timeStamp);
    void getNext(vector<cv::Mat>& imgs, double& timeStamp, std::deque<sensor_msgs::Imu>& imu_msgs);
    void getNext(vector<cv::Mat>& imgs, double& timeStamp, std::deque<sensor_msgs::Imu>& imu_msgs, std::deque<sensor_msgs::NavSatFix>& gps_msgs);


    void imuCallback(const sensor_msgs::ImuPtr& msg);
    void subscribeToImu();
    void read_ros_data(MCDataUtilSettings settings);
    bool isDataLoaded();

    void loadNext(vector<cv::Mat>& imgset, std::deque<sensor_msgs::Imu>& imu_messages);
    void share_imu_data(std::deque<sensor_msgs::Imu>& imu_msgs, double image_t);

    void subscribeToGPS();
    void gpsCallback(const sensor_msgs::NavSatFixPtr& msg);
    void share_gps_data(std::deque<sensor_msgs::NavSatFix>& gps_messages, double image_t);
    void loadNext(vector<cv::Mat>& imgset, std::deque<sensor_msgs::Imu>& imu_msgs_out, std::deque<sensor_msgs::NavSatFix>& gps_messages_frontend);

private:
    //ROS variables
    ros::NodeHandle nh_;
    vector<string> cam_topics_;
    string imu_topic_;
    image_transport::ImageTransport it_;
    vector<image_transport::CameraSubscriber> cam_subs;
    ros::Subscriber imu_sub_;
    vector<sensor_msgs::ImagePtr> img_msgs;
    vector<sensor_msgs::CameraInfo> cam_info_msgs;
    vector<callBackFunctor*> cbs;
    std::mutex mBufMutex;
    
    string gps_topic_;
    ros::Subscriber gps_sub_;
    std::mutex mBufMutex_gps;
    
    
};



