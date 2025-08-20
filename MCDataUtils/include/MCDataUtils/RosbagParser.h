//
// Created by auv on 4/19/21.
//

#ifndef SRC_ROSBAGPARSER_H
#define SRC_ROSBAGPARSER_H

#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>

#include "DatasetReaderBase.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Imu.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/NavSatFix.h>

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional>
// using namespace std;
// using namespace cv;

class RosbagParser : public DatasetReaderBase {
    
    
    public:


        RosbagParser(){
        }
        ~RosbagParser() {
            // Set the stop_parsing flag to true
            stop_parsing = true;

            // // Notify the parsing thread to wake up if it's currently waiting
            // queue_condition.notify_all();

            // Join the parsing thread to ensure it finishes before the object is destroyed
            if (parsing_thread.joinable()) {
                parsing_thread.join();
            }
        }


        void initialize(MCDataUtilSettings refocus_set);
        void read_rostopic_data(MCDataUtilSettings settings);
        void read_rosbag_data(MCDataUtilSettings settings);

        void parseBag(string rosbag_path, int skip);
        void loadNext(vector<cv::Mat>& imgs, std::deque<sensor_msgs::Imu>& imu_messages, std::deque<sensor_msgs::NavSatFix>& gps_messages,  double& timeStamp);
        void loadNext(vector<cv::Mat>& imgs, std::deque<sensor_msgs::Imu>& imu_messages, double& timeStamp);
        void loadNext(vector<cv::Mat>& imgs, double& timeStamp);

        struct rostopic{
            
            std::vector<std::string> image_topics;
            std::vector<std::string> imu_topic;
            std::vector<std::string> gps_topic;
        };
        

        struct rosmessage{


            std::vector<std::deque<sensor_msgs::Image::ConstPtr>> image_msgs;
            std::deque<sensor_msgs::Imu::ConstPtr> imu_msgs;
            std::deque<sensor_msgs::NavSatFix::ConstPtr> gps_msgs;
        
        };


        void loadNext(vector<cv::Mat>& imgs);

        void loadNext(vector<cv::Mat>& imgset, std::deque<sensor_msgs::Imu>& imu_messages);
//        void loadNext(vector<cv::Mat>& imgset, std::deque<sensor_msgs::Imu>& imu_messages,double& timeStamp );
        void loadNext(vector<cv::Mat>& imgset, std::deque<sensor_msgs::Imu>& imu_msgs_out, std::deque<sensor_msgs::NavSatFix>& gps_messages_frontend);


        void getNext(vector<cv::Mat>& imgs, double& timeStamp);
        void getNext(vector<cv::Mat>& imgs , vector<string>& segmaskImgs,double& timeStamp);
        void getNext(vector<cv::Mat>& imgs, double& timeStamp, std::deque<sensor_msgs::Imu>& imu_msgs);
        void getNext(vector<cv::Mat>& imgs, double& timeStamp, std::deque<sensor_msgs::Imu>& imu_msgs, std::deque<sensor_msgs::NavSatFix>& gps_msgs);

        rostopic rostopics;
        rosmessage rosmessages;

        int rosmessages_image_size = 0;
        int current_image_count =0;
        vector<int> image_count;

        //new variables
        // Mutex to protect access to the deques
        std::mutex pause_mutex;
        bool pause_thread = true;
        bool pause_parsing = true;
        std::condition_variable pause_condition;


        // Concurrent queue to hold the parsed messages
        std::queue<rosbag::MessageInstance> msg_queue;
        std::mutex queue_mutex;
        std::condition_variable queue_condition;

        // Condition variable and mutex to handle chunk consumption
        std::condition_variable chunk_consumed_condition;
        std::mutex chunk_consumed_mutex;

        // Flag to indicate when parsing is done
        std::atomic<bool> parsing_done{ false };

        std::atomic<bool> stop_parsing{ false };

        // Member variables for chunk processing
        std::vector<cv::Mat> imgs_buffer;
        std::deque<sensor_msgs::Imu> imu_messages_buffer;
        std::deque<sensor_msgs::Imu::ConstPtr> unused_imu_messages;
        double last_imu_timestamp_buffer = 0.0;
        bool chunk_data_valid = false;

        // Thread for parsing
        std::thread parsing_thread;



    private:
        
        

        double last_imu_timestamp = -1;
        double last_gps_timestamp = -1;


        // debug variables
        int total_image_count = 0;
        double orig_time = 0;



};


#endif //SRC_ROSBAGPARSER_H
