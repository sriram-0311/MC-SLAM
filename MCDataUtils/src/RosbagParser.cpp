#include "MCDataUtils/RosbagParser.h"


void RosbagParser::initialize(MCDataUtilSettings refocus_set){


    settings = refocus_set;
    read_rostopic_data(refocus_set);
    read_rosbag_data(refocus_set);
    VLOG(0) << "parsed all the ros data" ;
}



void RosbagParser::read_rostopic_data(MCDataUtilSettings settings){


    string path = settings.calib_file_path;
    LOG(INFO) << "Reading calibration data" <<path<<endl;

    FileStorage fs(path, FileStorage::READ);
    FileNode fn = fs.root();


    FileNodeIterator fi = fn.begin(), fi_end = fn.end();
    int i=0;
    for (; fi != fi_end; ++fi, i++) {

        FileNode f = *fi;
        if (f.name().find("cam",0) == string::npos)
            break;

        string cam_name;
        f["rostopic"]>>cam_name;
        rostopics.image_topics.push_back(cam_name);


        VLOG(0)<<"Camera "<<i<<" topic: "<< rostopics.image_topics[i]<<endl;

        string cam_model;
        f["camera_model"]>>cam_model;
        string dist_model;
        f["distortion_model"]>>dist_model;

        if (cam_model.compare("pinhole"))
            LOG(FATAL)<<"Only pinhole camera model is supported as of now!";


        // Reading distortion coefficients
        vector<double> dc;
        Mat_<double> dist_coeff = Mat_<double>::zeros(1,4);
        f["distortion_coeffs"] >> dc;
        for (int j=0; j < (int)dc.size(); j++)
            dist_coeff(0,j) = (double)dc[j];

        vector<int> ims;
        f["resolution"] >> ims;
        if (i>0) {
            if (((int)ims[0] != calib_img_size_.width) || ((int)ims[1] != calib_img_size_.height))
                LOG(FATAL)<<"Resolution of all images is not the same!";
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
                t(r,0) = (double)f2[3]; //we donot have a scale yet.. not sure what metrics this will be
            }
        }


        R_mats_kalibr.push_back(R.clone());
        t_vecs_kalibr.push_back(t.clone());
        bool CAMCHAIN = true;
        if(CAMCHAIN){
            // Converting R and t matrices to be relative to world coordinates
            if (i>0) {
                Mat R3 = R.clone()*R_mats_[i-1].clone();
                Mat t3 = R.clone()*t_vecs_[i-1].clone() + t.clone();
                R = R3.clone(); t = t3.clone();
            }
        }


        Mat Rt = build_Rt(R, t);
        Mat P = K_mat*Rt;

        VLOG(2)<<K_mat;
        VLOG(2)<<Rt;
        VLOG(3)<<P;

        R_mats_.push_back(R);
        t_vecs_.push_back(t);
        dist_coeffs_.push_back(dist_coeff);
        K_mats_.push_back(K_mat);
        P_mats_.push_back(P);




    }

    rosmessages.image_msgs.resize(rostopics.image_topics.size());
    num_cams_ = rostopics.image_topics.size();

    VLOG(0) << " imu data: " << settings.imu << endl;
    if(settings.imu){
        FileNode f = *fi;

        if (f.name().find("imu",0) != string::npos){
            string imu_topic_;
            f["rostopic"]>>imu_topic_;
            rostopics.imu_topic.push_back(imu_topic_);
            VLOG(0) << " imu topic:" << rostopics.imu_topic[0] << endl;
        }
        else{
            VLOG(0)<<"ERROR: No imu topic found in .yaml file. But Imu is set to true"<<endl;
        }

    }
    fi++;


    if(settings.gps){

        FileNode f1 = *fi;

        if (f1.name().find("gps",0) != string::npos){

            VLOG(0) << " found gps topic:" << settings.gps << endl;
            string gps_topic_;
            f1["rostopic"]>>gps_topic_;
            rostopics.gps_topic.push_back(gps_topic_);
            VLOG(0) << " gps topic:" << rostopics.gps_topic[0] << endl;

        }
        else{
            VLOG(0)<<"ERROR: No gps topic found in .yaml file. But gps is set to true"<<endl;
        }


    }

    img_size_ = calib_img_size_;

}

void RosbagParser::read_rosbag_data(MCDataUtilSettings settings){


    string rosbag_path = settings.rosbag_path;
    VLOG(0) << " Rosbag path is " << rosbag_path;

    parsing_thread = std::thread(&RosbagParser::parseBag, this, settings.rosbag_path, 0);
    parsing_thread.detach();
}

//// Destructor definition
//RosbagParser::~RosbagParser() {
//    // Set the stop_parsing flag to true
//    stop_parsing = true;
//
//    // Notify the parsing thread to wake up if it's currently waiting
//    queue_condition.notify_all();
//
//    // Join the parsing thread to ensure it finishes before the object is destroyed
//    if (parsing_thread.joinable()) {
//        parsing_thread.join();
//    }
//}

/*

write a function to read specific topics from the bag file and store the messages

*/

void RosbagParser::parseBag(string rosbag_path, int skip){


    rosbag::Bag bag;
    bag.open(rosbag_path, rosbag::bagmode::Read);

    //generate a list of topics to parse
    std::vector<std::string> topics;

    if(rostopics.image_topics.size() > 0){
        for(int i = 0 ; i < (int)rostopics.image_topics.size() ; i++){
            topics.push_back(rostopics.image_topics[i]);
        }
    }
    rosmessages.image_msgs.resize(rostopics.image_topics.size());

    if(rostopics.imu_topic.size() > 0){

        assert(settings.imu == 1);
        topics.push_back(rostopics.imu_topic[0]);
    }

    if(rostopics.gps_topic.size() > 0){

        assert(settings.gps == 1);
        topics.push_back(rostopics.gps_topic[0]);
    }

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    vector<int> image_count = (vector<int>)rostopics.image_topics.size();

    for (rosbag::MessageInstance& msg : view)
    {
//        std::lock_guard<std::mutex> lock(queue_mutex);
        const std::string& msg_topic = msg.getTopic();

        /// This code is to check different sections of the KRI dataset.
        //to make KRI work //1689805335 - parking lot // 1689804915 - main big loop // 1689805299 - Small loop 1686589803.50
        // relocalization 1689805080 // tri bag 1684172073.761915440 // 3rd loop 1689805237.26 // tri bag start - 1684172076.86
        // 1.676927878.2732682E9 //1709063860.10 //1709063231.65 // 1689618978.09 // 1689618975.491 // 1709063854.97 // 1709235407.56
        // 1692296763.34 // 1709573830.622 // 1709573833.522
//        if(msg.getTime().toSec() < 1689804910.07){
//            continue;
//        }
//        else if(msg.getTime().toSec() > 1689805102 ){
//            break;
//        }
        // for 12 jun gps2 bag
//                if(msg.getTime().toSec() <  1686589810){
//                    continue;
//                }


        if(rostopics.imu_topic.size() > 0){

            sensor_msgs::ImuConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
            if (imu_msg != nullptr && msg_topic == rostopics.imu_topic[0]) {
                const double imu_data_timestamp = imu_msg->header.stamp.toNSec();
                if (imu_data_timestamp > last_imu_timestamp) {

                    rosmessages.imu_msgs.push_back(imu_msg);
                    last_imu_timestamp = imu_data_timestamp;

                }
                else if (imu_data_timestamp <= last_imu_timestamp) {
                    LOG(WARNING) << "IMU message received out of order. Ignoring.";
                }
                continue;
            }

        }

        if(rostopics.gps_topic.size() > 0){

            sensor_msgs::NavSatFixConstPtr gps_msg = msg.instantiate<sensor_msgs::NavSatFix>();
            if (gps_msg != nullptr && msg_topic == rostopics.gps_topic[0]) {
                const double gps_data_timestamp = gps_msg->header.stamp.toNSec();
                if (gps_data_timestamp > last_gps_timestamp) {

                    rosmessages.gps_msgs.push_back(gps_msg);
                    last_gps_timestamp = gps_data_timestamp;

                }
                else if (gps_data_timestamp <= last_gps_timestamp) {
                    LOG(WARNING) << "GPS message received out of order. Ignoring.";
                }
                continue;
            }

        }

        //check if msg is  an image message and we need to send all camera images at once
        sensor_msgs::ImageConstPtr img_msg = msg.instantiate<sensor_msgs::Image>();
        //       VLOG(0) << "skip is " << skip_image << endl;
        if(img_msg != nullptr ){
            for(int i =0; i < (int)rostopics.image_topics.size() ; i++){

                if (msg_topic == rostopics.image_topics[i]) {
                    image_count[i]++;
                    rosmessages.image_msgs[i].push_back(img_msg);
                }
            }

        }
        else
        {
            VLOG(0) << "NULL Message - " << std::endl;
            continue;
        }

        pause_parsing = true;

        for (int i = 0; i < (int)rostopics.image_topics.size(); i++) {
            if (image_count[i] < 1) {
                pause_parsing = false;
            }
        }

        if(pause_parsing){

            VLOG(0) << "pause parsing" << endl;
            std::unique_lock<std::mutex> lock(pause_mutex);
            pause_thread = true;
            pause_condition.notify_all();
            pause_condition.wait(lock, [&]() { return !pause_thread; });
            image_count = (vector<int>)rostopics.image_topics.size();
            continue;

        }
    }

    parsing_done = true; // Set the flag to indicate that parsing is done
    bag.close();
}

/*
Different kinds of get next functions to send the messages to the mcslamapp file.
hoping it reads in a seuqential manner. check out datasetreader to see how images were being sent in a sequential manner.

imu timestamps and gps timestamps should be sent in a proper manner.


*/


void RosbagParser::getNext(vector<cv::Mat>& imgs, double& timeStamp){


    loadNext(imgs, timeStamp);
    if(total_image_count == 0){
        orig_time = timeStamp;
    }


    total_image_count++;
    VLOG(0) << std::setprecision(15) << "loaded next image at: " << timeStamp << endl;
    VLOG(0) << "total image count: " << total_image_count << endl;
    VLOG(0) << "time since start: " << timeStamp - orig_time << endl;
}


void RosbagParser::getNext(vector<cv::Mat>& imgs, double& timeStamp, std::deque<sensor_msgs::Imu>& imu_msgs){

    loadNext(imgs, imu_msgs, timeStamp);
    if(total_image_count == 0){
        orig_time = timeStamp;
    }
    total_image_count++;
    VLOG(0) << std::setprecision(20) << "loaded next image at: " << timeStamp << endl;
    VLOG(0) << std::setprecision(9) << "total image count: " << total_image_count << endl;
    VLOG(0) << std::setprecision(9) << "time since start: " << timeStamp - orig_time << endl;

}

void RosbagParser::getNext(vector<cv::Mat>& imgs , vector<string>& segmaskImgs,double& timeStamp){}

void RosbagParser::getNext(vector<cv::Mat>& imgs, double& timeStamp, std::deque<sensor_msgs::Imu>& imu_msgs, std::deque<sensor_msgs::NavSatFix>& gps_msgs){

    loadNext(imgs, imu_msgs, gps_msgs, timeStamp);
    if(total_image_count == 0){
        orig_time = timeStamp;
    }
    total_image_count++;
//    VLOG(0) << std::setprecision(20) << "loaded next image at: " << timeStamp << endl;
//    VLOG(0) << std::setprecision(9) << "total image count: " << total_image_count << endl;
//    VLOG(0) << std::setprecision(9) << "time since start: " << timeStamp - orig_time << endl;

}

void RosbagParser::loadNext(vector<cv::Mat>& imgs, double& timeStamp){

    for(int i = 0; i < (int)rostopics.image_topics.size() ; i++){

        assert(rosmessages.image_msgs[i].size() > 0);
        //
        sensor_msgs::ImageConstPtr img_msg = rosmessages.image_msgs[i].front();
        rosmessages.image_msgs[i].pop_front();
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
//            if (img_msg->height !=  1024 || img_msg->width != 1224) {
//                VLOG(0) << img_msg->header.stamp << std::endl;
//                VLOG(0) << "Image height and width - " << img_msg->height << ", " << img_msg->width << std::endl;
//                continue;
//            }

            if (img_msg->encoding == "bgr8") {
                cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
            } else {
                cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
            }
            Mat cvImg = cv_ptr->image;
            Mat img;
            cvImg.convertTo(img, CV_32F);
            img /= 255.0;
            imgs.push_back(img.clone());
            timeStamp = img_msg->header.stamp.toSec();

            if(i == 0){

                img_size_ = Size(img.cols, img.rows);
            }


        }
        catch (cv_bridge::Exception& e)
        {
            VLOG(0) << "cv_bridge exception: " << e.what();
            ROS_ERROR("cv_bridge exception: %s", e.what());
            continue;
        }


    }

    for(int i = 0; i < (int)rostopics.image_topics.size() ; i++){
        assert(rosmessages.image_msgs[i].size() == 0);
    }


    VLOG(0) << "rosmessages.image_msgs " << rosmessages.image_msgs.size();


    std::unique_lock<std::mutex> lock(pause_mutex);
    pause_thread = false;
    pause_condition.notify_all();
    lock.unlock();
}


void RosbagParser::loadNext(vector<cv::Mat>& imgs, std::deque<sensor_msgs::Imu>& imu_messages, double& timeStamp){

    for(int i = 0; i < (int)rostopics.image_topics.size() ; i++){
//        VLOG(0) << "rosmessages.image_msgs[i].size() " << rosmessages.image_msgs[i].size();
        assert(rosmessages.image_msgs[i].size() > 0);

        sensor_msgs::ImageConstPtr img_msg = rosmessages.image_msgs[i].front();
//        if(img_msg == nullptr){
//            continue;
//        }
        rosmessages.image_msgs[i].pop_front();
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            if (img_msg != nullptr) {
//            if (img_msg->height !=  1024 || img_msg->width != 1224) {
//                VLOG(0) << img_msg->header.stamp << std::endl;
//                VLOG(0) << "Image height and width - " << img_msg->height << ", " << img_msg->width << std::endl;
//                continue;
//            }
                if (img_msg->encoding == "mono8") {
                    cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
                } else {
                    cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
                }
                Mat cvImg = cv_ptr->image;
                Mat img;
                cvImg.convertTo(img, CV_32F);
                img /= 255.0;
                imgs.push_back(img.clone());
                timeStamp = img_msg->header.stamp.toSec();

                if (i == 0) {
                    img_size_ = Size(img.cols, img.rows);
                }
            }
            else{
                VLOG(0) << "Image message is NULL" << std::endl;
            }
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            continue;
        }


    }

    for(int i = 0; i < (int)rostopics.image_topics.size() ; i++){
        assert(rosmessages.image_msgs[i].size() == 0);
    }

    while(rosmessages.imu_msgs.size() > 0){

        sensor_msgs::Imu::ConstPtr imu_msg = rosmessages.imu_msgs.front();
        if(imu_msg->header.stamp.toSec() < timeStamp){
            imu_messages.push_back(*imu_msg);
            rosmessages.imu_msgs.pop_front();
            // VLOG(0) << std::setprecision(20) <<  " last imu timestamp: " << imu_messages.back().header.stamp.toSec() << endl;
        }

        else{
            if(imu_messages.size() > 0)
            {
                VLOG(0) << std::setprecision(20) << " last imu timestamp: " << imu_messages.back().header.stamp.toSec()
                        << endl;
            }
            LOG(WARNING) << "IMU message received greater than image timestamp. Ignoring.";
            break;
        }

    }

    VLOG(0) << "rosmessages.imu_msgs.size()" << rosmessages.imu_msgs.size();
    VLOG(0) << "rosmessages.image_msgs " << rosmessages.image_msgs.size();


    std::unique_lock<std::mutex> lock(pause_mutex);
    pause_thread = false;
    pause_condition.notify_all();
    lock.unlock();


}

void RosbagParser::loadNext(vector<cv::Mat>& imgs){}


void RosbagParser::loadNext(vector<cv::Mat>& imgs, std::deque<sensor_msgs::Imu>& imu_messages, std::deque<sensor_msgs::NavSatFix>& gps_messages,  double& timeStamp){


    if (parsing_done)
    {
        return;
    }
    for(int i = 0; i < (int)rostopics.image_topics.size() ; i++){

        assert(rosmessages.image_msgs[i].size() > 0);

        sensor_msgs::ImageConstPtr img_msg = rosmessages.image_msgs[i].front();
        if(img_msg==NULL){
            continue;
        }
//        rosmessages.image_msgs[i].pop_front();

        try
        {
            /// The below code is for the KRI Dataset.
//            if (img_msg->height !=  1024 || img_msg->width != 1224) {
//                VLOG(0) << img_msg->header.stamp << std::endl;
//                VLOG(0) << "Image height and width - " << img_msg->height << ", " << img_msg->width << std::endl;
//                continue;
//            }
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
            Mat cvImg = cv_ptr->image;
            Mat img;
            cvImg.convertTo(img, CV_32F);
            img /= 255.0;
            imgs.push_back(img.clone());
            timeStamp = img_msg->header.stamp.toSec();

            if(i == 0){
                img_size_ = Size(img.cols, img.rows);
            }
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            continue;
        }
        rosmessages.image_msgs[i].pop_front();

    }

    for(int i = 0; i < (int)rostopics.image_topics.size() ; i++){
        assert(rosmessages.image_msgs[i].size() == 0);
    }

    while(rosmessages.imu_msgs.size() > 0){

        sensor_msgs::Imu::ConstPtr imu_msg = rosmessages.imu_msgs.front();
        if(imu_msg->header.stamp.toSec() < timeStamp){
            imu_messages.push_back(*imu_msg);
            rosmessages.imu_msgs.pop_front();
            // VLOG(0) << std::setprecision(20) <<  " last imu timestamp: " << imu_messages.back().header.stamp.toSec() << endl;
        }

        else{
            if(imu_messages.size() > 0)
            {
                VLOG(0) << std::setprecision(20) << " last imu timestamp: " << imu_messages.back().header.stamp.toSec()
                        << endl;
            }
            LOG(WARNING) << "IMU message received greater than image timestamp. Ignoring.";
            break;
        }

    }

    while(rosmessages.gps_msgs.size() > 0){

        sensor_msgs::NavSatFix ::ConstPtr gps_msg = rosmessages.gps_msgs.front();
        if(gps_msg->header.stamp.toSec() < timeStamp){
            gps_messages.push_back(*gps_msg);
            rosmessages.gps_msgs.pop_front();
            // VLOG(0) << std::setprecision(20) <<  " last imu timestamp: " << imu_messages.back().header.stamp.toSec() << endl;
        }

        else{
            if(gps_messages.size() > 0)
            {
                VLOG(0) << std::setprecision(20) << " last gps timestamp: " << gps_messages.back().header.stamp.toSec()
                        << endl;
            }
            LOG(WARNING) << "GPS message received greater than image timestamp. Ignoring.";
            break;
        }

    }



//    VLOG(0) << "rosmessages.imu_msgs.size() " << rosmessages.imu_msgs.size();
//    VLOG(0) << "rosmessages.image_msgs " << rosmessages.image_msgs.size();
//    VLOG(0) << "rosmessages.gps_msgs " << rosmessages.gps_msgs.size();


    std::unique_lock<std::mutex> lock(pause_mutex);
    pause_thread = false;
    pause_condition.notify_all();
    lock.unlock();


}




/*
void RosbagParser::loadNext(vector<cv::Mat>& imgs, std::deque<sensor_msgs::Imu>& imu_messages, double& timeStamp){

    for(int i = 0; i < rostopics.image_topics.size() ; i++){

        if(rosmessages.image_msgs[i].size() > 0){

            sensor_msgs::ImageConstPtr img_msg = rosmessages.image_msgs[i].front();
            rosmessages.image_msgs[i].pop_front();

            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
                Mat cvImg = cv_ptr->image;
                Mat img;
                cvImg.convertTo(img, CV_32F);
                img /= 255.0;
                imgs.push_back(img.clone());
                timeStamp = img_msg->header.stamp.toSec();

                if(i == 0){

                    img_size_ = Size(img.cols, img.rows);
                }


            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

        }
    }
    while(rosmessages.imu_msgs.size() > 0){

        sensor_msgs::Imu::ConstPtr imu_msg = rosmessages.imu_msgs.front();
        // VLOG(0) << std::setprecision(20) << " imu timestamp: " << imu_msg->header.stamp.toSec() << endl;
        if(imu_msg->header.stamp.toSec() < timeStamp){
            imu_messages.push_back(*imu_msg);
            rosmessages.imu_msgs.pop_front();
            // VLOG(0) << std::setprecision(20) <<  " last imu timestamp: " << imu_messages.back().header.stamp.toSec() << endl;
        }

        else{
            if(imu_messages.size() > 0)
            {
                VLOG(0) << std::setprecision(20) << " last imu timestamp: " << imu_messages.back().header.stamp.toSec()
                        << endl;
            }
            LOG(WARNING) << "IMU message received greater than image timestamp. Ignoring.";
            break;
        }
    }
    return;

}

*/