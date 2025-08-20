//
// Created by Pushyami Kaveti on 9/16/19.
//

#include "MCDataUtils/RosDataReader.h"

void RosDataReader::initialize(MCDataUtilSettings refocus_set) {

    settings = refocus_set;
    CAMCHAIN = true;
    img_counter = 0;
    read_ros_data( settings);

}

/// Call back method for images of each camera
/// \param msg
/// \param info_msg

void RosDataReader::callBackFunctor::CB(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        if (this->re->grab_frame && !this->got_frame) {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            this->re->ros_imgs[this->cam_ind] = cv_ptr->image;
            std::stringstream ss;
            ss<<msg->header.stamp;
            this->re->ros_img_seq_nums[this->cam_ind] = this->re->settings.segmasks_path+"/cam"+to_string(this->cam_ind) +"/"+ ss.str()+"."+this->re->MASK_TYPE; //msg->header.seq should be seq. but seeing some lag in recorded rosbag
            //Copy camera info to publish later for rectified images
            sensor_msgs::CameraInfo c; //=  boost::make_shared<sensor_msgs::CameraInfo>();
            c.header = info_msg->header;
            c.height = info_msg->height;
            c.width = info_msg->width;
            c.distortion_model = info_msg->distortion_model;
            c.D = info_msg->D;
            c.K = info_msg->K;
            c.R = info_msg->R;
            c.P = info_msg->P;
            c.binning_x = info_msg->binning_x;
            c.binning_y = info_msg->binning_y;
            c.roi = info_msg->roi;

            this->re->cam_info_msgs[this->cam_ind] = c;
            //copying done

            this->re->tStamp = msg->header.stamp;
            this->counter++ ;
            this->got_frame= true;
        }

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(RosDataReader::callBackFunctor* t = dynamic_cast<RosDataReader::callBackFunctor*>(this)) {
    }else{        ROS_ERROR("The objects is not of callBackFunctor. Check the code");
    }

}

void RosDataReader::subscribeToImu() {

    if(ros::ok()){
        imu_sub_ = nh_.subscribe(imu_topic_, 1000000, &RosDataReader::imuCallback, this);
        VLOG(1)<<"Subscribed to IMU topic"<<endl;
    }

}

void RosDataReader::imuCallback(const sensor_msgs::ImuPtr& msg) {
    std::unique_lock<std::mutex> lock(mBufMutex);
    sensor_msgs::Imu Imu = *msg;
    this->imu_msgs.push_back(Imu);

}

void RosDataReader::subscribeToGPS() {

    if(ros::ok()){

        gps_sub_ = nh_.subscribe(gps_topic_, 1, &RosDataReader::gpsCallback, this);

    }

}

void RosDataReader::gpsCallback(const sensor_msgs::NavSatFixPtr& msg) {
    std::unique_lock<std::mutex> lock(mBufMutex_gps);
    sensor_msgs::NavSatFix gps = *msg;
    this->gps_msgs.push_back(gps);
    VLOG(0) << "found gps message" << endl;
    
}


bool RosDataReader::isDataLoaded(){
    bool res= true;
    for (int i=0; i<cbs.size(); i++){
        if( !cbs[i]->got_frame)
            res= false;
    }
    return res;
}


void RosDataReader::loadNext(vector<cv::Mat>& imgset){

    VLOG(2)<<"GRABBING FRAMEs"<<endl;
    grab_frame=true;

    // load all imu messages
    while(!isDataLoaded()); 

    grab_frame = false;
    for (int i=0; i<cbs.size(); i++) {
        cbs[i]->got_frame = false;
    }
    VLOG(2)<<"GRABBING FRAMES DONE....."<<endl;
    // increment the image counter
    img_counter++;
    imgset.clear();
    for (int cam_ind=0; cam_ind< num_cams_; cam_ind++){
        Mat imgg;
        ros_imgs[cam_ind].convertTo(imgg, CV_32F);
        imgg /= 255.0;
        imgset.push_back(imgg.clone());

        if (cam_ind == 0){
            img_size_ = Size(imgg.cols, imgg.rows);
        }
    }

}


void RosDataReader::share_imu_data(std::deque<sensor_msgs::Imu>& imu_msgs_out, double image_t){

    std::unique_lock<std::mutex> lock(mBufMutex);
    // fo debug data
    sensor_msgs::Imu Imu_meas = imu_msgs.front();
    while(!imu_msgs.empty()){

        sensor_msgs::Imu Imu_meas = imu_msgs.front();
        double imu_time = Imu_meas.header.stamp.toSec();
        if(imu_time < image_t){

            imu_msgs_out.push_back(Imu_meas);
            imu_msgs.pop_front();
         }
        else{
          break;
        }
    }
}

void RosDataReader::share_gps_data(std::deque<sensor_msgs::NavSatFix>& gps_messages, double image_t)
{
    std::unique_lock<std::mutex> lock(mBufMutex_gps);
    while(!gps_msgs.empty()){
        sensor_msgs::NavSatFix gps_meas = gps_msgs.front();

        if(gps_meas.header.stamp.toSec() < image_t){
            gps_messages.push_back(gps_meas);
            gps_msgs.pop_front();
        }
        else{
            break;
        }
    }


}


void RosDataReader::loadNext(vector<cv::Mat>& imgset, std::deque<sensor_msgs::Imu>& imu_msgs_out, std::deque<sensor_msgs::NavSatFix>& gps_messages_frontend)
{


    VLOG(2)<<"GRABBING FRAMEs"<<endl;
    grab_frame=true;

    // load all imu messages
    while(!isDataLoaded()); 
    share_imu_data(imu_msgs_out, tStamp.toSec());
    share_gps_data(gps_messages_frontend, tStamp.toSec());

    if(isDataLoaded()){
        grab_frame = false;
        for (int i=0; i<cbs.size(); i++) {
            cbs[i]->got_frame = false;
        }
        VLOG(2)<<"GRABBING FRAMES DONE....."<<endl;
        // increment the image counter
        img_counter++;
        VLOG(2)<<"image timestamp : "<<tStamp;
        VLOG(2)<<"image counter : "<<img_counter;
        imgset.clear();
        for (int cam_ind=0; cam_ind< num_cams_; cam_ind++){
            Mat imgg;
            ros_imgs[cam_ind].convertTo(imgg, CV_32F);
            imgg /= 255.0;
            imgset.push_back(imgg.clone());

            if (cam_ind == 0){
                img_size_ = Size(imgg.cols, imgg.rows);
            }
        }
    }

}

void RosDataReader::loadNext(vector<cv::Mat>& imgset, std::deque<sensor_msgs::Imu>& imu_msgs_out){

    VLOG(2)<<"GRABBING FRAMEs"<<endl;
    grab_frame=true;

    // load all imu messages
    while(!isDataLoaded()); 
    share_imu_data(imu_msgs_out, tStamp.toSec());

    if(isDataLoaded()){
        grab_frame = false;
        for (int i=0; i<cbs.size(); i++) {
            cbs[i]->got_frame = false;
        }
        VLOG(2)<<"GRABBING FRAMES DONE....."<<endl;
        // increment the image counter
        img_counter++;
        VLOG(2)<<"image timestamp : "<<tStamp;
        VLOG(2)<<"image counter : "<<img_counter;
        imgset.clear();
        for (int cam_ind=0; cam_ind< num_cams_; cam_ind++){
            Mat imgg;
            ros_imgs[cam_ind].convertTo(imgg, CV_32F);
            imgg /= 255.0;
            imgset.push_back(imgg.clone());

            if (cam_ind == 0){
                img_size_ = Size(imgg.cols, imgg.rows);
            }
        }
    }

}

void RosDataReader::read_ros_data(MCDataUtilSettings settings){

    MASK_TYPE = settings.mask_type;
    string path = settings.calib_file_path;
    LOG(INFO) << "Reading calibration data from ROS ..."<<path<<endl;
    VLOG(2)<<"segmask path"<<settings.segmasks_path<<endl;

    FileStorage fs(path, FileStorage::READ);
    FileNode fn = fs.root();

    FileNodeIterator fi = fn.begin(), fi_end = fn.end();
    int i=0;
    for (; fi != fi_end; ++fi, i++) {

        FileNode f = *fi;
        if (f.name().find("cam",0) == string::npos)
            break;
        string cam_name; f["rostopic"]>>cam_name;

        cam_topics_.push_back(cam_name);


        VLOG(2)<<"Camera "<<i<<" topic: "<<cam_topics_[i]<<endl;

        RosDataReader::callBackFunctor* func = new RosDataReader::callBackFunctor(*this , i);
        cbs.push_back(func);
        VLOG(0) << cam_topics_[i] << endl;
        cam_subs.push_back(it_.subscribeCamera(cam_topics_[i], 1000 , &RosDataReader::callBackFunctor::CB, cbs[i]));
        //it_.subscribeCamera(cam_topics_[i], 1 , &RosDataReader::imgCallBack , this);
        // cout << "cam_subs.size() =  " << cam_subs.size() << endl;
        ros_imgs.push_back(Mat());
        ros_img_seq_nums.push_back("");
        cam_info_msgs.push_back(sensor_msgs::CameraInfo());
        // cam_subs.push_back(it_.subscribeCamera(cam_topics_[i], 1 , &saRefocus::camCallback, this));


        // READING CAMERA PARAMETERS from here coz its only one time now due to static camera array
        // in future will have to subscribe from camera_info topic
        // Reading distortion coefficients
        vector<double> dc;
        Mat_<double> dist_coeff = Mat_<double>::zeros(1,5);
        f["distortion_coeffs"] >> dc;
        if(settings.radtan){
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
                t(r,0) = (double)f2[3];  // * 1000; //. 1000 for real rig we donot have a scale yet.. not sure what metrics this will be
            }
        }

        R_mats_kalibr.push_back(R.clone());
        t_vecs_kalibr.push_back(t.clone());

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
        VLOG(2)<<P;

        R_mats_.push_back(R);
        t_vecs_.push_back(t);
        dist_coeffs_.push_back(dist_coeff);
        K_mats_.push_back(K_mat);
        P_mats_.push_back(P);



    }

    VLOG(0) << " imu data: " << settings.imu << endl;
    if(settings.imu){
        FileNode f = *fi;

        if (f.name().find("imu",0) != string::npos){
            f["rostopic"]>>imu_topic_;
            subscribeToImu();
        }
        else{
            VLOG(0)<<"ERROR: No imu topic found in .yaml file. But Imu is set to true"<<endl;
        }

    }
    fi++;

    VLOG(0) << " gps data: " << settings.gps << endl;
    if(settings.gps){

        FileNode f1 = *fi;
        if (f1.name().find("gps",0) != string::npos){

            f1["rostopic"]>>gps_topic_;


            subscribeToGPS();
        }
        else{
            VLOG(1)<<"ERROR: No gps topic found in .yml file. But gps is set to true"<<endl;
        }


    }



    img_size_ = calib_img_size_;
    num_cams_ = i;
    VLOG(0) << "number of cams = "<< num_cams_ << endl;
}


void RosDataReader::getNext(vector<cv::Mat>& imgs, double& timeStamp){

    if (settings.imu){
        loadNext(imgs, imu_msgs);
        timeStamp = tStamp.toSec();
    }
    else{
        loadNext(imgs);
        timeStamp = tStamp.toSec();

    }

}

void RosDataReader::getNext(vector<cv::Mat>& imgs, double& timeStamp, std::deque<sensor_msgs::Imu>& imu_messages_frontend){

        loadNext(imgs, imu_messages_frontend);
        timeStamp = tStamp.toSec();


}


void RosDataReader::getNext(vector<cv::Mat>& imgs, double& timeStamp, std::deque<sensor_msgs::Imu>& imu_messages_frontend, std::deque<sensor_msgs::NavSatFix>& gps_messages_frontend){

        loadNext(imgs, imu_messages_frontend, gps_messages_frontend);
        timeStamp = tStamp.toSec();

}

void RosDataReader::getNext(vector<cv::Mat>& imgs , vector<string>& segmaskImgs, double& timeStamp){
    loadNext(imgs);
    copy(ros_img_seq_nums.begin(), ros_img_seq_nums.end(), back_inserter(segmaskImgs));
    timeStamp = tStamp.toSec();

}

