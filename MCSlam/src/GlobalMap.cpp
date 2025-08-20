//
// Created by Pushyami Kaveti on 3/24/21.
//

#include "MCSlam/GlobalMap.h"
Landmark::Landmark(Mat p, MultiCameraFrame* lf_frame, int featInd, Point2f uv, int id) {
    lId = id;
    pt3D = p;
    KFs.push_back(lf_frame);
    featInds.push_back(featInd);
    uv_ref.push_back(uv);
    normal = Mat::zeros(3,1, CV_64F);
    updateNormal(lf_frame, featInd);
}

//Landmark::Landmark(Mat p, MonoFrame* frame, int featInd, Point2f uv, int id) {
//    lId = id;
//    pt3D = p.clone();
//    KFs_mono.push_back(frame);
//    featInds.push_back(featInd);
//    uv_ref.push_back(uv);
//}

void Landmark::addLfFrame(MultiCameraFrame* lf_frame, int featInd, Point2f uv){
    KFs.push_back(lf_frame);
    featInds.push_back(featInd);
    uv_ref.push_back(uv);
    updateNormal(lf_frame, featInd);
}

//void Landmark::addMonoFrame(MonoFrame* frame, int featInd, Point2f uv){
//    KFs_mono.push_back(frame);
//    featInds.push_back(featInd);
//    uv_ref.push_back(uv);
//}

void Landmark::updateNormal(MultiCameraFrame* lf_frame, int featInd){
    Mat normal = Mat::zeros(3,1, CV_64F);
    int n_rays=0;
    IntraMatch im = lf_frame->intraMatches[featInd];
    int ii=0;
    for(int featInd : im.matchIndex){
        if(featInd != -1){
            // get the position of the camera
            Mat cur_T_ref = Mat::eye(4, 4, CV_64F);
            lf_frame->camconfig_.R_mats_[ii].copyTo(cur_T_ref(cv::Range(0, 3), cv::Range(0, 3)));
            lf_frame->camconfig_.t_mats_[ii].copyTo(cur_T_ref(cv::Range(0, 3), cv::Range(3, 4)));
            Mat W_T_cur =  lf_frame->pose * cur_T_ref.inv();
            Mat c = W_T_cur.rowRange(0,3).colRange(3,4);
            Mat normal_cur = this->pt3D - c;
            normal = normal + normal_cur/cv::norm(normal_cur);
            n_rays++;
        }
        ii++;
    }
    if(this->KFs.size() != 0){
       // VLOG(2)<<"Norm of old normal: "<<cv::norm(this->normal);
        if(this->KFs.size() == 1){
            this->normal =  normal/n_rays;
            this->n_rays = n_rays;
        }
        else{
            //VLOG(2)<<"norm of temp normal: "<<cv::norm(normal);
            this->normal =  (this->normal * this->n_rays) + normal;
            this->n_rays = this->n_rays + n_rays;
            this->normal = this->normal/this->n_rays ;
        }

       // VLOG(2)<<"Norm of updated normal: "<<cv::norm(this->normal);
    }
    else{
        VLOG(2)<<"Error. KF size in landmarks cannoty be zero here";
    }
}

void Landmark::updateNormal(){
    Mat normal = Mat::zeros(3,1, CV_64F);
    int n_rays=0;
     for(int i =0; i <this->KFs.size(); i++){
         int idx = featInds[i];
         MultiCameraFrame* kf = this->KFs[i];
         IntraMatch im = kf->intraMatches[idx];
         int ii=0;
         for(int featInd : im.matchIndex){
             if(featInd != -1){
                 // get the position of the camera
                 Mat cur_T_ref = Mat::eye(4, 4, CV_64F);
                 kf->camconfig_.R_mats_[ii].copyTo(cur_T_ref(cv::Range(0, 3), cv::Range(0, 3)));
                 kf->camconfig_.t_mats_[ii].copyTo(cur_T_ref(cv::Range(0, 3), cv::Range(3, 4)));
                 Mat W_T_cur =  kf->pose * cur_T_ref.inv();
                 Mat c = W_T_cur.rowRange(0,3).colRange(3,4);
                 Mat normal_cur = this->pt3D - c;
                 normal = normal + normal_cur/cv::norm(normal_cur);
                 n_rays++;
             }
             ii++;
         }
     }
     VLOG(2)<<"lm normal:"<<this->normal;
     VLOG(2)<<"lm normal update:"<<normal/n_rays;
     Mat normal1 = normal/n_rays;
     VLOG(2)<<"Angle between: "<<normal1.dot(this->normal)/ (cv::norm(normal1) * cv::norm(this->normal)) ;
     //this->normal = normal/cv::norm(normal);
}



GlobalMap::GlobalMap() {

    num_lms = 0;
    lmID = 0;
}

int GlobalMap::insertLandmark(Mat p, MultiCameraFrame* lf_frame,  int featInd, Point2f uv) {
    Landmark* l = new Landmark(p,lf_frame, featInd, uv, lmID);
    auto res = mapPoints.insert(pair<int,Landmark*>(lmID , l));
    if(!res.second){
        // the landmark ID is already present
        cout<<"Something wrong. The landmark is already inserted"<<endl;
        return -1;
    }
    num_lms++;
    lmID++;
    return (lmID-1);
}

//int GlobalMap::insertLandmark(Mat p, MonoFrame* frame,  int featInd, Point2f uv) {
//    Landmark* l = new Landmark(p,frame, featInd, uv, lmID);
//    auto res = mapPoints.insert(pair<int,Landmark*>(lmID , l));
//    if(!res.second){
//        // the landmark ID is already present
//        cout<<"Something wrong. The landmark is already inserted"<<endl;
//        return -1;
//    }
//    num_lms++;
//    lmID++;
//    return (lmID-1);
//}

void GlobalMap::insertLandmark(Landmark* p) {

    auto res = mapPoints.insert(pair<int,Landmark*>(lmID , p));
    if(!res.second){
        // the landmark ID is already present
        return;
    }
    lmID++;
    num_lms++;
}

void GlobalMap::deleteLandmark(int lid){
    Landmark* l = getLandmark(lid);
    int i =0;
    for(auto lf : l->KFs){
        lf->lIds[l->featInds[i]] = -1;
        i++;
    }
    mapPoints.erase(lid);
    num_lms--;
}

bool GlobalMap::updateLandmark(int lid, cv::Mat &point_new, double& diff_norm){

    Landmark* landmark = getLandmark(lid);
    Mat diff_lm = landmark->pt3D - point_new;
    VLOG(4)<<"diff in lm: "<<diff_lm.at<double>(0,0)<<","<<diff_lm.at<double>(1,0)<<","<<diff_lm.at<double>(2,0)<<endl;
    diff_norm = cv::norm(diff_lm);
//    if(diff_norm > 0.5)
//    VLOG(0)<<"LiD: "<<lid<<", ("<<landmark->pt3D.at<double>(0,0)<<","<<landmark->pt3D.at<double>(1,0)<<","<<landmark->pt3D.at<double>(2,0)
//           <<")--->("<<point_new.at<double>(0,0)<<","<<point_new.at<double>(1,0)<<","<<point_new.at<double>(2,0)<<")"<<endl;
//    landmark->pt3D = point_new.clone();
    if(diff_norm < 5.0){ // todo: worth testing without this condition for gps.
//        VLOG(0)<<"Updating landmark with ID : "<<lid<<endl;
        landmark->pt3D = point_new.clone();
        return true;
    }
    else{
        //delete landmrak form the map

//        VLOG(0)<<"Deleting landmark with ID : "<<lid<<endl;
//        VLOG(0) <<"diff_norm: " << diff_norm << endl;
//        deleteLandmark(lid);
        return false;
    }
}

Landmark* GlobalMap::getLandmark(int lid){
    auto res = mapPoints.find(lid);
    if (res != mapPoints.end()) {
        return res->second;
    } else {
        VLOG(0) << "Not found\n";
        return NULL;
    }

}

void GlobalMap::printMap(){
    std::map<int, Landmark*>::iterator it;
    for(it = mapPoints.begin() ; it != mapPoints.end() ; ++it){
        Landmark* l = it->second;
        cout<<"landmark "<<it->first<<","<<l->lId << " pt: "<<l->pt3D<<" frame seen : ";
        for (auto i = l->KFs.begin(); i !=l->KFs.end(); ++i)
            cout<<(*i)->frameId<<"," ;
        cout<<endl;
    }
}