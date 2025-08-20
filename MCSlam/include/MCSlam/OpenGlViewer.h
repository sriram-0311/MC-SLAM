//
// Created by auv on 6/15/20.
//

#ifndef LIGHT_FIELDS_ROS_OPENGLVIEWER_H
#define LIGHT_FIELDS_ROS_OPENGLVIEWER_H
#include "MCSlam/FrontEnd.h"
#include "MCSlam/Tracking.h"
#include "MCSlam/relocalization.h"
#include <pangolin/pangolin.h>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <mutex>

using namespace std;

class OpenGlViewer {
public:
    OpenGlViewer(const string &strSettingPath, FrontEnd* front_end );

    ~OpenGlViewer(){

    }
    void goLive();
    // goLiveFastTracking function taking as parameter a pointer to the frontend class
    void goLiveFastTracking(FrontEnd* front_end);
    void convertToOpenGlCameraMatrix(cv::Mat& pose, pangolin::OpenGlMatrix &M);
    void DrawCurrentCamera(pangolin::OpenGlMatrix& Twc, float r, float g, float b);
    //void DrawAllCameras(bool mono);
    void DrawAllCameras(int whichPoses, float r, float g, float b, bool lines);
    void DrawMapPoints(vector<Point3f>& mapPoints, vector<Point3f>& pointColors);
    void DrawAllPosesFromMap(std::vector<cv::Mat> posesFromMap, std::vector<Point3f> pointColors);
    void DrawNeighborPoses(vector<Mat> neighborPoses);
    void DrawAllCameraRelocalization(float r, float g, float b, bool lines, std::vector<cv::Mat> posesFromMap, std::vector<Point3f> pointColors, cv::Mat Tbc);
    void DrawMapPoints();
    void DrawFrame();
    bool CheckFinish();
    void requestFinish();
    bool isFinished();
    void setFinish();
    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;
    bool finishRequested;
    bool finished;
    mutex mMutexFinish;
    FrontEnd* frontEnd;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

};


#endif //LIGHT_FIELDS_ROS_OPENGLVIEWER_H
