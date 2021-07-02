
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "Camera.h"
#include "FindApriltag.h"
#include "DirectLinearTransform.h"
using namespace std;
using namespace ZLZ_SLAM;
int main(int argc, char **argv)
{
    cv::Mat imgCVR;
    cv::Mat imgCVL;
    StereoCamera::Ptr pCamera = StereoCamera::CreateStereoCamera("/home/zlz/catkin_ws/src/apriltag_test/config/mynteye.yaml");
    unordered_map<int, ZLZ_SLAM::zPoint3d> priorPoints;
    imgCVL = cv::imread("/home/zlz/PhotoSave/L0.jpg");
    imgCVR = cv::imread("/home/zlz/PhotoSave/R0.jpg");
    DirectLinearTransform dlt(pCamera,200);
    StereoApriltagDetecter stereoDetector(pCamera);
    cv::Mat T;
    cv::Mat grayR,grayL;
    cv::Mat distroImg;
    if (!imgCVL.empty())
    {
        
        pCamera->UndistrotImage(imgCVL, distroImg, StereoCamera::IMG_TYPE_LEFT);
        
        cv::cvtColor(distroImg, grayL,cv::COLOR_RGB2GRAY);
    }
    if(!imgCVR.empty()){
        cv::Mat distroImgR;
        pCamera->UndistrotImage(imgCVR, distroImgR, StereoCamera::IMG_TYPE_RIGHT);
        cv::Mat gray;
        cv::cvtColor(distroImgR, grayR,cv::COLOR_RGB2GRAY);
    }
    if(!imgCVL.empty()&&!imgCVR.empty()){
        zarray_t *leftZarry=nullptr;
        TagsPos detections;
        detections=stereoDetector.Detect(grayL, grayR,leftZarry);
        dlt.CalT(detections, T);
        stereoDetector.DrawDetection(distroImg, leftZarry,T);
        cv::imshow("DistroImage", distroImg);
        cout << "T" << T << endl;
    }
    
    cv::waitKey(0);

    return 0;
}