
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
    
    StereoCamera::Ptr pCamera = StereoCamera::CreateStereoCamera("/home/zlz/catkin_ws/src/apriltag_test/config/mynteye.yaml");
    
    DirectLinearTransform dlt(pCamera,200);
    StereoApriltagDetecter stereoDetector(pCamera,"/home/zlz/catkin_ws/src/apriltag_test/config/PriorTagWorldPos.txt");
    for (int matIndex = 0; matIndex < 26; matIndex++){
        std::string imgPath = "/home/zlz/PhotoSave/L" + to_string(matIndex) + ".jpg";
        cv::Mat imgCVL;
        imgCVL = cv::imread(imgPath);
        if(!imgCVL.empty()){
            cv::Mat grayL;
            cv::Mat distroImgL;
            pCamera->UndistrotImage(imgCVL, distroImgL, StereoCamera::IMG_TYPE_LEFT);   
            cv::cvtColor(distroImgL, grayL,cv::COLOR_RGB2GRAY);
            zarray_t *leftZarry=nullptr;
            TagsPos detections;
            stereoDetector.DetectMono(grayL,leftZarry);
            detections = stereoDetector.mDetections;
            cv::Mat T;
            if(!dlt.CalT(detections, T)){
                continue;
            }
            cout << "T_S" << T << endl;
            dlt.PoseOptimizationOnlyPose(detections, T);
            cout << "T_D" << T << endl;
            stereoDetector.DrawDetection(distroImgL, leftZarry, T);
            cv::imshow("DistroImage", distroImgL);
            std::string imgSavePath = "/home/zlz/PhotoSave/L" + to_string(matIndex) + "result.jpg";
            cv::imwrite(imgSavePath, distroImgL);
            cv::waitKey(100);
        }
    }
    return 0;
}