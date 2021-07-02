#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
#include <string>
#include <thread>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include "Camera.h"
#include "FindApriltag.h"
#include "DirectLinearTransform.h"
using namespace std;
using namespace ZLZ_SLAM;
cv::Mat imgCVR;
cv::Mat imgCVL;
void GrabStereo(const sensor_msgs::ImageConstPtr& msgsLeft,const sensor_msgs::ImageConstPtr& msgsRight){
    cv_bridge::CvImageConstPtr cvPtrL;
    cvPtrL=cv_bridge::toCvCopy(msgsLeft);
    imgCVL=cvPtrL->image;
    cv_bridge::CvImageConstPtr cvPtrR;
    cvPtrR=cv_bridge::toCvCopy(msgsRight);
    imgCVR=cvPtrR->image;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_test_node");
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> leftSub(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> rightSub(nh, "/camera/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), leftSub,rightSub);
    sync.registerCallback(boost::bind(GrabStereo,_1,_2));
    ros::Rate r(100);
    StereoCamera::Ptr pCamera = StereoCamera::CreateStereoCamera("/home/zlz/catkin_ws/src/apriltag_test/config/mynteye.yaml");
    DirectLinearTransform dlt(pCamera,200);
    StereoApriltagDetecter stereoDetector(pCamera);
    cv::Mat T;
    cv::Mat grayL,grayR; 
    cv::Mat distroImg;
    while (ros::ok())
    {
        if(!imgCVL.empty()){
            
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
            zarray_t * leftZarray=nullptr;
            TagsPos  detections;
            detections=stereoDetector.Detect(grayL, grayR,leftZarray);
            dlt.CalT(detections,T);
            stereoDetector.DrawDetection(distroImg, leftZarray, T);
            cv::imshow("DistroImage", distroImg);
            
        }
        cv::waitKey(20);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
