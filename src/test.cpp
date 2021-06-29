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
#include "FindApritag.h"
using namespace std;
using namespace ZLZ_SLAM;
cv::Mat imgCVR;
cv::Mat imgCVL;

void CalApritagDepth(StereoCamera::Ptr pCamera,FindApritag::TagsPos& leftDetections,
                     FindApritag::TagsPos& rightDetections,
                     unordered_map<int, ZLZ_SLAM::zPoint3d>& priorPoints){
    for (FindApritag::TagsPos::iterator it = leftDetections.begin(); it != leftDetections.end();it++){
        if(rightDetections.count(it->first)&&priorPoints.count(it->first)){
            double depth = pCamera->CalDepth(it->second.pixel, rightDetections[it->first].pixel);
            it->second.cameraCoor=pCamera->Pixel2Camera(it->second.pixel,depth);
            it->second.worldCoor = priorPoints[it->first];
            //cout << "no." << it->first << " " << it->second.cameraCoor.x << " " << it->second.cameraCoor.y << " " << it->second.cameraCoor.z << endl;
        }
    }
}
cv::Mat CalH(vector<zPoint>& points,cv::Mat K){
    int n = points.size();
    cv::Mat D(int(n * 2), 9, CV_64FC1, cv::Scalar(0));
    double *D_ptr;
    for (int i = 0; i < n; i++){
        D_ptr = D.ptr<double>(2 * i);
        D_ptr[3] = -points[i].worldCoor.x;
        D_ptr[4] = -points[i].worldCoor.y;
        D_ptr[5] = -1;
        D_ptr[6] = points[i].worldCoor.x * points[i].pixel.v;
        D_ptr[7] = points[i].worldCoor.y * points[i].pixel.v;
        D_ptr[8] = points[i].pixel.v;
        D_ptr = D.ptr<double>(2*i+1);
        D_ptr[0] = points[i].worldCoor.x;
        D_ptr[1] = points[i].worldCoor.y;
        D_ptr[2] = 1;
        D_ptr[6] = -points[i].worldCoor.x*points[i].pixel.u;
        D_ptr[7] = -points[i].worldCoor.y*points[i].pixel.u;
        D_ptr[8] = -points[i].pixel.u;
    }
    cv::Mat u, w, vt;
    cv::SVDecomp(D, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    cv::Mat H = vt.row(8).reshape(0, 3);
    cout << H << endl;
    return H;
}

float CheckInlinear(FindApritag::TagsPos& points,unordered_map<int, bool>& inlinears,cv::Mat& H){
    vector<Pixel> pixels;
    float score;
    const double h11 = H.at<double>(0, 0);
    const double h12 = H.at<double>(0, 1);
    const double h13 = H.at<double>(0, 2);
    const double h21 = H.at<double>(1, 0);
    const double h22 = H.at<double>(1, 1);
    const double h23 = H.at<double>(1, 2);
    const double h31 = H.at<double>(2, 0);
    const double h32 = H.at<double>(2, 1);
    const double h33 = H.at<double>(2, 2);
    for (FindApritag::TagsPos::iterator it = points.begin(), mend = points.end(); it != mend;it++){
        zPoint point = it->second;
        double s = point.cameraCoor.z;
        double u1 = h11 * point.worldCoor.x + h12 * point.worldCoor.y + h13;
        u1 /= s;
        double v1 = h21 * point.worldCoor.x + h22 * point.worldCoor.y + h23;
        v1 /= s;
        double error = (u1 - point.pixel.u) * (u1 - point.pixel.u) + (v1 - point.pixel.v) * (v1 - point.pixel.v);
        if(error<5.991){
            score = 5.991 - error;
            inlinears[it->first]=true;
        }
        else{
            inlinears[it->first] = false;
        }
    }
    cout << score << endl;
    return score;
}
void DirectLinearTransform(FindApritag::TagsPos& detections,ZLZ_SLAM::StereoCamera::Ptr pCamera){
    if(detections.size()<8){
        cout << "The number of detections must larger than 6." << endl;
        return;
    }
    cv::RNG rng;
    const int nIterations = 200;
    vector<int> candiateNo;
    cv::Mat H;
    candiateNo.reserve(detections.size());
    for (FindApritag::TagsPos::iterator it = detections.begin(); it != detections.end();it++){
        candiateNo.push_back(it->first);
    }
    unordered_map<int, bool> inlinears;
    float maxScore=0;
    for (int i = 0; i < nIterations; i++)
    {
        vector<int> candiateNoTemp = candiateNo;
        vector<zPoint> points;
        unordered_map<int, bool> inlinearsI;
        for (int j = 0; j < 4; j++){
            int no = rng.uniform(0, candiateNoTemp.size()-1);
            points.push_back(detections[candiateNoTemp[no]]);
            candiateNoTemp[no] = candiateNoTemp.back();
            candiateNoTemp.pop_back();
        }
        cv::Mat Hi = CalH(points,pCamera->mK);
        float score = CheckInlinear(detections, inlinearsI, Hi);
        if(score>maxScore){
            maxScore = score;
            inlinears = inlinearsI;
            H = Hi;
        }
        
    }
    cv::Mat kInv;
    cv::invert(pCamera->mK,kInv);
    double lamda = 1 / cv::norm(kInv * H.col(0));
    cv::Mat r1 = lamda * kInv * H.col(0);
    cv::Mat r2 = lamda * kInv * H.col(1);
    cv::Mat t = lamda * kInv * H.col(2);
    cv::Mat r3 = r1.cross(r2);
    cv::Mat T(4,4,CV_64FC1,cv::Scalar(0));
    cv::Mat R(3,3,CV_64FC1,cv::Scalar(0));
    r1.copyTo(R.col(0).rowRange(0,3));
    cout << r1 << endl;
    r2.copyTo(R.col(1).rowRange(0,3));
    r3.copyTo(R.col(2).rowRange(0,3));
    cv::Mat w, u, vt;
    cv::SVDecomp(R, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    R = u * vt;
    R.copyTo(T.colRange(0, 3).rowRange(0, 3));
    t.copyTo(T.col(3).rowRange(0, 3));
    if(cv::determinant(R)<0){
        T = -T;
    }
    T.at < double >(3,3)= 1;
    cout << "T:" << T << endl;
}
void ReadPriotPoints(unordered_map<int, ZLZ_SLAM::zPoint3d>& priorPoints){
    priorPoints[0] = zPoint3d(0.000, 0.000, 0.000);
    priorPoints[1] = zPoint3d(0.047, 0.000, 0.000);
    priorPoints[2] = zPoint3d(0.094, 0.000, 0.000);
    priorPoints[3] = zPoint3d(0.141, 0.000, 0.000);
    priorPoints[4] = zPoint3d(0.000, 0.047, 0.000);
    priorPoints[5] = zPoint3d(0.047, 0.047, 0.000);
    priorPoints[6] = zPoint3d(0.094, 0.047, 0.000);
    priorPoints[7] = zPoint3d(0.141, 0.047, 0.000);
    priorPoints[8] = zPoint3d(0.000, 0.094, 0.000);
    priorPoints[9] = zPoint3d(0.047, 0.094, 0.000);
    priorPoints[10] = zPoint3d(0.094, 0.094, 0.000);
    priorPoints[11] = zPoint3d(0.141, 0.094, 0.000);
    priorPoints[12] = zPoint3d(0.000, 0.141, 0.000);
    priorPoints[13] = zPoint3d(0.047, 0.141, 0.000);
    priorPoints[14] = zPoint3d(0.094, 0.141, 0.000);
    priorPoints[15] = zPoint3d(0.141, 0.141, 0.000);
}
int main(int argc, char **argv)
{
    
    FindApritag findApritagL(argc, argv);
    FindApritag findApritagR(argc, argv);
    FindApritag::TagsPos detectionsL, detectionsR;
    StereoCamera::Ptr pCamera = StereoCamera::CreateStereoCamera("/home/zlz/catkin_ws/src/apriltag_test/config/mynteye.yaml");
    unordered_map<int, ZLZ_SLAM::zPoint3d> priorPoints;
    ReadPriotPoints(priorPoints);
    imgCVL = cv::imread("/home/zlz/PhotoSave/L0.jpg");
    imgCVR = cv::imread("/home/zlz/PhotoSave/L0.jpg");
    if (!imgCVL.empty())
    {
        cv::Mat distroImg;
        pCamera->UndistrotImage(imgCVL, distroImg, StereoCamera::IMG_TYPE_LEFT);
        cv::Mat gray;
        cv::cvtColor(distroImg, gray,cv::COLOR_RGB2GRAY);
        detectionsL.clear();
        findApritagL.Detect(gray, detectionsL);
        // Draw detection outlines
        findApritagL.DrawDetections(distroImg);
        
        imshow("Tag Detections", distroImg);
    }
    if(!imgCVR.empty()){
        cv::Mat distroImg;
        pCamera->UndistrotImage(imgCVR, distroImg, StereoCamera::IMG_TYPE_RIGHT);
        cv::Mat gray;
        cv::cvtColor(distroImg, gray,cv::COLOR_RGB2GRAY);
        detectionsR.clear();
        findApritagR.Detect(gray, detectionsR);
    }
    if(!imgCVL.empty()&&!imgCVR.empty()){
        CalApritagDepth(pCamera, detectionsL,detectionsR,priorPoints);
        DirectLinearTransform(detectionsL, pCamera);
    }

    cv::waitKey(0);

    return 0;
}
