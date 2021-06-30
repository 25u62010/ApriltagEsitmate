
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

void Normalize(const FindApritag::TagsPos& points, FindApritag::TagsPos& vNormalizedPoints,cv::Mat &T ,cv::Mat &T1)
{
    float meanU = 0;
    float meanV = 0;
    float meanX = 0;
    float meanY = 0;

    const int N = points.size();

    vNormalizedPoints.clear();

    for (FindApritag::TagsPos::const_iterator it = points.begin(), end = points.end(); it != end;it++)
    {
        meanU += it->second.pixel.u;
        meanV += it->second.pixel.v;
        meanX += it->second.worldCoor.x;
        meanY += it->second.worldCoor.y;
    }

    meanU = meanU/N;
    meanV = meanV/N;
    meanX = meanX/N;
    meanY = meanY/N;

    float meanDevU = 0;
    float meanDevV = 0;
    float meanDevX = 0;
    float meanDevY = 0;

    for(FindApritag::TagsPos::const_iterator it = points.begin(), end = points.end(); it != end;it++)
    {
        int tagNo = it->first;
        vNormalizedPoints[tagNo].pixel.u = it->second.pixel.u - meanU;
        vNormalizedPoints[tagNo].pixel.v = it->second.pixel.v - meanV;
        vNormalizedPoints[tagNo].worldCoor.x = it->second.worldCoor.x - meanX;
        vNormalizedPoints[tagNo].worldCoor.y = it->second.worldCoor.y - meanY;

        meanDevU += fabs(vNormalizedPoints[tagNo].pixel.u);
        meanDevV += fabs(vNormalizedPoints[tagNo].pixel.v);
        meanDevX += fabs(vNormalizedPoints[tagNo].worldCoor.x);
        meanDevY += fabs(vNormalizedPoints[tagNo].worldCoor.y);
    }
    meanDevU = meanDevU / N;
    meanDevV = meanDevV / N;
    meanDevX = meanDevX / N;
    meanDevY = meanDevY / N;
    float sU = 1.0 / meanDevU;
    float sV = 1.0 / meanDevV;
    float sX = 1.0 / meanDevX;
    float sY = 1.0 / meanDevY;
    for(FindApritag::TagsPos::iterator it = vNormalizedPoints.begin(), end = vNormalizedPoints.end(); it != end;it++)
    {
        it->second.pixel.u = it->second.pixel.u * sU;
        it->second.pixel.v = it->second.pixel.v * sV;
        it->second.worldCoor.x = it->second.worldCoor.x * sX;
        it->second.worldCoor.y = it->second.worldCoor.y * sY;
    }

    T = cv::Mat::eye(3,3,CV_64FC1);
    T.at<double>(0,0) = sU;
    T.at<double>(1,1) = sV;
    T.at<double>(0,2) = -meanU*sU;
    T.at<double>(1,2) = -meanV*sV;
    T1 = cv::Mat::eye(3,3,CV_64FC1);
    T1.at<double>(0,0) = sX;
    T1.at<double>(1,1) = sY;
    T1.at<double>(0,2) = -meanX*sX;
    T1.at<double>(1,2) = -meanY*sY;
}

void CalApritagDepth(StereoCamera::Ptr pCamera,FindApritag::TagsPos& leftDetections,
                     FindApritag::TagsPos& rightDetections,
                     unordered_map<int, ZLZ_SLAM::zPoint3d>& priorPoints){
    for (FindApritag::TagsPos::iterator it = leftDetections.begin(); it != leftDetections.end();it++){
        if(rightDetections.count(it->first)&&priorPoints.count(it->first)){
            double depth = pCamera->CalDepth(it->second.pixel, rightDetections[it->first].pixel);
            it->second.cameraCoor=pCamera->Pixel2Camera(it->second.pixel,depth);
            it->second.worldCoor = priorPoints[it->first];
            cout << "no." << it->first << " " << it->second.cameraCoor.x << ", " << it->second.cameraCoor.y << ", " << it->second.cameraCoor.z << endl;
        }
    }
}
cv::Mat CalH(vector<zPoint>& points){
    int n = points.size();
    cv::Mat D(int(n * 2), 9, CV_64FC1, cv::Scalar(0));
    double *D_ptr;
    for (int i = 0; i < n; i++){
        D_ptr = D.ptr<double>(2 * i);
        D_ptr[0] = points[i].worldCoor.x;
        D_ptr[1] = points[i].worldCoor.y;
        D_ptr[2] = 1;
        D_ptr[6] = -points[i].worldCoor.x * points[i].pixel.u;
        D_ptr[7] = -points[i].worldCoor.y * points[i].pixel.u;
        D_ptr[8] = -points[i].pixel.u;

        D_ptr = D.ptr<double>(2*i+1);
        
        D_ptr[3] = points[i].worldCoor.x;
        D_ptr[4] = points[i].worldCoor.y;
        D_ptr[5] = 1;
        D_ptr[6] = -points[i].worldCoor.x * points[i].pixel.v;
        D_ptr[7] = -points[i].worldCoor.y * points[i].pixel.v;
        D_ptr[8] = -points[i].pixel.v;
    }
    cv::Mat u, w, vt;
    cv::SVDecomp(D, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    cv::Mat H = vt.row(8).reshape(0, 3);
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
        if(u1<0||v1<0){
            continue;
        }
        double error = (u1 - point.pixel.u) * (u1 - point.pixel.u) + (v1 - point.pixel.v) * (v1 - point.pixel.v);
        if(error<5.991){
            score = 5.991 - error;
            inlinears[it->first]=true;
        }
        else{
            inlinears[it->first] = false;
        }
    }
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
    cv::Mat homongraph;
    cv::Mat kInv;
    cv::invert(pCamera->mK,kInv);

    candiateNo.reserve(detections.size());
    for (FindApritag::TagsPos::iterator it = detections.begin(); it != detections.end();it++){
        candiateNo.push_back(it->first);
    }
    cv::Mat T_UV, T_XY;
    FindApritag::TagsPos vNormalizedPoints;
    Normalize(detections, vNormalizedPoints, T_UV, T_XY);
    cv::Mat Tinv = T_UV.inv();
    unordered_map<int, bool> inlinears;
    float maxScore=0;
    for (int i = 0; i < nIterations; i++)
    {
        vector<int> candiateNoTemp = candiateNo;
        vector<zPoint> points;
        unordered_map<int, bool> inlinearsI1,inlinearsI2;
        for (int j = 0; j < 8; j++){
            int no = rng.uniform(0, candiateNoTemp.size());
            points.push_back(vNormalizedPoints[candiateNoTemp[no]]);
            candiateNoTemp[no] = candiateNoTemp.back();
            candiateNoTemp.pop_back();
        }
        cv::Mat Hi = CalH(points);
        Hi = Tinv * Hi * T_XY;
        double lamdai = 1 / cv::norm(kInv * Hi.col(0));
        Hi *= lamdai;
        float score1 = CheckInlinear(detections, inlinearsI1, Hi);
        if(score1>maxScore){
            maxScore = score1;
            inlinears = inlinearsI1;
            homongraph = Hi;
        }
        Hi = -Hi;
        float score2 = CheckInlinear(detections, inlinearsI2, Hi);
        if(score2>maxScore){
            maxScore = score2;
            inlinears = inlinearsI2;
            homongraph = Hi;
        }
    }
    
    
    double lamda = 1 / cv::norm(kInv * homongraph.col(0));
    cv::Mat r1 = lamda * kInv * homongraph.col(0);
    cv::Mat r2 = lamda * kInv * homongraph.col(1);
    cv::Mat t = lamda * kInv * homongraph.col(2);
    cv::Mat r3 = r1.cross(r2);
    cv::Mat T(4,4,CV_64FC1,cv::Scalar(0));
    cv::Mat R(3,3,CV_64FC1,cv::Scalar(0));
    r1.copyTo(R.col(0).rowRange(0,3));
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
    cout <<T << endl;
    
}
void OpencvSolvePose(FindApritag::TagsPos& detections,ZLZ_SLAM::StereoCamera::Ptr pCamera){
    vector<cv::Point2f> uv;
    vector<cv::Point3f> xyz;
    int count = 0;
    for (auto detection : detections)
    {
        float s = detection.second.cameraCoor.z;
        cv::Point2f pixeli;
        pixeli.x = detection.second.pixel.u;
        pixeli.y= detection.second.pixel.v;
        cv::Point3f worldCoorI;
        worldCoorI.x = detection.second.worldCoor.x;
        worldCoorI.y = detection.second.worldCoor.y;
        worldCoorI.z = detection.second.worldCoor.z;
        uv.push_back(pixeli);
        xyz.push_back(worldCoorI);
        count++;
        if(count==3){
            break;
        }
    }
    cv::Mat homongraph = cv::findHomography(xyz, uv);
    cv::Mat rvec0 = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat tvec0 = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat distCoffeeFolish;
    cv::Mat cam;
    cam = (cv::Mat_<double>(3,3) <<pCamera->mK.at<double>(0,0), 0., pCamera->mK.at<double>(0,2),
        0., pCamera->mK.at<double>(1,1), pCamera->mK.at<double>(1,2),
        0., 0., pCamera->mK.at<double>(2,2));
    cv::solvePnP(xyz, uv, cam, distCoffeeFolish, rvec0, tvec0, cv::SOLVEPNP_P3P);
    cv::Mat rmat0;
    cv::Rodrigues(rvec0,rmat0);
    cv::Mat T(4,4,CV_32FC1);
    rmat0.copyTo(T.rowRange(0, 3).colRange(0, 3));
    tvec0.copyTo(T.rowRange(0, 3).col(3));
    cout << "T"
            "="
         << T << endl;
}

void ReadPriotPoints(unordered_map<int, ZLZ_SLAM::zPoint3d>& priorPoints){
    priorPoints[0] = zPoint3d(-0.0187269, 0.0235515, 0.406504);
    priorPoints[1] = zPoint3d(0.0279681, 0.0250091, 0.404674);
    priorPoints[2] = zPoint3d(0.0748893, 0.0264632, 0.404931);
    priorPoints[3] = zPoint3d(0.122301, 0.0278866, 0.406035);
    priorPoints[4] = zPoint3d(-0.0203539, 0.0703956, 0.407084);
    priorPoints[5] = zPoint3d(0.0265281, 0.0719214, 0.404903);
    priorPoints[6] = zPoint3d(0.0734862, 0.0735234, 0.404974);
    priorPoints[7] = zPoint3d(0.120967, 0.0751284, 0.406239);
    priorPoints[8] = zPoint3d(-0.0217832, 0.117352, 0.408016);
    priorPoints[9] = zPoint3d(0.0250485, 0.11917, 0.406369);
    priorPoints[10] = zPoint3d(0.0720398, 0.1209, 0.40554);
    priorPoints[11] = zPoint3d(0.119692, 0.122749, 0.407054);
    priorPoints[12] = zPoint3d(-0.023545, 0.164959, 0.409988);
    priorPoints[13] = zPoint3d(0.0235046, 0.166806, 0.408037);
    priorPoints[14] = zPoint3d(0.0707229, 0.1686, 0.406986);
    priorPoints[15] = zPoint3d(0.118351, 0.170304, 0.407742);
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
    imgCVR = cv::imread("/home/zlz/PhotoSave/R0.jpg");
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
        OpencvSolvePose(detectionsL, pCamera);
    }

    cv::waitKey(0);

    return 0;
}