#pragma once
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

class DirectLinearTransform{
public:
    DirectLinearTransform(ZLZ_SLAM::StereoCamera::Ptr& pCamera,int nIterations);
    void CalT(TagsPos &detections,cv::Mat& T);

private:
    const int mnIterations;
    const cv::Mat mK;
    void Normalize(const TagsPos &points, TagsPos &vNormalizedPoints, cv::Mat &T, cv::Mat &T1);
    cv::Mat CalH(vector<ZLZ_SLAM::zPoint> &points);
    float CheckInlinear(TagsPos &points, unordered_map<int, bool> &inlinears, cv::Mat &H);
    void OpencvSolvePose(TagsPos &detections, cv::Mat &T);
};