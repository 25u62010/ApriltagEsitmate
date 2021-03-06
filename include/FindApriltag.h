#pragma once
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCircle49h12.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>
#include <apriltag/common/getopt.h>

#include "Camera.h"
#include <unordered_map>
struct TagPos{
    float s;
    ZLZ_SLAM::Pixel pixel;
    ZLZ_SLAM::Pixel pixelR;
    ZLZ_SLAM::zPoint3d worldCoor;
    TagPos(){
	}
	TagPos(ZLZ_SLAM::Pixel _pixel):pixel(_pixel){};
};
typedef std::unordered_map<int, TagPos> TagsPos;
class FindApriltag{
public:
    FindApriltag(int argc,char** argv);
    ~FindApriltag();
    void Detect(cv::Mat& imgray,zarray_t*& pDetectionsArray);
    void DrawDetections(cv::Mat& dst, zarray_t *pDetectionsArray);
    
private:
    getopt_t * mpGetopt;
    apriltag_detector_t *mpApriltagDetector;
    const char *famname;
    apriltag_family_t *mpApriltagFamily;
    
};
class StereoApriltagDetecter{
public:
    StereoApriltagDetecter(ZLZ_SLAM::StereoCamera::Ptr pCamera,string priporPointsFile);
    void DetectStereo(cv::Mat& leftGray,cv::Mat& rightGray,zarray_t*& leftZarray);
    void DetectMono(cv::Mat& leftGray,zarray_t*& leftZarray);
    TagsPos mDetections;
    void DrawDetection(cv::Mat& dst, zarray_t *pDetectionsArray,cv::Mat& T);
    void DrawDetection(cv::Mat &dst, zarray_t *pDetectionsArray);

private:
    FindApriltag mDetecter;
    ZLZ_SLAM::StereoCamera::Ptr mpCamera;
    unordered_map<int, ZLZ_SLAM::zPoint3d> mPriorPoints;
    void CalApritagDepth(TagsPos &leftDetections, TagsPos &rightDetections);
    void ReadPriotPoints(const string& filePath);
};