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
class FindApritag{
public:
    typedef std::unordered_map<int, ZLZ_SLAM::zPoint> TagsPos;
    FindApritag(int argc,char** argv);
    ~FindApritag();
    void Detect(cv::Mat& imgray,TagsPos& detectionsCenter);
    void DrawDetections(cv::Mat& dst);
    
private:
    getopt_t * mpGetopt;
    apriltag_detector_t *mpApriltagDetector;
    const char *famname;
    apriltag_family_t *mpApriltagFamily;
    zarray_t *mpDetectionsArray;
    
};