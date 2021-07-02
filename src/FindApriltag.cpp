#include "FindApriltag.h"
#include <string>

using namespace std;
FindApriltag::FindApriltag(int argc,char** argv){
    mpGetopt = getopt_create();
    getopt_add_bool(mpGetopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(mpGetopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(mpGetopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(mpGetopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(mpGetopt, 't', "threads", "1", "Use this many CPU threads");
    getopt_add_double(mpGetopt, 'x', "decimate", "2.0", "Decimate input image by this factor");
    getopt_add_double(mpGetopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(mpGetopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
    if (!getopt_parse(mpGetopt, argc, argv, 1) ||
            getopt_get_bool(mpGetopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(mpGetopt);
        throw "Cannot init Getopt";
    }
    apriltag_family_t *mpApriltagFamily = NULL;
    famname = getopt_get_string(mpGetopt, "family");
    if (!strcmp(famname, "tag36h11"))
    {
        mpApriltagFamily = tag36h11_create();
    }
    else if (!strcmp(famname, "tag25h9"))
    {
        mpApriltagFamily = tag25h9_create();
    }
    else if (!strcmp(famname, "tag16h5"))
    {
        mpApriltagFamily = tag16h5_create();
    }
    else if (!strcmp(famname, "tagCircle21h7"))
    {
        mpApriltagFamily = tagCircle21h7_create();
    }
    else if (!strcmp(famname, "tagCircle49h12"))
    {
        mpApriltagFamily = tagCircle49h12_create();
    }
    else if (!strcmp(famname, "tagStandard41h12"))
    {
        mpApriltagFamily = tagStandard41h12_create();
    }
    else if (!strcmp(famname, "tagStandard52h13"))
    {
        mpApriltagFamily = tagStandard52h13_create();
    }
    else if (!strcmp(famname, "tagCustom48h12"))
    {
        mpApriltagFamily = tagCustom48h12_create();
    }
    else
    {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }
    mpApriltagDetector = apriltag_detector_create();
    apriltag_detector_add_family(mpApriltagDetector, mpApriltagFamily);
    mpApriltagDetector->quad_decimate = getopt_get_double(mpGetopt, "decimate");
    mpApriltagDetector->quad_sigma = getopt_get_double(mpGetopt, "blur");
    mpApriltagDetector->nthreads = getopt_get_int(mpGetopt, "threads");
    mpApriltagDetector->debug = getopt_get_bool(mpGetopt, "debug");
    mpApriltagDetector->refine_edges = getopt_get_bool(mpGetopt, "refine-edges");
}

FindApriltag::~FindApriltag(){
    apriltag_detector_destroy(mpApriltagDetector);    
}
void FindApriltag::Detect(cv::Mat& imgray,zarray_t*& pDetectionsArray){
    image_u8_t im = {.width = imgray.cols,
                     .height = imgray.rows,
                     .stride = imgray.cols,
                     .buf = imgray.data};
    if(pDetectionsArray!=nullptr){
        apriltag_detections_destroy(pDetectionsArray);
    }
    pDetectionsArray = apriltag_detector_detect(mpApriltagDetector, &im);
}
void FindApriltag::DrawDetections(cv::Mat& dst, zarray_t *pDetectionsArray){
    for (int i = 0; i < zarray_size(pDetectionsArray); i++) {
        apriltag_detection_t *det;
        zarray_get(pDetectionsArray, i, &det);
        line(dst, cv::Point(det->p[0][0], det->p[0][1]),
                cv::Point(det->p[1][0], det->p[1][1]),
                cv::Scalar(0, 0xff, 0), 2);
        line(dst, cv::Point(det->p[0][0], det->p[0][1]),
                cv::Point(det->p[3][0], det->p[3][1]),
                cv::Scalar(0, 0, 0xff), 2);
        line(dst, cv::Point(det->p[1][0], det->p[1][1]),
                cv::Point(det->p[2][0], det->p[2][1]),
                cv::Scalar(0xff, 0, 0), 2);
        line(dst, cv::Point(det->p[2][0], det->p[2][1]),
                cv::Point(det->p[3][0], det->p[3][1]),
                cv::Scalar(0xff, 0, 0), 2);

        string text = to_string(det->id);
        int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
        double fontscale = 1.0;
        int baseline;
        cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2,
                                        &baseline);
        putText(dst, text, cv::Point(det->c[0]-textsize.width/2,
                                det->c[1]+textsize.height/2),
                fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
        
    }
}
StereoApriltagDetecter::StereoApriltagDetecter(ZLZ_SLAM::StereoCamera::Ptr pCamera)
                                            :mDetecter(FindApriltag(0,nullptr)){
    mpCamera = pCamera;
    ReadPriotPoints(mPriorPoints);
}
TagsPos StereoApriltagDetecter::Detect(cv::Mat& leftGray,cv::Mat& rightGray,zarray_t*& leftZarray){
    mDetecter.Detect(leftGray,leftZarray);
    TagsPos leftPos;
    for (int i = 0; i < zarray_size(leftZarray); i++){
        apriltag_detection_t *det;
        zarray_get(leftZarray, i, &det);
        leftPos[det->id].pixel = ZLZ_SLAM::Pixel(det->c[0], det->c[1]);
    }
    zarray_t *rightZarray=nullptr;
    mDetecter.Detect(rightGray,rightZarray);
    TagsPos rightPos;
    for (int i = 0; i < zarray_size(rightZarray); i++){
        apriltag_detection_t *det;
        zarray_get(rightZarray, i, &det);
        rightPos[det->id].pixel = ZLZ_SLAM::Pixel(det->c[0], det->c[1]);
    }
    CalApritagDepth(leftPos, rightPos);
    return mDetections;
}
TagsPos StereoApriltagDetecter::Detect(cv::Mat& leftGray,cv::Mat& rightGray){
    zarray_t * leftZarray=nullptr;
    mDetecter.Detect(leftGray, leftZarray);
    TagsPos leftPos;
    for (int i = 0; i < zarray_size(leftZarray); i++){
        apriltag_detection_t *det;
        zarray_get(leftZarray, i, &det);
        leftPos[det->id].pixel = ZLZ_SLAM::Pixel(det->c[0], det->c[1]);
    }
    zarray_t *rightZarray=nullptr;
    mDetecter.Detect(rightGray,rightZarray);
    TagsPos rightPos;
    for (int i = 0; i < zarray_size(rightZarray); i++){
        apriltag_detection_t *det;
        zarray_get(rightZarray, i, &det);
        rightPos[det->id].pixel = ZLZ_SLAM::Pixel(det->c[0], det->c[1]);
    }
    CalApritagDepth(leftPos, rightPos);
    return mDetections;
}
void StereoApriltagDetecter::CalApritagDepth(TagsPos &leftDetections,TagsPos &rightDetections){
    for (TagsPos ::iterator it = leftDetections.begin(); it != leftDetections.end();it++){
        if(rightDetections.count(it->first)&&mPriorPoints.count(it->first)){
            double depth = mpCamera->CalDepth(it->second.pixel, rightDetections[it->first].pixel);
            if(depth <= 0){
                continue;
            }
            mDetections[it->first].pixel = it->second.pixel;
            mDetections[it->first].cameraCoor = mpCamera->Pixel2Camera(it->second.pixel, depth);
            mDetections[it->first].worldCoor = mPriorPoints[it->first];
            cout << "no." << it->first << " " << mDetections[it->first].cameraCoor.x 
                                      << ", " << mDetections[it->first].cameraCoor.y
                                      << ", " << mDetections[it->first].cameraCoor.z << endl;
        }
    }
}
void StereoApriltagDetecter::ReadPriotPoints(unordered_map<int, ZLZ_SLAM::zPoint3d>& priorPoints){
    priorPoints[0] = ZLZ_SLAM::zPoint3d(-0.0187269, 0.0235515, 0.406504);
    priorPoints[1] = ZLZ_SLAM::zPoint3d(0.0279681, 0.0250091, 0.404674);
    priorPoints[2] = ZLZ_SLAM::zPoint3d(0.0748893, 0.0264632, 0.404931);
    priorPoints[3] = ZLZ_SLAM::zPoint3d(0.122301, 0.0278866, 0.406035);
    priorPoints[4] = ZLZ_SLAM::zPoint3d(-0.0203539, 0.0703956, 0.407084);
    priorPoints[5] = ZLZ_SLAM::zPoint3d(0.0265281, 0.0719214, 0.404903);
    priorPoints[6] = ZLZ_SLAM::zPoint3d(0.0734862, 0.0735234, 0.404974);
    priorPoints[7] = ZLZ_SLAM::zPoint3d(0.120967, 0.0751284, 0.406239);
    priorPoints[8] = ZLZ_SLAM::zPoint3d(-0.0217832, 0.117352, 0.408016);
    priorPoints[9] = ZLZ_SLAM::zPoint3d(0.0250485, 0.11917, 0.406369);
    priorPoints[10] = ZLZ_SLAM::zPoint3d(0.0720398, 0.1209, 0.40554);
    priorPoints[11] = ZLZ_SLAM::zPoint3d(0.119692, 0.122749, 0.407054);
    priorPoints[12] = ZLZ_SLAM::zPoint3d(-0.023545, 0.164959, 0.409988);
    priorPoints[13] = ZLZ_SLAM::zPoint3d(0.0235046, 0.166806, 0.408037);
    priorPoints[14] = ZLZ_SLAM::zPoint3d(0.0707229, 0.1686, 0.406986);
    priorPoints[15] = ZLZ_SLAM::zPoint3d(0.118351, 0.170304, 0.407742);
}
void StereoApriltagDetecter::DrawDetection(cv::Mat& dst, zarray_t *pDetectionsArray,cv::Mat& T){
    mDetecter.DrawDetections(dst, pDetectionsArray);
    cv::copyMakeBorder(dst, dst, 0, 0, 180, 0,cv::BORDER_CONSTANT,cv::Scalar(125,125,125));
    cv::Mat t = T.col(3).rowRange(0, 3);
    cv::Mat R = T.colRange(0, 3).rowRange(0, 3);
    setprecision(2);
    double fontscale = 0.6;
    int fontface = cv::FONT_HERSHEY_COMPLEX;
    int baseline;
    string text = "X = " + to_string(t.at<double>(0, 0));
    cv::Size textsize = cv::getTextSize(text, fontface, 1, 1,&baseline);
    putText(dst, text, cv::Point(15 ,30),fontface, fontscale, cv::Scalar(0, 0, 0), 2);
    
    text = "Y = " + to_string(t.at<double>(1, 0));  
    textsize = cv::getTextSize(text, fontface, 1, 1, &baseline);
    putText(dst, text, cv::Point(15 , 60),fontface, fontscale, cv::Scalar(0, 0, 0), 2);

    text = "Z = " + to_string(t.at<double>(2, 0));
    textsize = cv::getTextSize(text, fontface, 1, 1, &baseline);
    putText(dst, text, cv::Point(15 , 90),fontface, fontscale, cv::Scalar(0, 0, 0), 2);
    
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
 
    bool singular = sy < 1e-6; // If
 
    float roll, pitch, yaw;
    if (!singular)
    {
        roll = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        pitch = atan2(-R.at<double>(2,0), sy);
        yaw = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        roll = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        pitch = atan2(-R.at<double>(2,0), sy);
        yaw = 0;
    }

    text = "Roll = " + to_string(roll);  
    textsize = cv::getTextSize(text, fontface, 1, 1, &baseline);
    putText(dst, text, cv::Point(15 , 150),fontface, fontscale, cv::Scalar(0, 0, 0), 2);

    text = "Pitch = " + to_string(pitch);  
    textsize = cv::getTextSize(text, fontface, 1, 1, &baseline);
    putText(dst, text, cv::Point(15 , 180),fontface, fontscale, cv::Scalar(0, 0, 0), 2);

    text = "Yaw = " + to_string(yaw);  
    textsize = cv::getTextSize(text, fontface, 1, 1, &baseline);
    putText(dst, text, cv::Point(15 , 210),fontface, fontscale, cv::Scalar(0, 0, 0), 2);

}