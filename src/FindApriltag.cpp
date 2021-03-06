#include "FindApriltag.h"
#include <string>
#include <fstream>
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
    if(pDetectionsArray == nullptr){
        return;
    }
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
StereoApriltagDetecter::StereoApriltagDetecter(ZLZ_SLAM::StereoCamera::Ptr pCamera,string priporPointsFile)
                                            :mDetecter(FindApriltag(0,nullptr)){
    mpCamera = pCamera;
    ReadPriotPoints(priporPointsFile);
}
void StereoApriltagDetecter::DetectStereo(cv::Mat& leftGray,cv::Mat& rightGray,zarray_t*& leftZarray){
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
}
void StereoApriltagDetecter::DetectMono(cv::Mat& leftGray,zarray_t*& leftZarray){
    mDetecter.Detect(leftGray,leftZarray);
    mDetections.clear();
    for (int i = 0; i < zarray_size(leftZarray); i++){
        apriltag_detection_t *det;
        zarray_get(leftZarray, i, &det);
        int id = det->id;
        mDetections[id].s = -1;
        if(mPriorPoints.count(id)){
            mDetections[id].pixel = ZLZ_SLAM::Pixel(det->c[0], det->c[1]);
            mDetections[id].s = static_cast<float>(-1);
            mDetections[id].worldCoor = mPriorPoints[id];
        }
    }
}
void StereoApriltagDetecter::CalApritagDepth(TagsPos &leftDetections,TagsPos &rightDetections){
    for (TagsPos ::iterator it = leftDetections.begin(); it != leftDetections.end();it++){
        if(rightDetections.count(it->first)&&mPriorPoints.count(it->first)){
            double depth = mpCamera->CalDepth(it->second.pixel, rightDetections[it->first].pixel);
            if(depth <= 0){
                continue;
            }
            mDetections[it->first].pixel = it->second.pixel;
            mDetections[it->first].pixelR = rightDetections[it->first].pixel;
            mDetections[it->first].s = static_cast<float>(depth);
            mDetections[it->first].worldCoor = mPriorPoints[it->first];
        }
    }
}
void StereoApriltagDetecter::ReadPriotPoints(const string& filePath){
    ifstream file;
    file.open(filePath.c_str());
    if(file.eof()){
        throw "Cannot open prior position file.";
        return;
    }
    while(!file.eof()){
        string s;
        stringstream ss;
        getline(file, s);
        ss << s;
        int tag;
        double x, y, z;
        ss >> tag;
        ss >> x;
        ss >> y;
        ss >> z;
        mPriorPoints[tag]=ZLZ_SLAM::zPoint3d(x, y, z);
    }
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
    
    double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
 
    bool singular = sy < 1e-6; // If
 
    double roll, pitch, yaw;
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
void StereoApriltagDetecter::DrawDetection(cv::Mat& dst, zarray_t *pDetectionsArray){
    mDetecter.DrawDetections(dst, pDetectionsArray);
    cv::copyMakeBorder(dst, dst, 0, 0, 180, 0,cv::BORDER_CONSTANT,cv::Scalar(125,125,125));
    setprecision(2);
    double fontscale = 0.6;
    int fontface = cv::FONT_HERSHEY_COMPLEX;
    int baseline;
    string text = "X = #.#######";
    cv::Size textsize = cv::getTextSize(text, fontface, 1, 1,&baseline);
    putText(dst, text, cv::Point(15 ,30),fontface, fontscale, cv::Scalar(0, 0, 0), 2);
    
    text = "Y = #.#######";  
    textsize = cv::getTextSize(text, fontface, 1, 1, &baseline);
    putText(dst, text, cv::Point(15 , 60),fontface, fontscale, cv::Scalar(0, 0, 0), 2);

    text = "Z = #.#######";
    textsize = cv::getTextSize(text, fontface, 1, 1, &baseline);
    putText(dst, text, cv::Point(15 , 90),fontface, fontscale, cv::Scalar(0, 0, 0), 2);
 

    text = "Roll = #.#######";  
    textsize = cv::getTextSize(text, fontface, 1, 1, &baseline);
    putText(dst, text, cv::Point(15 , 150),fontface, fontscale, cv::Scalar(0, 0, 0), 2);

    text = "Pitch = #.#######";  
    textsize = cv::getTextSize(text, fontface, 1, 1, &baseline);
    putText(dst, text, cv::Point(15 , 180),fontface, fontscale, cv::Scalar(0, 0, 0), 2);

    text = "Yaw = #.#######";  
    textsize = cv::getTextSize(text, fontface, 1, 1, &baseline);
    putText(dst, text, cv::Point(15 , 210),fontface, fontscale, cv::Scalar(0, 0, 0), 2);

}