#include "FindApritag.h"
#include <string>

using namespace std;
FindApritag::FindApritag(int argc,char** argv){
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
    mpDetectionsArray = nullptr;
}

FindApritag::~FindApritag(){
    apriltag_detector_destroy(mpApriltagDetector);    
}
void FindApritag::Detect(cv::Mat& imgray,TagsPos& detectionsCenter){
    image_u8_t im = {.width = imgray.cols,
                     .height = imgray.rows,
                     .stride = imgray.cols,
                     .buf = imgray.data};
    if(mpDetectionsArray!=nullptr){
        apriltag_detections_destroy(mpDetectionsArray);
    }
    detectionsCenter.clear();
    mpDetectionsArray = apriltag_detector_detect(mpApriltagDetector, &im);
    for (int i = 0; i < zarray_size(mpDetectionsArray);i++){
        apriltag_detection_t *det;
        zarray_get(mpDetectionsArray, i, &det);
        detectionsCenter[det->id].pixel = ZLZ_SLAM::Pixel(det->c[0], det->c[1]);
    }
}
void FindApritag::DrawDetections(cv::Mat& dst){
    for (int i = 0; i < zarray_size(mpDetectionsArray); i++) {
        apriltag_detection_t *det;
        zarray_get(mpDetectionsArray, i, &det);
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