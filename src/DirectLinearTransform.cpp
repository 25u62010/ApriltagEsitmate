#include "DirectLinearTransform.h"

using namespace std;
using namespace ZLZ_SLAM;
DirectLinearTransform::DirectLinearTransform(StereoCamera::Ptr& pCamera,int nIterations)
                                   :mnIterations(nIterations),mK(pCamera->mK){
    
}


void DirectLinearTransform::Normalize(const TagsPos  &points, TagsPos  &vNormalizedPoints, cv::Mat &T, cv::Mat &T1)
{
    float meanU = 0;
    float meanV = 0;
    float meanX = 0;
    float meanY = 0;

    const int N = points.size();

    vNormalizedPoints.clear();

    for (TagsPos ::const_iterator it = points.begin(), end = points.end(); it != end;it++)
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

    for(TagsPos ::const_iterator it = points.begin(), end = points.end(); it != end;it++)
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
    for(TagsPos ::iterator it = vNormalizedPoints.begin(), end = vNormalizedPoints.end(); it != end;it++)
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

cv::Mat DirectLinearTransform::CalH(vector<zPoint>& points){
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

float DirectLinearTransform::CheckInlinear(TagsPos & points,unordered_map<int, bool>& inlinears,cv::Mat& H){
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
    for (TagsPos ::iterator it = points.begin(), mend = points.end(); it != mend;it++){
        zPoint point = it->second;
        double s = point.cameraCoor.z;
        double u1 = h11 * point.worldCoor.x + h12 * point.worldCoor.y + h13;
        u1 /= s;
        double v1 = h21 * point.worldCoor.x + h22 * point.worldCoor.y + h23;
        v1 /= s;
        double error = (u1 - point.pixel.u) * (u1 - point.pixel.u) + (v1 - point.pixel.v) * (v1 - point.pixel.v);
        if( error < 5.991 ){
            score += 5.991 - error;
            inlinears[it->first] = true;
        }
        else{
            inlinears[it->first] = false;
        }
    }
    return score;
}
void DirectLinearTransform::CalT(TagsPos  &detections,cv::Mat& T){
    if(detections.size()<8){
        cout << "The number of detections must larger than 6." << endl;
        return;
    }
    cv::RNG rng;
    vector<int> candiateNo;
    cv::Mat homongraph;
    cv::Mat kInv;
    cv::invert(mK,kInv);

    candiateNo.reserve(detections.size());
    for (TagsPos ::iterator it = detections.begin(); it != detections.end();it++){
        candiateNo.push_back(it->first);
    }
    cv::Mat T_UV, T_XY;
    TagsPos  vNormalizedPoints;
    Normalize(detections, vNormalizedPoints, T_UV, T_XY);
    cv::Mat Tinv = T_UV.inv();
    unordered_map<int, bool> inlinears;
    float maxScore=0;
    for (int i = 0; i < mnIterations; i++)
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
    
    cv::Mat r1 = kInv * homongraph.col(0);
    cv::Mat r2 = kInv * homongraph.col(1);
    cv::Mat t =  kInv * homongraph.col(2);
    cv::Mat r3 = r1.cross(r2);
    T=cv::Mat(4,4,CV_64FC1,cv::Scalar(0));
    cv::Mat R(3,3,CV_64FC1,cv::Scalar(0));
    r1.copyTo(R.col(0).rowRange(0,3));
    r2.copyTo(R.col(1).rowRange(0,3));
    r3.copyTo(R.col(2).rowRange(0,3));
    cv::Mat w, u, vt;
    cv::SVDecomp(R, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    R = u * vt;
    
    if( t.at<double>(2,0)<0 ){
        cv::Mat Rz_pi_2 = (cv::Mat_<double>(3,3) << -1, 0, 0, 0, -1, 0, 0, 0, 1);
        t = -t;
        R = Rz_pi_2 * R;
    }
    R.copyTo(T.colRange(0, 3).rowRange(0, 3));
    t.copyTo(T.col(3).rowRange(0, 3));
    if(cv::determinant(R)<0){
        T = -T;
    }
    T.at < double >(3,3)= 1;
}
void DirectLinearTransform::OpencvSolvePose(TagsPos & detections,cv::Mat& T){
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
    cam = (cv::Mat_<double>(3,3) <<mK.at<double>(0,0), 0., mK.at<double>(0,2),
        0., mK.at<double>(1,1), mK.at<double>(1,2),
        0., 0., mK.at<double>(2,2));
    cv::solvePnP(xyz, uv, cam, distCoffeeFolish, rvec0, tvec0, cv::SOLVEPNP_P3P);
    cv::Mat rmat0;
    cv::Rodrigues(rvec0,rmat0);
    T=cv::Mat(4,4,CV_32FC1);
    rmat0.copyTo(T.rowRange(0, 3).colRange(0, 3));
    tvec0.copyTo(T.rowRange(0, 3).col(3));
    T.at < double >(3,3)= 1;
}