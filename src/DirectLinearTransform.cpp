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

    for (TagsPos ::const_iterator it = points.begin(), end = points.end(); it != end;it++){
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

    for(TagsPos ::const_iterator it = points.begin(), end = points.end(); it != end;it++){
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

cv::Mat DirectLinearTransform::CalH(vector<TagPos>& points){
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

float DirectLinearTransform::CheckInlinear(TagsPos & points,unordered_map<int, double>& scalars,cv::Mat& H){
    vector<Pixel> pixels;
    float score=0;
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
        TagPos point = it->second;
        double s= h31 * point.worldCoor.x + h32 * point.worldCoor.y + h33;
        double u1 = h11 * point.worldCoor.x + h12 * point.worldCoor.y + h13;
        u1 /= s;
        double v1 = h21 * point.worldCoor.x + h22 * point.worldCoor.y + h23;
        v1 /= s;
        double error = (u1 - point.pixel.u) * (u1 - point.pixel.u) + (v1 - point.pixel.v) * (v1 - point.pixel.v);
        if( error < 5.991 ){
            score += 5.991 - error;
            scalars[it->first] = s;
        }
    }
    return score;
}
bool DirectLinearTransform::CalT(TagsPos  &detections,cv::Mat& T){
    if(detections.size()<8){
        cout << "The number of detections must larger than 6." << endl;
        return false;
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
    unordered_map<int, double> scalar;
    float maxScore=0;
    for (int i = 0; i < mnIterations; i++){
        vector<int> candiateNoTemp = candiateNo;
        vector<TagPos> points;
        unordered_map<int, double> scalars1,scalars2;
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
        float score1 = CheckInlinear(detections, scalars1, Hi);
        if(score1>maxScore){
            maxScore = score1;
            scalar = scalars1;
            homongraph = Hi;
        }
        Hi = -Hi;
        float score2 = CheckInlinear(detections, scalars2, Hi);
        if(score2>maxScore){
            maxScore = score2;
            scalar = scalars2;
            homongraph = Hi;
        }
    }
    for(TagsPos ::iterator it = detections.begin(), mend = detections.end(); it != mend;it++){
        int tagIndex = it->first;
        if (scalar.find(tagIndex)!=scalar.end()){
            it->second.s = scalar[tagIndex];
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
    
    if( t.at<double>(2,0)< 0 ){
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
    return true;
}
void DirectLinearTransform::OpencvSolvePose(TagsPos & detections,cv::Mat& T){
    vector<cv::Point2f> uv;
    vector<cv::Point3f> xyz;
    int count = 0;
    for (auto detection : detections){
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
g2o::SE3Quat DirectLinearTransform::toSE3Quat(const cv::Mat &cvT)
{
    Eigen::Matrix<double,3,3> R;
    R << cvT.at<double>(0,0), cvT.at<double>(0,1), cvT.at<double>(0,2),
         cvT.at<double>(1,0), cvT.at<double>(1,1), cvT.at<double>(1,2),
         cvT.at<double>(2,0), cvT.at<double>(2,1), cvT.at<double>(2,2);

    Eigen::Matrix<double,3,1> t(cvT.at<double>(0,3), cvT.at<double>(1,3), cvT.at<double>(2,3));
    return g2o::SE3Quat(R, t);
}
cv::Mat DirectLinearTransform::toCvMat(const g2o::SE3Quat &SE3){
    Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
    cv::Mat cvMat(4, 4, CV_64F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<double>(i,j)=eigMat(i,j);
    return cvMat.clone();
}
int DirectLinearTransform::PoseOptimizationOnlyPose(TagsPos &detections, cv::Mat &T){
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    // Set Frame vertex
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(toSE3Quat(T));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);
    const float delta= sqrt(5.996);
    vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdges;
    for (TagsPos::iterator it = detections.begin(), itEnd = detections.end(); it != itEnd; it++){
        Eigen::Matrix<double,2,1> obs;
        obs << it->second.pixel.u, it->second.pixel.v;
        g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e->setMeasurement(obs);
        e->setInformation(Eigen::Matrix2d::Identity());
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(delta); 
        e->fx = mK.at<double>(0, 0);
        e->fy = mK.at<double>(1, 1);
        e->cx = mK.at<double>(0, 2);
        e->cy = mK.at<double>(1, 2);
        e->Xw[0] = static_cast<double>(it->second.worldCoor.x) ;
        e->Xw[1] = static_cast<double>(it->second.worldCoor.y) ;
        e->Xw[2] = static_cast<double>(it->second.worldCoor.z) ;
        optimizer.addEdge(e);
        vpEdges.push_back(e);
    }
    int nBad = 0;
    const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
    for(size_t i=0; i<4; i++){
        vSE3->setEstimate(toSE3Quat(T));
        optimizer.initializeOptimization(0);
        optimizer.optimize(20);
        for(size_t edgeIndex=0, iend=vpEdges.size(); edgeIndex<iend; edgeIndex++){
            g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdges[edgeIndex];
            const float chi2 = e->chi2();
            if(chi2>chi2Mono[i]){               
                e->setLevel(1);                 
                nBad++;
            }
            else{
                e->setLevel(0);
            }
            if(i==2)
                e->setRobustKernel(0); 
        }
    }
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    T = toCvMat(SE3quat_recov);
    return nBad;
}
int DirectLinearTransform::PoseOptimization(TagsPos &detections, cv::Mat &T){
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    // Set Frame vertex
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(toSE3Quat(T));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);
    const float delta= sqrt(5.996);
    vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdges;
    for (TagsPos::iterator it = detections.begin(), itEnd = detections.end(); it != itEnd; it++){
        Eigen::Matrix<double,2,1> obs;
        obs << it->second.pixel.u, it->second.pixel.v;
        g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e->setMeasurement(obs);
        e->setInformation(Eigen::Matrix2d::Identity());
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(delta); 
        e->fx = mK.at<double>(0, 0);
        e->fy = mK.at<double>(1, 1);
        e->cx = mK.at<double>(0, 2);
        e->cy = mK.at<double>(1, 2);
        e->Xw[0] = static_cast<double>(it->second.worldCoor.x) ;
        e->Xw[1] = static_cast<double>(it->second.worldCoor.y) ;
        e->Xw[2] = static_cast<double>(it->second.worldCoor.z) ;
        optimizer.addEdge(e);
        vpEdges.push_back(e);
    }
    int nBad = 0;
    const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
    for(size_t i=0; i<4; i++){
        vSE3->setEstimate(toSE3Quat(T));
        optimizer.initializeOptimization(0);
        optimizer.optimize(20);
        for(size_t edgeIndex=0, iend=vpEdges.size(); edgeIndex<iend; edgeIndex++){
            g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdges[edgeIndex];
            const float chi2 = e->chi2();
            if(chi2>chi2Mono[i]){               
                e->setLevel(1);                 
                nBad++;
            }
            else{
                e->setLevel(0);
            }
            if(i==2)
                e->setRobustKernel(0); 
        }
    }
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    T = toCvMat(SE3quat_recov);
    return nBad;
}