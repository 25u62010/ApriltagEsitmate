/*
cv::Mat CalH(vector<zPoint>& points){
    int n = points.size();
    cv::Mat D(int(n * 2), 9, CV_64FC1, cv::Scalar(0));
    for (int i = 0; i < n; i++){
        double* D_ptr = D.ptr<double>(2*i);
        D_ptr[3] = points[i].worldCoor.x * points[i].cameraCoor.z;
        D_ptr[4] = points[i].worldCoor.y * points[i].cameraCoor.z;
        D_ptr[5] = points[i].cameraCoor.z;
        D_ptr[6] = -points[i].worldCoor.x * points[i].cameraCoor.y;
        D_ptr[7] = -points[i].worldCoor.y * points[i].cameraCoor.y;
        D_ptr[8] = -points[i].cameraCoor.y;
        D_ptr = D.ptr<double>(2*i+1);
        D_ptr[0] = points[i].worldCoor.x * points[i].cameraCoor.z;
        D_ptr[1] = points[i].worldCoor.y * points[i].cameraCoor.z;
        D_ptr[2] = points[i].cameraCoor.z;
        D_ptr[6] = -points[i].cameraCoor.x * points[i].worldCoor.x;
        D_ptr[7] = -points[i].cameraCoor.x * points[i].worldCoor.y;
        D_ptr[8] = -points[i].cameraCoor.x;
    }
    
    cv::Mat u, w, vt;
    cv::SVDecomp(D, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    cv::Mat H = vt.row(8).reshape(0, 3);
    cv::Mat r1 = H.col(0);
    cv::Mat r2 = H.col(1);
    cv::Mat r3 = r1.cross(r2);
    cv::Mat T(3,4,CV_64FC1);
    r1.copyTo(T.col(0));
    r2.copyTo(T.col(1));
    r3.copyTo(T.col(2));
    T.col(2).copyTo(T.col(3));
    double determinatR = cv::determinant(T.rowRange(0, 3).colRange(0, 3));
    T /= determinatR;
    cout << "T:" << T << endl;
    return cv::Mat{};
}
cv::Mat CalT(vector<zPoint>& points, cv::Mat& K){
    int n = points.size();
    cv::Mat D(int(n * 2), 12, CV_64FC1, cv::Scalar(0));
    for (int i = 0; i < n; i++){
        double* D_ptr = D.ptr<double>(2*i);
        D_ptr[4] = -(points[i].worldCoor.x)*points[i].cameraCoor.z;
        D_ptr[5] = -(points[i].worldCoor.y)*points[i].cameraCoor.z;
        D_ptr[6] = -(points[i].worldCoor.z)*points[i].cameraCoor.z;
        D_ptr[7] = -points[i].cameraCoor.z;
        D_ptr[8] = points[i].worldCoor.x*points[i].cameraCoor.y;
        D_ptr[9] = points[i].worldCoor.y*points[i].cameraCoor.y;
        D_ptr[10] = points[i].worldCoor.z*points[i].cameraCoor.y;
        D_ptr[11] = points[i].cameraCoor.y;
        D_ptr = D.ptr<double>(2*i+1);
        D_ptr[0] = (points[i].worldCoor.x)*points[i].cameraCoor.z;
        D_ptr[1] = (points[i].worldCoor.y)*points[i].cameraCoor.z;
        D_ptr[2] = (points[i].worldCoor.z)*points[i].cameraCoor.z;
        D_ptr[3] = points[i].cameraCoor.z;
        D_ptr[8] = -points[i].cameraCoor.x*points[i].worldCoor.x;
        D_ptr[9] = -points[i].cameraCoor.x*points[i].worldCoor.y;
        D_ptr[10] = -points[i].cameraCoor.x*points[i].worldCoor.z;
        D_ptr[11] = -points[i].cameraCoor.x;
    }
    
    cv::Mat u, w, vt;
    cv::SVDecomp(D, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    cv::Mat T = vt.row(11).reshape(0, 3);
    double determinatR = cv::determinant(T.rowRange(0, 3).colRange(0, 3));
    
    T /= determinatR;
    cout << "determinatR:" << determinatR << endl;
    return cv::Mat{};
}*/
