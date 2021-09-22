#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "Camera.h"
#include "FindApriltag.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/se3_ops.h"
#include "Thirdparty/g2o/g2o/types/se3quat.h"
#include "Thirdparty/g2o/g2o/types/types_sba.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

class DirectLinearTransform {
public:
    DirectLinearTransform(ZLZ_SLAM::StereoCamera::Ptr &pCamera, int nIterations);
    bool CalT(TagsPos &detections, cv::Mat &T);
    int PoseOptimizationOnlyPose(TagsPos &detections, cv::Mat &T);

private:
    const int mnIterations;
    const cv::Mat mK;
    void Normalize(const TagsPos &points, TagsPos &vNormalizedPoints, cv::Mat &T, cv::Mat &T1);
    cv::Mat CalH(vector<TagPos> &points);
    float CheckInlinear(TagsPos &points, unordered_map<int, double> &scalars, cv::Mat &H);
    g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
    cv::Mat toCvMat(const g2o::SE3Quat &SE3);
};