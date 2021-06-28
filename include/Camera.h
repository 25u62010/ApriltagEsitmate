#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace std;

namespace ZLZ_SLAM {

struct Pixel {
	float u;
	float v;
	Pixel() {
		
	}
	Pixel(const Pixel& Other){
		u = Other.u;
		v = Other.v;
	}
	Pixel(float _u,float _v) 
	:u(_u),v(_v){

	}
	Pixel operator=(const Pixel& Other){
		u = Other.u;
		v = Other.v;
		return *this;
	}
};
struct zPoint3d{
	float x;
	float y;
	float z;
	zPoint3d(float _x,float _y,float _z)
	:x(_x),y(_y),z(_z){

	}
	zPoint3d() {
	}
	zPoint3d(const Eigen::Vector3f& other) {
		x = other(0);
		y = other(1);
		z = other(2);
	}
	zPoint3d operator*(float a) {
		return zPoint3d(a*x,a*y,a*z);
	}
	zPoint3d& operator=(const zPoint3d& other) {
		x = other.x;
		y = other.y;
		z = other.z;
		return *this;
	}
	zPoint3d& operator=(const Eigen::Vector3f& other) {
		x = other(0);
		y = other(1);
		z = other(2);
		return *this;
	}
};
struct zPoint{
	Pixel pixel;
	zPoint3d cameraCoor;
	zPoint3d worldCoor;
	zPoint(){
	}
	zPoint(Pixel _pixel):pixel(_pixel){};
};
class MonoCamera {
public:
	
	typedef shared_ptr<MonoCamera> Ptr;
	static MonoCamera::Ptr CreateCamera(string configFilePath);
	MonoCamera(float fx, float fy, float cx, float cy, int imgWidth, int imgHeight,cv::Mat& distroMat1, cv::Mat& distroMat2);
	MonoCamera(cv::Mat& k, int imgWidth, int imgHeight,cv::Mat& distroMat1, cv::Mat& distroMat2);
	Pixel Camera2Pixel(const cv::Mat& cameraCoor);
	Pixel Camera2Pixel(const zPoint3d& cameraCoor);
	zPoint3d Pixel2Camera(const Pixel& pixelCoor,float depth=1);
	static zPoint3d Transformation(const cv::Mat& worldCoor, const cv::Mat& R, const cv::Mat& t);
	static zPoint3d Transformation(const zPoint3d& worldCoor, const cv::Mat& R, const cv::Mat& t);
	static Eigen::Vector3f Transformation(const Eigen::Vector3f& worldCoor, const Eigen::Matrix3f& R, const Eigen::Vector3f& t);
	void UndistrotImage(const cv::Mat& src,cv::Mat& dst);
	cv::Mat mK;
private:
	const float mFx;
	const float mFy;
	const float mCx;
	const float mCy;
	const int mImgWidth;
	const int mImgHeight;
	
	cv::Mat mDistroMat1;
	cv::Mat mDistroMat2;
};
class StereoCamera {
public:
	enum ImgType{
		IMG_TYPE_LEFT=0,
		IMG_TYPE_RIGHT
	};
	StereoCamera(MonoCamera::Ptr camerLeft,MonoCamera::Ptr rightLeft,double baseLine);
	void SparseStereoMatch(const cv::Mat& left, const cv::Mat& right, cv::Mat& depth);
	typedef shared_ptr<StereoCamera> Ptr;
	static StereoCamera::Ptr CreateStereoCamera(string configFilePath);
	void UndistrotImage(const cv::Mat& src,cv::Mat& dst,ImgType eflag);
	double CalDepth(Pixel leftPoint,Pixel rightPoint);
	zPoint3d Pixel2Camera(const Pixel& pixelCoor,float depth=1);
	cv::Mat mK;

private:
	MonoCamera::Ptr mpCameraLeft;
	MonoCamera::Ptr mpCameraRight;
	const double mb;
};
}