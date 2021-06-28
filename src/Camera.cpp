#include "Camera.h"
using namespace ZLZ_SLAM;

MonoCamera::MonoCamera(float fx, float fy, float cx, float cy, int imgWidth, int imgHeight, cv::Mat& distroMat1, cv::Mat& distroMat2)
	:mFx(fx), mFy(fy) , mCx(cx) , mCy(cy),mImgHeight(imgHeight),mImgWidth(imgWidth),mK(3,3,CV_32F,cv::Scalar(0)) {
	mK.at<float>(0, 0) = fx;
	mK.at<float>(1, 1) = fy;
	mK.at<float>(0, 2) = cx;
	mK.at<float>(1, 2) = cy;
	mK.at<float>(2, 2) = 1.0;

	distroMat1.copyTo(mDistroMat1);
	distroMat2.copyTo(mDistroMat2);
}
MonoCamera::MonoCamera(cv::Mat& k, int imgWidth, int imgHeight,cv::Mat& distroMat1, cv::Mat& distroMat2)
	:mFx (k.at<double>(0, 0)),mFy (k.at<double>(1, 1)),mCx (k.at<double>(0, 2)),mCy(k.at<double>(1, 2))
	,mImgHeight(imgHeight),mImgWidth(imgWidth){
	k.copyTo(mK);
	distroMat1.copyTo(mDistroMat1);
	distroMat2.copyTo(mDistroMat2);
}
Pixel MonoCamera::Camera2Pixel(const cv::Mat& cameraCoor) {
	Pixel pixelCoor;
	float invZ = 1 / cameraCoor.at<float>(2);
	pixelCoor.u = mFx * cameraCoor.at<float>(0)*invZ + mCx;
	pixelCoor.v = mFx * cameraCoor.at<float>(1)*invZ + mCy;
	return pixelCoor;
}
Pixel MonoCamera::Camera2Pixel(const zPoint3d& cameraCoor) {
	Pixel pixelCoor;
	float invZ = 1 / cameraCoor.z;
	pixelCoor.u = mFx * cameraCoor.x*invZ + mCx;
	pixelCoor.v = mFy * cameraCoor.y*invZ + mCy;
	return pixelCoor;
}
zPoint3d MonoCamera::Pixel2Camera(const Pixel& pixelCoor,float depth) {
	zPoint3d cameraCoor(0,0,1);
	cameraCoor.x = (pixelCoor.u - mCx) / mFx;
	cameraCoor.y = (pixelCoor.v - mCy) / mFy;
	return cameraCoor*depth;
}
zPoint3d MonoCamera::Transformation(const cv::Mat& worldCoor,const cv::Mat& R, const cv::Mat& t) {
	assert(worldCoor.rows == 3 && worldCoor.cols == 1);
	assert(R.rows == 3 && R.cols == 3);
	assert(t.rows == 3 && t.cols == 1);
	cv::Mat camerCoor =R*worldCoor+t;
	return zPoint3d(camerCoor.at<float>(0), camerCoor.at<float>(1), camerCoor.at<float>(2));
}
zPoint3d MonoCamera::Transformation(const zPoint3d& worldCoor,const cv::Mat& R, const cv::Mat& t) {
	assert(R.rows == 3 && R.cols == 3);
	assert(t.rows == 3 && t.cols == 1);
	zPoint3d camerCoor;
	camerCoor.x = R.at<float>(0, 0)*worldCoor.x + R.at<float>(0, 1)*worldCoor.y + R.at<float>(0, 2)*worldCoor.z;
	camerCoor.y = R.at<float>(1, 0)*worldCoor.x + R.at<float>(1, 1)*worldCoor.y + R.at<float>(2, 2)*worldCoor.z;
	camerCoor.z = R.at<float>(2, 0)*worldCoor.x + R.at<float>(2, 1)*worldCoor.y + R.at<float>(2, 2)*worldCoor.z;
	return camerCoor;
}
Eigen::Vector3f MonoCamera::Transformation(const Eigen::Vector3f& worldCoor, const Eigen::Matrix3f& R, const Eigen::Vector3f& t) {
	return R * worldCoor + t;
}
MonoCamera::Ptr MonoCamera::CreateCamera(string configFilePath) {
	cv::FileStorage configFile;
	float fx, fy, cx, cy;
	int imageWidth, imageHeight;
	cv::Mat D;
	try {
		configFile.open(configFilePath, cv::FileStorage::READ);
		configFile["camera.left"]["D"] >> D;
		fx = configFile["camera.left"]["fx"];
		fy = configFile["camera.left"]["fy"];
		cx = configFile["camera.left"]["cx"];
		cy = configFile["camera.left"]["cy"];
		imageWidth = configFile["camera.left"]["width"];
		imageHeight = configFile["camera.left"]["height"];
	}
	catch(cv::Exception ex){
		cout << ex.what()<<endl;
		exit(0);
	}
	if (!configFile.isOpened()) {
		return nullptr;
	}
	
	cv::Mat M1, M2;
	cv::Mat K(3, 3, CV_32F, cv::Scalar(0));
	K.at<float>(0, 0) = fx;
	K.at<float>(1, 1) = fy;
	K.at<float>(0, 2) = cx;
	K.at<float>(1, 2) = cy;
	K.at<float>(2, 2) = 1;
	
	cv::initUndistortRectifyMap(K, D, cv::Mat::eye(cv::Size(3, 3), CV_32F), K, cv::Size(imageWidth,imageHeight), CV_32F, M1, M2);

	return MonoCamera::Ptr(new MonoCamera(fx, fy, cx, cy, imageWidth,imageHeight,M1,M2));
}
void MonoCamera::UndistrotImage(const cv::Mat& src, cv::Mat& dst) {
	cv::remap(src, dst, mDistroMat1, mDistroMat2, cv::INTER_LINEAR);
}
StereoCamera::StereoCamera(MonoCamera::Ptr cameraLeft, MonoCamera::Ptr camerRight, double baseLine)
	:mb(baseLine){
	mpCameraLeft = cameraLeft;
	mpCameraRight = camerRight;
	mK = mpCameraLeft->mK;
}
void StereoCamera::SparseStereoMatch(const cv::Mat& left, const cv::Mat& right, cv::Mat& depth) {
	int rows = left.rows;
	int cols = left.cols;
	for (int row = 0; row < rows; row++) {
		const uchar* leftPixel = left.ptr<uchar>(0);

	}
}
StereoCamera::Ptr StereoCamera::CreateStereoCamera(string configFilePath){
	cv::FileStorage fSettings;
	
	fSettings.open(configFilePath, cv::FileStorage::READ);
	cv::FileNode node;
	if(!fSettings.isOpened()){
		throw "ERROR: Wrong path to settings";
	}
	cv::Mat k_L, k_R, p_L, p_R, R_L, R_R, D_L, D_R;

	node=fSettings["LEFT.K"];
	if(node.empty()){
		throw "LEFT.K parameter dosen't exist!";
	}
	k_L = node.mat();
	
	node=fSettings["RIGHT.K"];
	if(node.empty()){
		throw "RIGHT.K parameter dosen't exist!";
	}
	k_R = node.mat();

	node=fSettings["LEFT.P"];
	if(node.empty()){
		throw "LEFT.P parameter dosen't exist!";
	}
	p_L = node.mat();

	node=fSettings["RIGHT.P"];
	if(node.empty()){
		throw "RIGHT.P parameter dosen't exist!";
	}
	p_R = node.mat();

	node=fSettings["LEFT.D"];
	if(node.empty()){
		throw "LEFT.D parameter dosen't exist!";
	}
	D_L = node.mat();

	node=fSettings["RIGHT.D"];
	if(node.empty()){
		throw "RIGHT.D parameter dosen't exist!";
	}
	D_R = node.mat();

	node=fSettings["LEFT.R"];
	if(node.empty()){
		throw "LEFT.R parameter dosen't exist!";
	}
	R_L = node.mat();

	node=fSettings["RIGHT.R"];
	if(node.empty()){
		throw "RIGHT.R parameter dosen't exist!";
	}
	R_R = node.mat();

	int rows_l, rows_r, cols_l, cols_r;
	
	node = fSettings["LEFT.height"];
	if(node.empty()||!node.isInt()){
		throw "LEFT.height parameter dosen't exist or is not a real number!";
	}
	rows_l = node.operator int();

	node = fSettings["LEFT.width"];
	if(node.empty()||!node.isInt()){
		throw "LEFT.width parameter dosen't exist or is not a real number!";
	}
	cols_l = node.operator int();
	
	node = fSettings["RIGHT.height"];
	if(node.empty()||!node.isInt()){
		throw "RIGHT.height parameter dosen't exist or is not a real number!";
	}
	rows_r = node.operator int();

	node = fSettings["RIGHT.width"];
	if(node.empty()||!node.isInt()){
		throw "RIGHT.width parameter dosen't exist or is not a real number!";
	}
	cols_r = node.operator int();

	cv::Mat M1L, M2L, M1R, M2R;
	cv::Mat newCamerMatrixL = p_L.rowRange(0, 3).colRange(0, 3);
	cv::initUndistortRectifyMap(k_L, D_L, R_L, newCamerMatrixL, cv::Size(cols_l, rows_l), CV_32F, M1L, M2L);
	shared_ptr<MonoCamera> cameraLeft(new MonoCamera(newCamerMatrixL,cols_l,rows_l,M1L,M2L));
	cv::Mat newCamerMatrixR = p_R.rowRange(0, 3).colRange(0, 3);
	cv::initUndistortRectifyMap(k_R, D_R, R_R, newCamerMatrixR, cv::Size(cols_r, rows_r), CV_32F, M1R, M2R);
	shared_ptr<MonoCamera> cameraRight(new MonoCamera(newCamerMatrixR,cols_r,rows_r,M1R,M2R));
	double baseLine;
	node = fSettings["Camera.bf"];
	if(node.empty()||!node.isReal()){
		throw "RIGHT.width parameter dosen't exist or is not a real number!";
	}
	baseLine = node.real();
			
	return shared_ptr<StereoCamera>(new StereoCamera(cameraLeft, cameraRight, baseLine));
}
void StereoCamera::UndistrotImage(const cv::Mat& src,cv::Mat& dst,ImgType flag){
	if(flag==IMG_TYPE_LEFT){
		mpCameraLeft->UndistrotImage(src, dst);
	}
	else{
		mpCameraRight->UndistrotImage(src, dst);
	}
}
double StereoCamera::CalDepth(Pixel leftPoint,Pixel rightPoint){
	double d =sqrt((leftPoint.u - rightPoint.u)*(leftPoint.u - rightPoint.u)
			      +(leftPoint.v - rightPoint.v)*(leftPoint.v - rightPoint.v));
	return mb / d;
}
zPoint3d StereoCamera::Pixel2Camera(const Pixel& pixelCoor,float depth){
	return mpCameraLeft->Pixel2Camera(pixelCoor,depth); 
}