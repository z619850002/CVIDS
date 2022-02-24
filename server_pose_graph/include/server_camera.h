#ifndef SERVER_CAMERA_H_
#define SERVER_CAMERA_H_


//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

//Std
#include <iostream>
#include <string>
#include <vector>
#include <map>

using namespace std;


class ServerCamera
{
public:
	ServerCamera();
	
	Eigen::Matrix3d GetK();
	Eigen::Matrix<double, 4, 1> GetD();

	void SetK(Eigen::Matrix3d mK);
	void SetD(Eigen::Matrix<double, 4, 1> mD);

	//Convert the pixel coordinate to the normalized camera coordinate.
	Eigen::Vector3d LiftProject(Eigen::Vector2d mPixelPoint);
	Eigen::Vector2d Project(Eigen::Vector3d mNormalizedPoint);

private:
	Eigen::Matrix3d m_mK;
	//k1, k2, p1, p2
	Eigen::Matrix<double, 4, 1> m_mD;

	void Distortion(Eigen::Vector2d& mUndistortedPoint, Eigen::Vector2d& mDistortedPoint);
	
};

inline Eigen::Matrix3d ServerCamera::GetK(){
	return this->m_mK;
}

inline Eigen::Matrix<double, 4, 1> ServerCamera::GetD(){
	return this->m_mD;
}

inline void ServerCamera::SetK(Eigen::Matrix3d mK){
	this->m_mK = mK;
}

inline void ServerCamera::SetD(Eigen::Matrix<double, 4, 1> mD){
	this->m_mD = mD;
}


#endif