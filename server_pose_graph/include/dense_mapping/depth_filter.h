#ifndef DEPTH_FILTER_H_
#define DEPTH_FILTER_H_

// for eigen 
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>


#include <sophus/so3.h>
#include <sophus/se3.h>


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/cudaarithm.hpp>

class DepthFilter
{
public:
	DepthFilter(int nHeight, int nWidth);

	void Update(cv::Mat mUpdateMu, cv::Mat mUpdateCov, bool bOutlier = false);


	void Update(cv::Mat mUpdateMu, cv::Mat mUpdateCov, Sophus::SE3 & mPose,  bool bOutlier = false);

	void PropogateDepth(
		cv::Mat & mLastRatio,
		cv::Mat & mLastMu,
		cv::Mat & mLastCov, 
		cv::Mat & mLastA,
		cv::Mat & mLastB,
		Eigen::Matrix3d mK,
		Sophus::SE3 mRelativePose_co);

	void FreeSpace(){

		this->m_mA.release();
		this->m_mB.release();

		this->m_mInvDepthMu.release();
		this->m_mInvDepthCov.release();

	}

	void ReleaseSpace(){
		m_mA.release();
		m_mB.release();
		m_mInvDepthMu.release();
		m_mInvDepthCov.release();
	}


	cv::Mat GetInvDepth(){
		return this->m_mInvDepthMu;
	}


	cv::Mat GetRatio(){
		return this->m_mA / (this->m_mA + this->m_mB);
	}

	cv::Mat GetCov(){
		return this->m_mInvDepthCov;
	}


	cv::Mat GetA(){
		return this->m_mA;
	}


	cv::Mat GetB(){
		return this->m_mB;
	}

private:
	double m_nMinInvDepth;
	double m_nMaxInvDepth;
	double m_nInvDepthRange;

	cv::Mat m_mA;
	cv::Mat m_mB;

	cv::Mat m_mInvDepthMu;
	cv::Mat m_mInvDepthCov;


	
};







#endif