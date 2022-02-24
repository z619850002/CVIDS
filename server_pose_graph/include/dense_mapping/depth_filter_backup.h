#ifndef DEPTH_FILTER_H_
#define DEPTH_FILTER_H_

// for eigen 
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>


#include <sophus/so3.h>
#include <sophus/se3.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/version.hpp>

#include "./calc_cost.h"
class DepthFilter
{
public:
	DepthFilter(int nHeight, int nWidth);

	void Update(cv::Mat mUpdateMu, cv::Mat mUpdateCov, bool bOutlier = false);

	cv::Mat GetInvDepth(){
		cv::Mat mInvDepthMu;
		this->m_mInvDepthMu.download(mInvDepthMu);
		mInvDepthMu.convertTo(mInvDepthMu, CV_64FC1);

		return mInvDepthMu;
	}


	cv::Mat GetRatio(){
		cv::Mat mRatio;
		cv::Mat mA, mB;
		this->m_mA.download(mA);
		this->m_mB.download(mB);


		mA.convertTo(mA, CV_64FC1);
		mB.convertTo(mB, CV_64FC1);
		
		return mA / (mA + mB);
	}

	void ReleaseSpace(){
		m_mA.release();
		m_mB.release();
		m_mInvDepthMu.release();
		m_mInvDepthCov.release();
	}

	void PropogateDepth(
		cv::Mat & mLastRatio,
		cv::Mat & mLastMu,
		cv::Mat & mLastCov, 
		cv::Mat & mLastA,
		cv::Mat & mLastB,
		Eigen::Matrix3d mK,
		Sophus::SE3 mRelativePose_co);


	cv::Mat GetCov(){
		cv::Mat mCov;
		this->m_mInvDepthCov.download(mCov);
		mCov.convertTo(mCov, CV_64FC1);
		return mCov;
	}


	cv::Mat GetA(){
		cv::Mat mA;
		this->m_mA.download(mA);
		mA.convertTo(mA, CV_64FC1);
		return mA;
	}


	cv::Mat GetB(){
		cv::Mat mB;
		this->m_mB.download(mB);
		mB.convertTo(mB, CV_64FC1);
		return mB;
	}

private:
	float m_nMinInvDepth;
	float m_nMaxInvDepth;
	float m_nInvDepthRange;

	// cv::Mat m_mA;
	// cv::Mat m_mB;

	// cv::Mat m_mInvDepthMu;
	// cv::Mat m_mInvDepthCov;



	cv::cuda::GpuMat m_mA;
	cv::cuda::GpuMat m_mB;

	cv::cuda::GpuMat m_mInvDepthMu;
	cv::cuda::GpuMat m_mInvDepthCov;

	bool m_bFirst;

	
};







#endif