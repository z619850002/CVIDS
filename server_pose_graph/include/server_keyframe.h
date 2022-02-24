#ifndef SERVER_KEYFRAME_H_
#define SERVER_KEYFRAME_H_

#include <vector>

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "parameters.h"
#include "server_brief_extractor.h"
#include "server_camera.h"
#include "./utility/server_utility.h"
//Sophus

#include "sophus/so3.h"
#include "sophus/se3.h"

#include <mutex>
#include "./dense_mapping/depth_estimator.h"

#include <std_msgs/Header.h>

#define MIN_LOOP_NUM 15


#define BIG_LOOP_NUM 15

using namespace std;



class ServerKeyFrame
{
public:
	//Constructor.
	ServerKeyFrame(	double nTimeStamp, int nClient, int nLocalIndex, 
					Sophus::SE3 mVIOPose, 
					Eigen::Matrix3d mRotation_ic,
					Eigen::Vector3d mTranslation_ic, 
					cv::Mat &mImage,
			 		vector<cv::Point3f> & gPoints3D, vector<cv::Point2f> &gPoints2D, 
			 		vector<int> & gPointsID,
			 		ServerCamera * pCamera);

	//TODO: Construct without image.
	ServerKeyFrame(	double nTimeStamp, int nClient, int nLocalIndex, 
					Sophus::SE3 mVIOPose, 
					Eigen::Matrix3d mRotation_ic,
					Eigen::Vector3d mTranslation_ic, 
			 		vector<cv::Point3f> & gWindowPoints3D, 
			 		vector<cv::Point2f> & gWindowPoints2D, 
			 		vector<int> & gWindowPointsID,
			 		vector<DVision::BRIEF::bitset> & gWindowDescriptors,
			 		vector<cv::Point2f> & gProjectedPoints2D, 
			 		vector<DVision::BRIEF::bitset> & gProjectedDescriptors,
			 		bool bFromVisionSLAM,
			 		ServerCamera * pCamera);


	//These 2 functions maybe useless now.
	//Compute the brief descriptors of points from the vio.
	void ComputeWindowBRIEFPoint();
	//Compute the brief descriptors on all points on the image.
	void ComputeBRIEFPoint();	

	//Find matches.
	bool SearchInArea(		const DVision::BRIEF::bitset iWindowDescriptor,
                            const vector<DVision::BRIEF::bitset> & gOldDescriptors,
                            const vector<cv::Point2f> & gOldPoints,
                            const vector<cv::Point2f> & gNormOldPoints,
                            cv::Point2f & iBestMatchedPoint,
                            cv::Point2f & iBestMatchedNormPoint);

	//Compute the hamming distance between 2 descriptors.
	int HammingDistance(const DVision::BRIEF::bitset & iDescriptorA, const DVision::BRIEF::bitset & iDescriptorB);

	//Find matches with the gKeyPointsOld with the brief descriptors.
	//Called in findConnection.
	void SearchByBRIEFDes(		const vector<DVision::BRIEF::bitset> & gOldDescriptors,
                                const vector<cv::Point2f> & gOldPoints,
                                const vector<cv::Point2f> & gOldNormPoints,
                                vector<uchar> & gStatus,
								vector<cv::Point2f> & gMatchedPoints2D,
								vector<cv::Point2f> & gNormMatchedPoints2D);


	//Remove outliers by epipolar constraints.
	void FundmantalMatrixRANSAC(	const vector<cv::Point2f> & gCurrentNormPoints,
                                    const vector<cv::Point2f> & gOldNormPoints,
                                    vector<uchar> & gStatus);

	//Remove outliers by PnP
	void PnPRANSAC(		const vector<cv::Point2f> & gOldNormPoints2D,
                        const vector<cv::Point3f> & gMatchedPoints3D,
                        vector<uchar> & gStatus,
                        Eigen::Vector3d & mOldTranslation_wi, 
                        Eigen::Matrix3d & mOldRotation_wi);

	bool FindConnection(ServerKeyFrame* pOldKeyFrame, bool bAlign = false);

	void GetVIOPose(Eigen::Vector3d & mTranslation_wi , Eigen::Matrix3d & mRotation_wi);
	void GetPose(Eigen::Vector3d & mTranslation_wi, Eigen::Matrix3d & mRotation_wi);
	void GetCameraPose(Eigen::Vector3d & mTranslation_wc, Eigen::Matrix3d & mRotation_wc);


	void UpdatePose(const Eigen::Vector3d & mTranslation_wi, const Eigen::Matrix3d & mRotation_wi);
	void UpdateVIOPose(const Eigen::Vector3d & mTranslation_wi, const Eigen::Matrix3d & mRotation_wi);

	void ShiftPose(const Eigen::Vector3d & mRelativeT, const Eigen::Matrix3d & mRelativeR);


	Eigen::Vector3d GetLoopRelativeT();
	Eigen::Quaterniond GetLoopRelativeQ();


	Eigen::Vector3d GetLoopRelativeT(int nIndex);
	Eigen::Quaterniond GetLoopRelativeQ(int nIndex);

	double GetLoopRelativeYaw();
	double GetLoopRelativeYaw(int nIndex);

	void UpdateLoop(Eigen::Matrix<double, 8, 1 > & mLoopInfo);

	//Initialize depth estimation
	void InitializeDepthEstimator();
	void PropogateDepthFilter(ServerKeyFrame * pPreviousKeyFrame);
	void FuseFrame(ServerKeyFrame * pMatchedKeyFrame, bool bFuse = true);
	void FinalizeDepthMap();

	void LoadDepthMap(cv::Mat & mDepthMap){
		this->m_mDepthMutex.lock();
		mDepthMap = 1/this->m_mInvDepthMap;
		this->m_mDepthMutex.unlock();
	}


	//The info published to open chisel.
	void LoadRefInfo(		Sophus::SE3 & mRefPose_wc,
                            std_msgs::Header & iHeader,
                            cv::Mat & mDepthMap,
                            cv::Mat & mColorMap,
                            cv::Mat & mK,
                            int & nWidth, 
                            int & nHeight);

	bool FreeSpace();

	void UndistortImage();



	Eigen::Matrix4d DeterminePropChainCov(ServerKeyFrame * pRefFrame){
		
		vector<ServerKeyFrame *> gPropChain;
		if (pRefFrame->m_nGlobalIndex <= this->m_nGlobalIndex){
			gPropChain = this->DeterminePropChain(pRefFrame);
			if (gPropChain.size()  == 0){
				Eigen::Matrix4d mCov;
				mCov.setZero();
				mCov(0 , 0) = 0.0001;
				mCov(1 , 1) = 0.0001;
				mCov(2 , 2) = 0.0001;
				mCov(3 , 3) = 0.0001;
				return mCov;
			}
			gPropChain.push_back(this);
		}else{
			if (gPropChain.size()  == 0){
				Eigen::Matrix4d mCov;
				mCov.setZero();
				mCov(0 , 0) = 0.0001;
				mCov(1 , 1) = 0.0001;
				mCov(2 , 2) = 0.0001;
				mCov(3 , 3) = 0.0001;
				return mCov;
			}
			gPropChain = pRefFrame->DeterminePropChain(this);
			gPropChain.push_back(pRefFrame);
			reverse(gPropChain.begin(),gPropChain.end());
		}



		Eigen::Matrix4d mCov;
		mCov.setZero();


		if (gPropChain.size()<=1){
			return mCov;
		}


		ServerKeyFrame * pComputeRefFrame = gPropChain[0];
		ServerKeyFrame * pComputeCurrentFrame = gPropChain[1];

		for (int i=1;i<gPropChain.size();i++){
			mCov = gPropChain[i]->ComputeCovFromPreviousFrame(mCov);
			// if (i<gPropChain.size()-1){
			// 	pComputeCurrentFrame = gPropChain[i+1];
			// 	pComputeRefFrame = gPropChain[i];
			// }
		}

		return mCov;

	}


	vector<ServerKeyFrame *> DeterminePropChain(
		ServerKeyFrame * pRefFrame){
		
		vector<ServerKeyFrame *> gChain;
		if (pRefFrame->m_nClientID != this->m_nClientID){
			return gChain;
		}


		int nMaxIndex = (this->m_nGlobalIndex - pRefFrame->m_nGlobalIndex+1);
		gChain.reserve(nMaxIndex);

		gChain.push_back(pRefFrame);
		ServerKeyFrame * pChainFrame = pRefFrame->m_pNextKeyFrame;
		if (pChainFrame == NULL){
			return gChain;
		}
		while (pChainFrame->m_nGlobalIndex < this->m_nGlobalIndex){
				// int nNum = 0;
				// while (nNum <5){
				// 	pChainFrame = pChainFrame->m_pNextKeyFrame;
				// 	nNum++;
				// 	if (pChainFrame->m_nGlobalIndex >= this->m_nGlobalIndex){
				// 		return gChain;
				// 	}
				// }
				gChain.push_back(pChainFrame);
				pChainFrame = pChainFrame->m_pNextKeyFrame;
				if (pChainFrame == NULL){
					gChain.clear();
					return gChain;		
				}
		}
		if (pChainFrame->m_nGlobalIndex != this->m_nGlobalIndex){
			gChain.clear();
		}

		return gChain;
		
	}


	Eigen::Matrix4d m_mJacobianF, m_mJacobianG;

	void GenerateFG(){
		ServerKeyFrame * pRefFrame = this->m_pPreviousKeyFrame;
		if (pRefFrame == NULL){
			return;
		}

		Eigen::Matrix3d mRefRotation_wi;
		Eigen::Vector3d mRefTranslation_wi;
		pRefFrame->GetVIOPose(mRefTranslation_wi, mRefRotation_wi);

		Eigen::Matrix3d mCurrentRotation_wi;
		Eigen::Vector3d mCurrentTranslation_wi;
		this->GetVIOPose(mCurrentTranslation_wi, mCurrentRotation_wi);

		Eigen::Vector3d mEulerRef = ServerUtility::R2ypr(mRefRotation_wi);
		Eigen::Vector3d mEulerCurrent = ServerUtility::R2ypr(mCurrentRotation_wi);

		Eigen::Matrix3d mRotation_cr = mCurrentRotation_wi.inverse() * mRefRotation_wi;

		

		double nRefY = mEulerRef(0) /180.0 * M_PI;
		double nRefP = mEulerRef(1) /180.0 * M_PI;
		double nRefR = mEulerRef(2) /180.0 * M_PI;

		double nCurrentY = mEulerCurrent(0) /180.0 * M_PI;
		double nCurrentP = mEulerCurrent(1) /180.0 * M_PI;
		double nCurrentR = mEulerCurrent(2) /180.0 * M_PI;


		Eigen::Matrix3d mRefRotationY, mRefRotationX, mCurrentRotationY, mCurrentRotationX;

		mRefRotationY << cos(nRefP), 0.0, sin(nRefP),
							0.0,     1.0, 0.0,
						-sin(nRefP), 0.0, cos(nRefP);

		mRefRotationX << 1.0,  0.0,   0.0,
						 0.0,  cos(nRefR), -sin(nRefR),
						 0.0,  sin(nRefR), cos(nRefR);

		mCurrentRotationY << cos(nCurrentP), 0.0, sin(nCurrentP),
							0.0,     1.0, 0.0,
						-sin(nCurrentP), 0.0, cos(nCurrentP);

		mCurrentRotationX << 1.0,  0.0,   0.0,
						 0.0,  cos(nCurrentR), -sin(nCurrentR),
						 0.0,  sin(nCurrentR), cos(nCurrentR);




		Eigen::Matrix3d mCurrentXY = mCurrentRotationX.transpose() * mCurrentRotationY.transpose();
		Eigen::Matrix3d mRefXY = mRefRotationX.transpose() * mRefRotationY.transpose();

		double a11 = mCurrentXY(0 , 0);
		double a12 = mCurrentXY(0 , 1);
		double a13 = mCurrentXY(0 , 2);
		double a21 = mCurrentXY(1 , 0);
		double a22 = mCurrentXY(1 , 1);
		double a23 = mCurrentXY(1 , 2);
		double a31 = mCurrentXY(2 , 0);
		double a32 = mCurrentXY(2 , 1);
		double a33 = mCurrentXY(2 , 2);


		double b11 = mRefXY(0 , 0);
		double b12 = mRefXY(0 , 1);
		double b13 = mRefXY(0 , 2);
		double b21 = mRefXY(1 , 0);
		double b22 = mRefXY(1 , 1);
		double b23 = mRefXY(1 , 2);
		double b31 = mRefXY(2 , 0);
		double b32 = mRefXY(2 , 1);
		double b33 = mRefXY(2 , 2);


		double nRelativeYaw_cr = mEulerRef(0) -mEulerCurrent(0);
		
		
		double t1 = mRefTranslation_wi(0);
		double t2 = mRefTranslation_wi(1);
		double t3 = mRefTranslation_wi(2);

		Eigen::Vector3d mC;
		Eigen::Vector3d mC2 = ServerUtility::ComputeJacobian(
				mCurrentXY,
        		mRefXY,
        		mRefTranslation_wi,
        		nRelativeYaw_cr);

		nRelativeYaw_cr = nRelativeYaw_cr /180.0 * M_PI;

		for (int j=0;j<3;j++){
			mC(j) = 0.0;
			for (int i=0;i<3;i++){
				mC(j)+= -(mCurrentXY(j,0) * mRefXY(i , 0) + mCurrentXY(j,1) * mRefXY(i , 1)) * mRefTranslation_wi(i) * sin(nRelativeYaw_cr);
				mC(j) += (mCurrentXY(j,1) * mRefXY(i , 0) - mCurrentXY(j,0) * mRefXY(i , 1)) * mRefTranslation_wi(i) * cos(nRelativeYaw_cr);
			}
		}

		// mC = mC * 180.0 / M_PI;

		//Propogate the covariance.

		Eigen::Matrix4d mF, mG;
		mF.setZero();
		mG.setZero();

		mF(0 , 0) = 1.0;
		mF.block<3 , 3>(1 , 1) = mRotation_cr;

		mG(0 , 0) = 1.0;
		mG.block<3 , 3>(1 , 1) = Eigen::Matrix3d::Identity();
		mG.block<3 , 1>(1 , 0) = mC;

		this->m_mJacobianF = mF;
		this->m_mJacobianG = mG;


	}

	Eigen::Matrix4d ComputeCovFromPreviousFrame(Eigen::Matrix4d mRefCov){

		double nObserveYawCov = 1;
		double nObserveTranslationCov = 0.1;

		Eigen::Matrix4d mObservationCov;
		mObservationCov.setZero();
		mObservationCov(0 , 0) = nObserveYawCov;
		mObservationCov(1 , 1) = nObserveTranslationCov;
		mObservationCov(2 , 2) = nObserveTranslationCov;
		mObservationCov(3 , 3) = nObserveTranslationCov;

		Eigen::Matrix4d mCurrentCov = this->m_mJacobianF * mRefCov * this->m_mJacobianF.transpose() + this->m_mJacobianG * mObservationCov * this->m_mJacobianG.transpose();
		return mCurrentCov;
	}


	Eigen::Matrix4d ComputeCov(ServerKeyFrame * pRefFrame, Eigen::Matrix4d mRefCov){

		double nObserveYawCov = 1;
		double nObserveTranslationCov = 0.1;

		Eigen::Matrix3d mRefRotation_wi;
		Eigen::Vector3d mRefTranslation_wi;
		pRefFrame->GetVIOPose(mRefTranslation_wi, mRefRotation_wi);

		Eigen::Matrix3d mCurrentRotation_wi;
		Eigen::Vector3d mCurrentTranslation_wi;
		this->GetVIOPose(mCurrentTranslation_wi, mCurrentRotation_wi);

		Eigen::Vector3d mEulerRef = ServerUtility::R2ypr(mRefRotation_wi);
		Eigen::Vector3d mEulerCurrent = ServerUtility::R2ypr(mCurrentRotation_wi);

		Eigen::Matrix3d mRotation_cr = mCurrentRotation_wi.inverse() * mRefRotation_wi;

		

		double nRefY = mEulerRef(0) /180.0 * M_PI;
		double nRefP = mEulerRef(1) /180.0 * M_PI;
		double nRefR = mEulerRef(2) /180.0 * M_PI;

		double nCurrentY = mEulerCurrent(0) /180.0 * M_PI;
		double nCurrentP = mEulerCurrent(1) /180.0 * M_PI;
		double nCurrentR = mEulerCurrent(2) /180.0 * M_PI;


		Eigen::Matrix3d mRefRotationY, mRefRotationX, mCurrentRotationY, mCurrentRotationX;

		mRefRotationY << cos(nRefP), 0.0, sin(nRefP),
							0.0,     1.0, 0.0,
						-sin(nRefP), 0.0, cos(nRefP);

		mRefRotationX << 1.0,  0.0,   0.0,
						 0.0,  cos(nRefR), -sin(nRefR),
						 0.0,  sin(nRefR), cos(nRefR);

		mCurrentRotationY << cos(nCurrentP), 0.0, sin(nCurrentP),
							0.0,     1.0, 0.0,
						-sin(nCurrentP), 0.0, cos(nCurrentP);

		mCurrentRotationX << 1.0,  0.0,   0.0,
						 0.0,  cos(nCurrentR), -sin(nCurrentR),
						 0.0,  sin(nCurrentR), cos(nCurrentR);




		Eigen::Matrix3d mCurrentXY = mCurrentRotationX.transpose() * mCurrentRotationY.transpose();
		Eigen::Matrix3d mRefXY = mRefRotationX.transpose() * mRefRotationY.transpose();

		double a11 = mCurrentXY(0 , 0);
		double a12 = mCurrentXY(0 , 1);
		double a13 = mCurrentXY(0 , 2);
		double a21 = mCurrentXY(1 , 0);
		double a22 = mCurrentXY(1 , 1);
		double a23 = mCurrentXY(1 , 2);
		double a31 = mCurrentXY(2 , 0);
		double a32 = mCurrentXY(2 , 1);
		double a33 = mCurrentXY(2 , 2);


		double b11 = mRefXY(0 , 0);
		double b12 = mRefXY(0 , 1);
		double b13 = mRefXY(0 , 2);
		double b21 = mRefXY(1 , 0);
		double b22 = mRefXY(1 , 1);
		double b23 = mRefXY(1 , 2);
		double b31 = mRefXY(2 , 0);
		double b32 = mRefXY(2 , 1);
		double b33 = mRefXY(2 , 2);


		double nRelativeYaw_cr = mEulerRef(0) -mEulerCurrent(0);
		
		
		double t1 = mRefTranslation_wi(0);
		double t2 = mRefTranslation_wi(1);
		double t3 = mRefTranslation_wi(2);

		Eigen::Vector3d mC;
		Eigen::Vector3d mC2 = ServerUtility::ComputeJacobian(
				mCurrentXY,
        		mRefXY,
        		mRefTranslation_wi,
        		nRelativeYaw_cr);

		nRelativeYaw_cr = nRelativeYaw_cr /180.0 * M_PI;

		for (int j=0;j<3;j++){
			mC(j) = 0.0;
			for (int i=0;i<3;i++){
				mC(j)+= -(mCurrentXY(j,0) * mRefXY(i , 0) + mCurrentXY(j,1) * mRefXY(i , 1)) * mRefTranslation_wi(i) * sin(nRelativeYaw_cr);
				mC(j) += (mCurrentXY(j,1) * mRefXY(i , 0) - mCurrentXY(j,0) * mRefXY(i , 1)) * mRefTranslation_wi(i) * cos(nRelativeYaw_cr);
			}
		}

		// mC = mC * 180.0 / M_PI;

		//Propogate the covariance.

		Eigen::Matrix4d mF, mG;
		mF.setZero();
		mG.setZero();

		mF(0 , 0) = 1.0;
		mF.block<3 , 3>(1 , 1) = mRotation_cr;

		mG(0 , 0) = 1.0;
		mG.block<3 , 3>(1 , 1) = Eigen::Matrix3d::Identity();
		mG.block<3 , 1>(1 , 0) = mC;


		Eigen::Matrix4d mObservationCov;
		mObservationCov.setZero();
		mObservationCov(0 , 0) = nObserveYawCov;
		mObservationCov(1 , 1) = nObserveTranslationCov;
		mObservationCov(2 , 2) = nObserveTranslationCov;
		mObservationCov(3 , 3) = nObserveTranslationCov;


		Eigen::Matrix4d mCurrentCov = mF * mRefCov * mF.transpose() + mG * mObservationCov * mG.transpose();
		return mCurrentCov;
		// cout << "mC is: " << endl << mC << endl;
		// cout << "mC2 is: " << endl << mC2 << endl;
		// cout << "Current cov is: " << endl << this->m_mVIOPoseCov << endl;
		// cout << "F is: " << endl << mF << endl;
		// cout << "G is: " << endl << mG << endl;

		// double nC1 = -(a11 * b11 + a12 * b12)* t1 * sin(nRelativeYaw_cr) 
		// 			 +(a12 * b11 - a11 * b12)* t1 * cos(nRelativeYaw_cr);
		// nC1 +=       -(a11 * b21 + a12 * b22)* t2 * sin(nRelativeYaw_cr) 
		// 	  		 +(a12 * b21 - a11 * b22)* t2 * cos(nRelativeYaw_cr);
		// nC1 +=       -(a11 * b31 + a12 * b32)* t3 * sin(nRelativeYaw_cr) 
		// 	  		 +(a12 * b31 - a11 * b32)* t3 * cos(nRelativeYaw_cr);
		
		
		// double nC2 = -(a21 * b11 + a22 * b12)* t1 * sin(nRelativeYaw_cr) 
		// 			 +(a22 * b11 - a21 * b12)* t1 * cos(nRelativeYaw_cr);
		// nC2 +=       -(a21 * b21 + a22 * b22)* t2 * sin(nRelativeYaw_cr) 
		// 	  		 +(a22 * b21 - a21 * b22)* t2 * cos(nRelativeYaw_cr);
		// nC2 +=       -(a21 * b31 + a22 * b32)* t3 * sin(nRelativeYaw_cr) 
		// 	  		 +(a22 * b31 - a21 * b32)* t3 * cos(nRelativeYaw_cr);
		
		
		// double nC3 = -(a31 * b11 + a32 * b12)* t1 * sin(nRelativeYaw_cr) 
		// 			 +(a32 * b11 - a31 * b12)* t1 * cos(nRelativeYaw_cr);
		// nC3 +=       -(a31 * b21 + a32 * b22)* t2 * sin(nRelativeYaw_cr) 
		// 	  		 +(a32 * b21 - a31 * b22)* t2 * cos(nRelativeYaw_cr);
		// nC3 +=       -(a31 * b31 + a32 * b32)* t3 * sin(nRelativeYaw_cr) 
		// 	  		 +(a32 * b31 - a31 * b32)* t3 * cos(nRelativeYaw_cr);
		
		// Eigen::Vector3d mC2(nC1, nC2, nC3);

		// cout << endl;
		// cout << "Test" << endl;
		// for (int i=0;i<10;i++){
		// 	cout << "mC  is: " << mC.transpose() << endl;
		// 	cout << "mC2 is: " << mC2.transpose() << endl;
		// }
		// cout << endl;

	}


public:
	bool m_bFreeSpace;
	//Is this keyframe from a vision slam system.
	bool m_bFromVisionSLAM;

	//Identifier of this keyframe.
	double m_nTimeStamp;                            //double time_stamp; 
	int m_nLocalIndex;								//int local_index;
	int m_nClientID;
	int m_nGlobalIndex;

	//Extrinsics
	Eigen::Matrix3d m_mRotation_ic;
	Eigen::Vector3d m_mTranslation_ic;


	
	//Pose offered by the VIO of the client.
	//Translation and rotation here is as the inverse of the pose.
	Sophus::SE3 m_mLocalPose_iw;
	//Translation.
	Eigen::Vector3d m_mLocalTranslation_wi;  			//Eigen::Vector3d vio_T_w_i; 
	//Rotation.
	Eigen::Matrix3d m_mLocalRotation_wi;				//Eigen::Matrix3d vio_R_w_i; 
	
	//Global Pose.
	//Translation and rotation here is as the inverse of the pose.
	Sophus::SE3 m_mGlobalPose_iw;
	Eigen::Vector3d m_mGlobalTranslation_wi;			//Eigen::Vector3d T_w_i;
	Eigen::Matrix3d m_mGlobalRotation_wi;				//Eigen::Matrix3d R_w_i;
	
	//Used for backup?
	//Translation and rotation here is as the inverse of the pose.
	Eigen::Vector3d m_mLocalT_Backup_wi;				//Eigen::Vector3d origin_vio_T;		
	Eigen::Matrix3d m_mLocalR_Backup_wi;				//Eigen::Matrix3d origin_vio_R;

	//Image of this frame.
	cv::Mat m_mImage;								//cv::Mat image;
	cv::Mat m_mUndistortedImage;
	cv::Mat m_mColoredImage;
 	//Compressed image.
	cv::Mat m_mThumbnailImage;						//cv::Mat thumbnail;

	//Points from the vio
	//These are all mappoints offered by VIO.
	vector<cv::Point3f> m_gPoints3D;				//vector<cv::Point3f> point_3d; 
	vector<cv::Point2f> m_gPoints2D;				//vector<cv::Point2f> point_2d_uv; vector<cv::Point2f> point_2d_norm;
	vector<cv::Point2f> m_gNormalizedWindowPoints;
	//The ID of the mappoints.
	vector<int> m_gPointsID;						//vector<double> point_id;
	vector<DVision::BRIEF::bitset> m_gWindowBriefDescriptors;	// vector<BRIEF::bitset> window_brief_descriptors;



	//These are all points extracted additionally.
	//All Points.
	//Used to be matched by others.
	vector<cv::Point2f> m_gProjectedPoints;				// vector<cv::KeyPoint> keypoints;
	vector<cv::Point2f> m_gNormalizedProjectedPoints;
	//Points from the vio.
	vector<DVision::BRIEF::bitset> m_gBriefDescriptors;			// vector<BRIEF::bitset> brief_descriptors;
	

	// vector<cv::KeyPoint> keypoints_norm;
	
	// bool has_fast_point;
	// int sequence;
	bool m_bHasLoop;								//bool has_loop;
	bool m_bHasBeenAligned;
    bool m_bHasBeenDisturbed;

	int m_nLoopIndex;								//int loop_index;
	cv::Mat m_mMatchedImage;	
	Eigen::Matrix<double, 8 , 1> m_mLoopInfo;		// Eigen::Matrix<double, 8, 1 > loop_info;


	vector<int> m_gLoopIndices;
	vector<Eigen::Matrix<double, 8 , 1>> m_gLoopInfos;


	ServerCamera * m_pServerCamera;

	ServerKeyFrame * m_pPreviousKeyFrame;
	ServerKeyFrame * m_pNextKeyFrame;


	//Used in dense mapping
	DepthEstimator * m_pEstimator;
	bool m_bDepthRefFrame;
	vector<ServerKeyFrame *> m_gMatchedKeyFrames;
	ServerKeyFrame * m_pRefKeyFrame;

	cv::Mat m_mInvDepthMap;
	cv::Mat m_mInvCovMap;
	cv::Mat m_mConvergeMap;
	bool m_bFinalizeDepthMap;
	//The map points are of the camera coordinates.
	vector<cv::Point3d> m_gMapPoints;
	//The color of these map points.
	//BGR
	vector<cv::Point3d> m_gMapPointsColor;




	std_msgs::Header m_iHeader;
	
	std::mutex m_mDepthMutex;
	std::mutex m_mPoseMutex;				
};

#endif

