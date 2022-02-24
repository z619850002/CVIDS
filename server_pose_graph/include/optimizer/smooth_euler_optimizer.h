#pragma once


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
#include <list>
#include <map>
#include <queue>
#include <set>
#include <iterator>

using namespace std;

typedef Eigen::Matrix<double, 6 ,1> Vector6d;


double NormalizeAngleDouble(const double& angle_degrees);

void YawPitchRollToRotationMatrixDouble(const double yaw, const double pitch, const double roll, Eigen::Matrix3d & R);



class SmoothEulerOptimizer
{

public:
	public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	SmoothEulerOptimizer(int nNodeSize){
		this->m_gConnections.reserve(nNodeSize);
		//Construct the connections.
		for (int i=0;i<nNodeSize;i++){
			this->m_gConnections.push_back(map<int,Eigen::Vector4d>());
		}
	}

	inline void AddKeyFramePose(Vector6d mPose){
		this->m_gKeyFramePoses.push_back(mPose);
	}

	inline void AddConnections(int nRefIndex, int nConnectedIndex, Eigen::Vector4d mPose_po){
		this->m_gConnections[nRefIndex][nConnectedIndex] = mPose_po;
	}


	void Optimize(int nStartNode = -1){
		if (nStartNode == -1){
			nStartNode = this->m_gKeyFramePoses.size()-1;
		}

		for (int nIteration = 0; nIteration < 5; nIteration++){
			for (int i=nStartNode;i>1;i--){
				vector<Vector6d> gPoses;
				map<int, Eigen::Vector4d> dConnections = this->m_gConnections[i];
				gPoses.reserve(dConnections.size());
				//Compute estimated poses.
				for (map<int, Eigen::Vector4d>::iterator pIter = dConnections.begin(); pIter !=dConnections.end(); pIter++){
					//previous_optimized
					Eigen::Vector4d mRelativePose_po = pIter->second;

					double nRelativeYaw_op = -mRelativePose_po(0 , 0);
					Eigen::Vector3d mRelativeTranslation_po = Eigen::Vector3d(
						mRelativePose_po(1 , 0),
						mRelativePose_po(2 , 0),
						mRelativePose_po(3 , 0));

					int nPreviousIndex = pIter->first;
					Vector6d mPreviousPose_wp = this->m_gKeyFramePoses[nPreviousIndex];
					Vector6d mOptimizedPose_wo = this->m_gKeyFramePoses[i];
					
					
					double nPreviousYaw = mPreviousPose_wp(0 , 0);
					double nOptimizedYaw = mOptimizedPose_wo(0 , 0);
					double nPredictedYaw = NormalizeAngleDouble(0.5 * (nOptimizedYaw + (nPreviousYaw + nRelativeYaw_op)));

					//Compute the translations.
					Eigen::Matrix3d mPreviousRotation_wp;
					YawPitchRollToRotationMatrixDouble(
						mPreviousPose_wp(0 , 0), 
						mPreviousPose_wp(1 , 0), 
						mPreviousPose_wp(2 , 0), 
						mPreviousRotation_wp);
					
					Eigen::Vector3d mPreviousTranslation_wp(
						mPreviousPose_wp(3 , 0),
						mPreviousPose_wp(4 , 0),
						mPreviousPose_wp(5 , 0));
					Eigen::Vector3d mOptimizedTranslation_wo(
						mOptimizedPose_wo(3 , 0),
						mOptimizedPose_wo(4 , 0),
						mOptimizedPose_wo(5 , 0));

					Eigen::Vector3d mPredictedTranslation_wo =  mPreviousTranslation_wp + mPreviousRotation_wp * mRelativeTranslation_po;
					mPredictedTranslation_wo = (mPredictedTranslation_wo + mOptimizedTranslation_wo)/2;
					Eigen::Vector3d mDiff2 = mPredictedTranslation_wo - mOptimizedTranslation_wo;


					Vector6d mEstimatePose;
					mEstimatePose << 	nPredictedYaw, mOptimizedPose_wo(1 , 0), mOptimizedPose_wo(2 , 0), 
										mPredictedTranslation_wo(0 , 0), mPredictedTranslation_wo(1 , 0), mPredictedTranslation_wo(2 , 0);

					gPoses.push_back(mEstimatePose);
				}
				if (gPoses.size()==0){
					continue;
				}
				Vector6d mOptimizedPose_wo = this->m_gKeyFramePoses[i];
				Vector6d mUpdatedPose = this->TrajectoryInterpolateMulti(gPoses, mOptimizedPose_wo);
				this->m_gKeyFramePoses[i] = mUpdatedPose;
			}
		}
		
	}

	vector<Vector6d> LoadPoses(){
		return this->m_gKeyFramePoses;
	}
private:
	//Euler and Translation.
	vector<Vector6d> m_gKeyFramePoses;

	//Relative Yaw and Translation.
	vector<map<int, Eigen::Vector4d>> m_gConnections;

	Vector6d TrajectoryInterpolateMulti(vector<Vector6d> gPoses, Vector6d mOptimizedPose_wo){
		double nRatio = 1.0/gPoses.size();
		double nUpdatedYaw = 0.0;
		double nFinalX = 0.0, nFinalY = 0.0, nFinalZ = 0.0;
		for (int i=0;i<gPoses.size();i++){
			nUpdatedYaw += NormalizeAngleDouble(gPoses[i](0 , 0) - mOptimizedPose_wo(0 , 0)); 
			nFinalX += gPoses[i](3,0)*nRatio;
			nFinalY += gPoses[i](4,0)*nRatio;
			nFinalZ += gPoses[i](5,0)*nRatio;
		}
		nUpdatedYaw /= gPoses.size();
		double nFinalYaw = mOptimizedPose_wo(0 , 0) +  nUpdatedYaw;
		Vector6d mFinalPose;
		mFinalPose << nFinalYaw, mOptimizedPose_wo(1 , 0), mOptimizedPose_wo(2 , 0), nFinalX, nFinalY, nFinalZ;
		return mFinalPose;
	}

};

