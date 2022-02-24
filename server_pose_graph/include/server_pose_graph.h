#pragma once

#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <string>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <queue>
#include <assert.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <ros/ros.h>
#include "server_keyframe.h"
#include "./optimizer/smooth_euler_optimizer.h"
#include <map>
#include <list>
#include <mutex>

#include "utility/server_utility.h"
#include "utility/server_visualization.h"
#include "server_plotter.h"


#include "../ThirdParty/DBoW/DBoW2.h"
#include "../ThirdParty/DVision/DVision.h"
#include "../ThirdParty/DBoW/TemplatedDatabase.h"
#include "../ThirdParty/DBoW/TemplatedVocabulary.h"

#include "./pcm_graph.h"

#define SHOW_S_EDGE false
#define SHOW_L_EDGE true
#define SAVE_LOOP_PATH true

using namespace std;

class ServerPoseGraph
{
public:
	//Constructor and deconstructor.
	ServerPoseGraph();
	~ServerPoseGraph();

	void AddDisturbance(){
		double nRatio = 1;
		this->m_mKeyFrameListMutex.lock();
		int nNum = 0;
		for (ServerKeyFrame * pKeyFrame : this->m_gAllKeyFrames){
			// nNum++;
			// if (nNum < 2){
			// 	continue;
			// }
			// nNum = 0;
			if (pKeyFrame->m_bHasLoop && !pKeyFrame->m_bHasBeenDisturbed){
					
					double nAngleDisturbance = 0.2;
					double nTranslationDisturbance = 0.02;
					for (int i=0;i<pKeyFrame->m_gLoopInfos.size();i++){

						pKeyFrame->m_gLoopInfos[i](7) = pKeyFrame->m_gLoopInfos[i](7) + nRatio * nAngleDisturbance;  

						pKeyFrame->m_gLoopInfos[i](0) = pKeyFrame->m_gLoopInfos[i](0) + nRatio * nTranslationDisturbance;  
						pKeyFrame->m_gLoopInfos[i](1) = pKeyFrame->m_gLoopInfos[i](1) + nRatio * nTranslationDisturbance;  
						pKeyFrame->m_gLoopInfos[i](2) = pKeyFrame->m_gLoopInfos[i](2) + nRatio * nTranslationDisturbance;	
					}		
					pKeyFrame->m_bHasBeenDisturbed = true;

			}
			  
		}

		this->m_mKeyFrameListMutex.unlock();
	}


	//TODO:Publisher should be registered.
	// void registerPub(ros::NodeHandle &n);

	//Register a new client.
	void RegisterClient(int nClientID);

	//Load the vocabulary.

	void LoadVocabulary(string aVocPath);


	//Add one frame.
	void AddKeyFrame(ServerKeyFrame* pKeyFrame);

	//Align 2 submaps with 2 matched keyframes.
	void AlignSubMaps(	ServerKeyFrame * pAlignedKeyFrame, ServerKeyFrame * pNotAlignedKeyFrame,
						Eigen::Matrix3d mRelativeQ, Eigen::Vector3d mRelativeT);
	
	void UpdateSubMaps( double nShiftYaw, Eigen::Vector3d mRelativeT,
    									ServerKeyFrame * pUpdateKeyFrame, int nClientID);


	void RecordConnection(int nFirstLoopIndex, int nCurrentIndex);


	//Find loop closure
	int DetectLoop(ServerKeyFrame* pKeyFrame, int nGlobalFrameIndex, bool bNew = true);

	//Get one keyframe
	ServerKeyFrame* GetKeyFrame(int nGlobalIndex);

	ServerKeyFrame * GetPreviousKeyFrame(ServerKeyFrame * pKeyFrame);

	//Optimize
	void Optimize4DoF();


	void SetPlotter(ServerPlotter * pServerPlotter);
	ServerPlotter * GetPlotter();

	//Used for visualization.
	//Not used yet!!
	// void UpdatePath();

	ServerKeyFrame * PopUpdatedKeyFrame(){
		ServerKeyFrame * pResultKeyFrame = NULL;
		// this->m_mUpdatedKeyFramesMutex.lock();

		// if (!this->m_gUpdatedKeyFrames.empty()){
		// 	pResultKeyFrame =  this->m_gUpdatedKeyFrames.front();
		// 	this->m_gUpdatedKeyFrames.pop();
		// }

		// this->m_mUpdatedKeyFramesMutex.unlock();
		return pResultKeyFrame;
	}

	void InsertUpdatedKeyFrame(ServerKeyFrame * pKeyFrame){
		// this->m_mUpdatedKeyFramesMutex.lock();
		// this->m_gUpdatedKeyFrames.push(pKeyFrame);
		// this->m_mUpdatedKeyFramesMutex.unlock();
	}

	ServerKeyFrame * PopDepthFrames();


	//Visualizer.
	ServerVisualization* m_pPoseGraphVisualization;
	
	//ClientID CurrentID
	map<int, int> m_dLocalIndices;
	map<int, vector<ServerKeyFrame *>> m_dKeyFramesEachMap;


	nav_msgs::Path m_gPaths[10]; 			//path[10]
	nav_msgs::Path m_iBasePath;			//base_path
	
	//The poses of each map.
	//These variables should be copied to multiple.
	//Drift is updated in the optimization.
	map<int, Eigen::Vector3d> m_dTranslationDrift;	//Vector3d t_drift;
	map<int, double>	m_dYawDrift;				//double yaw_drift;
	map<int, Eigen::Matrix3d>	m_dRotationDrift;	//Matrix3d r_drift;
	// world frame( base sequence or first sequence)<----> cur sequence frame  
	map<int, Eigen::Vector3d> m_dLocalTranslation_wr; //Vector3d w_t_vio;
	map<int, Eigen::Matrix3d> m_dLocalRotation_wr;	//Matrix3d w_r_vio;
	//If the submap has been aligned with the world coordinate system.
	map<int, bool> m_dAligned;						//sequence_align_world


	//Used for dense mapping.
	map<int, ServerKeyFrame *> m_dMappingRefKeyFrames;

	int GetKeyFrameSize(){
		this->m_mKeyFrameListMutex.lock();
		int nNum = this->m_gAllKeyFrames.size();
		this->m_mKeyFrameListMutex.unlock();
		return nNum;
	}


public:
	//Add the keyframe to database.
	void AddKeyFrameIntoVOC(ServerKeyFrame * pKeyFrame);

	//Used for PCM.
	ConnectionGraph * m_pConnectionGraph;

	//All keyframes.
	//Useless now.
	// map<int , list<ServerKeyFrame*>> m_dKeyFrameLists; 	//list<KeyFrame*> keyframelist;
	vector<ServerKeyFrame *> m_gAllKeyFrames;

	//
	queue<ServerKeyFrame *>  m_gOptimizationBuf; //std::queue<int> optimize_buf;


	//All Mutex used in multi threads
	std::mutex m_mKeyFrameListMutex;					//std::mutex m_keyframelist;
	std::mutex m_mOptimizationBufMutex;					//std::mutex m_optimize_buf;
	std::mutex m_mPathMutex;							//std::mutex m_path;
	std::mutex m_mDriftMutex;							//std::mutex m_drift;
	
	std::thread m_tOptimizationThread;					//std::thread t_optimization;
	
	//The global index for all keyframes.
	int m_nGlobalIndex;									//int global_index

	//If the submap has been loop closed.
	//Not used yet!
	// vector<bool> m_gIsLoopClosed;						//vector<bool> sequence_loop

	//Used for debug.
	map<int, cv::Mat> m_dImagePool;

	int m_nEarliestLoopIndex;							//int earliest_loop_index;
	int m_nLastLoopIndex;
	int m_nBaseSequenceIndex;							//int base_sequence;

	int m_nLastOptimizationIndex;


	int m_nFirstClient;

	ServerPlotter * m_pPlotter;



	BriefDatabase  m_iBriefDB;							//db
	BriefVocabulary * m_pVocabulary;						//voc


	queue<ServerKeyFrame *> m_gUpdatedKeyFrames;
	mutex m_mUpdatedKeyFramesMutex;


	queue<ServerKeyFrame *> m_gDenseKeyFrames;
	mutex m_mDenseKeyFramesMutex;



};

template <typename T>
T NormalizeAngle(const T& angle_degrees) {
  if (angle_degrees > T(180.0))
  	return angle_degrees - T(360.0);
  else if (angle_degrees < T(-180.0))
  	return angle_degrees + T(360.0);
  else
  	return angle_degrees;
};

class AngleLocalParameterization {
 public:

  template <typename T>
  bool operator()(const T* theta_radians, const T* delta_theta_radians,
                  T* theta_radians_plus_delta) const {
    *theta_radians_plus_delta =
        NormalizeAngle(*theta_radians + *delta_theta_radians);

    return true;
  }

  static ceres::LocalParameterization* Create() {
    return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,
                                                     1, 1>);
  }
};

template <typename T> 
void YawPitchRollToRotationMatrix(const T yaw, const T pitch, const T roll, T R[9])
{

	T y = yaw / T(180.0) * T(M_PI);
	T p = pitch / T(180.0) * T(M_PI);
	T r = roll / T(180.0) * T(M_PI);


	R[0] = cos(y) * cos(p);
	R[1] = -sin(y) * cos(r) + cos(y) * sin(p) * sin(r);
	R[2] = sin(y) * sin(r) + cos(y) * sin(p) * cos(r);
	R[3] = sin(y) * cos(p);
	R[4] = cos(y) * cos(r) + sin(y) * sin(p) * sin(r);
	R[5] = -cos(y) * sin(r) + sin(y) * sin(p) * cos(r);
	R[6] = -sin(p);
	R[7] = cos(p) * sin(r);
	R[8] = cos(p) * cos(r);
};

template <typename T> 
void RotationMatrixTranspose(const T R[9], T inv_R[9])
{
	inv_R[0] = R[0];
	inv_R[1] = R[3];
	inv_R[2] = R[6];
	inv_R[3] = R[1];
	inv_R[4] = R[4];
	inv_R[5] = R[7];
	inv_R[6] = R[2];
	inv_R[7] = R[5];
	inv_R[8] = R[8];
};

template <typename T> 
void RotationMatrixRotatePoint(const T R[9], const T t[3], T r_t[3])
{
	r_t[0] = R[0] * t[0] + R[1] * t[1] + R[2] * t[2];
	r_t[1] = R[3] * t[0] + R[4] * t[1] + R[5] * t[2];
	r_t[2] = R[6] * t[0] + R[7] * t[1] + R[8] * t[2];
};

struct FourDOFError
{
	FourDOFError(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i)
				  :t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i){}

	template <typename T>
	bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
	{
		T t_w_ij[3];
		t_w_ij[0] = tj[0] - ti[0];
		t_w_ij[1] = tj[1] - ti[1];
		t_w_ij[2] = tj[2] - ti[2];

		// euler to rotation
		T w_R_i[9];
		YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
		// rotation transpose
		T i_R_w[9];
		RotationMatrixTranspose(w_R_i, i_R_w);
		// rotation matrix rotate point
		T t_i_ij[3];
		RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);

		residuals[0] = (t_i_ij[0] - T(t_x));
		residuals[1] = (t_i_ij[1] - T(t_y));
		residuals[2] = (t_i_ij[2] - T(t_z));
		residuals[3] = NormalizeAngle(yaw_j[0] - yaw_i[0] - T(relative_yaw));

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
									   const double relative_yaw, const double pitch_i, const double roll_i) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          FourDOFError, 4, 1, 3, 1, 3>(
	          	new FourDOFError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
	}

	double t_x, t_y, t_z;
	double relative_yaw, pitch_i, roll_i;

};

struct FourDOFWeightError
{
	FourDOFWeightError(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i)
				  :t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i){
				  	weight = 1;
				  }

	template <typename T>
	bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
	{
		T t_w_ij[3];
		t_w_ij[0] = tj[0] - ti[0];
		t_w_ij[1] = tj[1] - ti[1];
		t_w_ij[2] = tj[2] - ti[2];

		// euler to rotation
		T w_R_i[9];
		YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
		// rotation transpose
		T i_R_w[9];
		RotationMatrixTranspose(w_R_i, i_R_w);
		// rotation matrix rotate point
		T t_i_ij[3];
		RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);

		residuals[0] = (t_i_ij[0] - T(t_x)) * T(weight);
		residuals[1] = (t_i_ij[1] - T(t_y)) * T(weight );
		residuals[2] = (t_i_ij[2] - T(t_z)) * T(weight );
		residuals[3] = NormalizeAngle((yaw_j[0] - yaw_i[0] - T(relative_yaw))) * T(weight) / T(10.0)  ;

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
									   const double relative_yaw, const double pitch_i, const double roll_i) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          FourDOFWeightError, 4, 1, 3, 1, 3>(
	          	new FourDOFWeightError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
	}

	double t_x, t_y, t_z;
	double relative_yaw, pitch_i, roll_i;
	double weight;

};