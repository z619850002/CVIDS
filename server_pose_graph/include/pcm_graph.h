#ifndef CONNECTION_GRAPH_H_
#define CONNECTION_GRAPH_H_

#include <vector>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <string>
#include <sstream>
#include "server_keyframe.h"
#include "./fmc/pcm.h"

//Sophus

#include "sophus/so3.h"
#include "sophus/se3.h"

#include "utility/server_utility.h"

using namespace std;

class MapConnection
{
public:
	MapConnection(){}

	//The loop frame of keyframe2 is keyframe1
	MapConnection(
		ServerKeyFrame * pKeyFrame1, 
		ServerKeyFrame * pKeyFrame2, 
		Sophus::SE3 mTransform_12){
		this->m_pKeyFrame1 = pKeyFrame1;
		this->m_pKeyFrame2 = pKeyFrame2;
		this->m_mTransform_12 = mTransform_12;
	}
	
	ServerKeyFrame * m_pKeyFrame1;
	ServerKeyFrame * m_pKeyFrame2;
	//Relative Pose from frame 2 to frame 1 
	Sophus::SE3 m_mTransform_12;
};


class ConnectionGraph
{
public:
	//Constructor.
	ConnectionGraph(int nAgentNum);

	void AddConnection(MapConnection * pConnection);

	void ComputeGraph();

	void ClearGraph();

	void SaveGraph(string aFilePath);

	vector<MapConnection *> GetConnections(){
		return this->m_gConnections;
	}

	// void SaveGraph(string aFilePath);


private:
	//Number of agents.
	int m_nAgentNum;

	//The threshold.
	double m_nGamma;

	//All connections.
	vector<MapConnection *> m_gConnections;


	
};


class GraphConsistencyEvaluator : public PCM::ConsistencyEvaluator
{
Eigen::MatrixXi m_mGraph;

public:
GraphConsistencyEvaluator(Eigen::MatrixXi mGraph)
{
  this->m_mGraph = mGraph;
}

bool evaluate_consistency(int i, int j)
{
  return this->m_mGraph(i,j);
}
};



#endif