#ifndef PCM_VOLUME_H_
#define PCM_VOLUME_H_

#include <Eigen/Core>
#include <Eigen/Dense>

#include <iostream>
#include <vector>
using namespace std;


class CovVolume
{
public:
	CovVolume(){

	}


	void RecomputeCov(){
		this->m_mCov = this->m_mDeltaF * this->m_mInitialCov * this->m_mDeltaF.transpose();
	}


	int m_nChainIndex;

	Eigen::Matrix4d m_mInitialCov, m_mObsCov;

	Eigen::Matrix4d m_mCov;
	Eigen::Matrix4d m_mDeltaF, m_mDeltaG, m_mDeltaFG;



	
};


#endif