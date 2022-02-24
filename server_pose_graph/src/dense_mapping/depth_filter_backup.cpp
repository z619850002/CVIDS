
#include <omp.h>
#include "../../include/dense_mapping/depth_filter.h"
#include "../../include/dense_mapping/dense_mapping_parameters.h"
using namespace std;

const double PI = 3.14159;

double NormPdf(
    const double &x,
    const double &mu,
    const double & sigma_sq)
{
  return (exp(-(x-mu)*(x-mu) / (2.0*sigma_sq)))*sqrt(2.0*PI*sigma_sq);
}

DepthFilter::DepthFilter(int nHeight, int nWidth)
{
	cout << "Initialize params" << endl;
	this->m_bFirst = true;




	//Initialize params.
	cv::Mat mA = cv::Mat(nHeight, nWidth, CV_32F, 15.0);
	cv::Mat mB = cv::Mat(nHeight, nWidth, CV_32F,15.0);

	cv::Mat mInvDepthMu = cv::Mat(nHeight, nWidth, CV_32F, 0.5);
	cv::Mat mInvDepthCov = cv::Mat(nHeight, nWidth, CV_32F, 100.0);

	this->m_mA.upload(mA);
	this->m_mB.upload(mB);
	this->m_mInvDepthMu.upload(mInvDepthMu);
	this->m_mInvDepthCov.upload(mInvDepthCov);

	this->m_nMinInvDepth = 0.01;
	this->m_nMaxInvDepth = 100;
	this->m_nInvDepthRange = this->m_nMaxInvDepth - this->m_nMinInvDepth;
	cout << "Complete initialization" << endl;
}


//TODO::Use gpu
void DepthFilter::Update(	cv::Mat mUpdateMu, 
							cv::Mat mUpdateCov, 
							bool bOutlier){

	cout << "Start Filtering" << endl;
	

	cv::Mat mUpdateMuFloat, mUpdateCovFloat;
	mUpdateMu.convertTo(mUpdateMuFloat, CV_32FC1);
	mUpdateCov.convertTo(mUpdateCovFloat, CV_32FC1);

	cv::cuda::GpuMat mUpdateMuGpu, mUpdateCovGpu;
	mUpdateMuGpu.upload(mUpdateMuFloat);
	mUpdateCovGpu.upload(mUpdateCovFloat);


	int nWidth = mUpdateMu.cols;
	int nHeight = mUpdateMu.rows;

	FusionFilter( 	this->m_mA.data, this->m_mA.step,
					this->m_mB.data, this->m_mB.step,
					this->m_mInvDepthMu.data, this->m_mInvDepthMu.step,
                  	this->m_mInvDepthCov.data, this->m_mInvDepthCov.step,
                  	mUpdateMuGpu.data, mUpdateMuGpu.step, 
                  	mUpdateCovGpu.data, mUpdateCovGpu.step,
                  	nWidth, nHeight,
                  	this->m_nInvDepthRange,
                  	this->m_nMinInvDepth, 
                  	this->m_nMaxInvDepth);


	mUpdateMuFloat.release();
	mUpdateCovFloat.release();
	mUpdateMuGpu.release();
	mUpdateCovGpu.release();


	// #pragma omp parallel for
	// for (int u=0;u<nWidth;u++){
	// 	#pragma omp parallel for
	// 	for (int v=0;v<nHeight;v++){
	// 		double nA = this->m_mA.at<double>(v , u);
	// 		double nB = this->m_mB.at<double>(v , u);
	// 		double nOldMu = this->m_mInvDepthMu.at<double>(v , u);
	// 		double nOldSigma = sqrt(this->m_mInvDepthCov.at<double>(v , u));
	// 		//Square
	// 		double nOldSigmaSq = nOldSigma * nOldSigma;

	// 		double nNewMu = mUpdateMu.at<double>(v , u);
	// 		double nNewSigma = sqrt(mUpdateCov.at<double>(v , u));

	// 		if (nNewMu < 0.01 || nNewMu > 100){
	// 			//Outlier.
	// 			nB += 1;
	// 			this->m_mB.at<double>(v , u) = nB;
	// 			continue;
	// 		}

	// 		//Square
	// 		double nNewSigmaSq = nNewSigma * nNewSigma;

	// 		double nM = (nNewSigmaSq * nOldMu + nOldSigmaSq * nNewMu)/(nOldSigmaSq + nNewSigmaSq);
	// 		//Square of s
	// 		double nS = (nNewSigmaSq * nOldSigmaSq)/(nNewSigmaSq + nOldSigmaSq);
			
	// 		double nC1 = (nA/(nA + nB)) * NormPdf(nNewMu, nOldMu, nNewSigmaSq + nOldSigmaSq);
	// 		double nC2 = (nB/(nA + nB)) * 1.0 / this->m_nInvDepthRange;

			

	// 		double nNorm = nC2 + nC1;
	// 		//Normalize c1 and c2
	// 		nC1 /= nNorm;
	// 		nC2 /= nNorm;

	// 		// if (nC1 < 0.9){
	// 		// 	//Outlier.
	// 		// 	nB += 1;
	// 		// 	this->m_mB.at<double>(v , u) = nB;
	// 		// 	continue;
	// 		// }


	// 		//C_pi1
	// 		double nF = nC1 * ((nA+1.0)/(nA+nB+1.0)) + nC2 * (nA/(nA+nB+1.0));
	// 		//C_pi2
	// 		double nE = nC1 * ((nA + 1.0) * (nA + 2.0)) / ((nA + nB + 1.0) * (nA + nB + 2.0)) +
	// 					nC2 * ((nA) * (nA + 1.0)) / ((nA + nB + 1.0) * (nA + nB + 2.0));


	// 		if (isnan(nC1 * nM)){
	// 			continue;
	// 		}

	// 		//Get the final fused result.
	// 		double nFusedMu = nC1 * nM + nC2 * nOldMu;
	// 		double nFusedSigma = nC1 * (nS + nM * nM) + nC2 * (nOldSigmaSq + nOldMu * nOldMu) - nFusedMu * nFusedMu;
	// 		double nFusedA = (nE - nF) / (nF - nE/nF);
	// 		double nFusedB = nFusedA * (1.0 - nF) / nF;

	// 		//Assign the fused value to the matrix.
	// 		this->m_mA.at<double>(v , u) = nFusedA;
	// 		this->m_mB.at<double>(v , u) = nFusedB;
	// 		this->m_mInvDepthMu.at<double>(v , u) = nFusedMu;
	// 		this->m_mInvDepthCov.at<double>(v , u) = nFusedSigma*nFusedSigma;
	// 	}
	// }
}





void DepthFilter::PropogateDepth
(	cv::Mat & mLastRatio,
	cv::Mat & mLastMu,
	cv::Mat & mLastCov, 
	cv::Mat & mLastA,
	cv::Mat & mLastB,
	Eigen::Matrix3d mK,
	Sophus::SE3 mRelativePose_co)
{
	int nWidth = this->m_mA.cols;
	int nHeight = this->m_mA.rows;

	cv::Mat mInvDepthMu, mInvDepthCov, mA, mB;
	this->m_mInvDepthMu.download(mInvDepthMu);
	this->m_mInvDepthCov.download(mInvDepthCov);
	this->m_mA.download(mA);


	cv::Mat mLastDepthMap = mLastMu;
	//Update the depth filter.
	#pragma omp parallel for
	for (int u=0;u<nWidth;u++){
		#pragma omp parallel for
		for (int v=0;v<nHeight;v++){
 			double nDepth = 1.0/mLastDepthMap.at<double>(v , u);
 			double nRatio = mLastRatio.at<double>(v , u);
 			double nCov = mLastCov.at<double>(v , u);
            Eigen::Vector3d mLastPoint3D(u , v , 1);
            mLastPoint3D = mK.inverse() * mLastPoint3D;
            mLastPoint3D /= mLastPoint3D(2);
            mLastPoint3D *= nDepth;
            mLastPoint3D = mRelativePose_co * mLastPoint3D;
            double nNewDepth = mLastPoint3D(2);
            mLastPoint3D /= mLastPoint3D(2);
            mLastPoint3D = mK * mLastPoint3D;
    		int uu = mLastPoint3D(0), vv = mLastPoint3D(1);
    		if (uu < 3 || uu > nWidth-3 || vv < 3 || vv > nHeight-3){
    			continue;
    		}
    		for (int us = -1; us<=1; us++){
    			for (int vs = -1; vs<=1; vs++){
    				if (nDepth > 0.1 && nDepth < 100 && nRatio > 0.5){
						double nScalar = (nDepth/nNewDepth);
						nScalar *= nScalar;
						nScalar *= nScalar;
						mInvDepthMu.at<float>(vv+vs , uu+us) = 1.0/nNewDepth;
						mInvDepthCov.at<float>(vv+vs , uu+us) = nScalar * nCov + 0.1;
						mA.at<float>(vv+vs , uu+us) +=0.1;
		    		}		
    			}
    		}
		}
	}

	this->m_mInvDepthMu.upload(mInvDepthMu);
	this->m_mInvDepthCov.upload(mInvDepthCov);
	this->m_mA.upload(mA);

}