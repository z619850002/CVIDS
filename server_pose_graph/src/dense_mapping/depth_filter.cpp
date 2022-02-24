
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

// ------------------------------------------------------------------
// parameters 
const int boarder = 20;     // 边缘宽度
const int width = 640;      // 宽度 
const int height = 480;     // 高度
const double fx = 481.2f /640.0*width;   // 相机内参
const double fy = -480.0f / 480.0*height;
const double cx = 319.5f / 640.0*width;
const double cy = 239.5f/ 480.0*height;
const int ncc_window_size = 2;  // NCC 取的窗口半宽度
const int ncc_area = (2*ncc_window_size+1)*(2*ncc_window_size+1); // NCC窗口面积
const double min_cov = 0.1; // 收敛判定：最小方差
const double max_cov = 10;  // 发散判定：最大方差


using namespace Sophus;
using namespace Eigen;
using namespace cv;
using namespace std;

// 像素到相机坐标系 
inline Vector3d px2cam ( const Vector2d px ) {
    return Vector3d ( 
        (px(0,0) - cx)/fx,
        (px(1,0) - cy)/fy, 
        1
    );
}

// 相机坐标系到像素 
inline Vector2d cam2px ( const Vector3d p_cam ) {
    return Vector2d (
        p_cam(0,0)*fx/p_cam(2,0) + cx, 
        p_cam(1,0)*fy/p_cam(2,0) + cy 
    );
}


double ComputeUncertainty(
    const Vector2d& pt_ref,  
    const double nInvDepth,
    const SE3& T_C_R)
{
    // 我是一只喵
    // 不知道这段还有没有人看
    // 用三角化计算深度
    SE3 T_R_C = T_C_R.inverse();
    Vector3d f_ref = px2cam( pt_ref );
    f_ref.normalize();
    
    
    // 方程
    // d_ref * f_ref = d_cur * ( R_RC * f_cur ) + t_RC
    // => [ f_ref^T f_ref, -f_ref^T f_cur ] [d_ref] = [f_ref^T t]
    //    [ f_cur^T f_ref, -f_cur^T f_cur ] [d_cur] = [f_cur^T t]
    // 二阶方程用克莱默法则求解并解之
    Vector3d t = T_R_C.translation();
    
    double depth_estimation = 1.0/nInvDepth;   // 深度值
    // cout << "Depth estimation2 is: " << depth_estimation << endl;
    
    // 计算不确定性（以一个像素为误差）
    Vector3d p = f_ref*depth_estimation;
    Vector3d a = p - t; 
    double t_norm = t.norm();
    double a_norm = a.norm();
    double alpha = acos( f_ref.dot(t)/t_norm );
    double beta = acos( -a.dot(t)/(a_norm*t_norm));
    double beta_prime = beta + atan(1/fx);
    double gamma = M_PI - alpha - beta_prime;
    double p_prime = t_norm * sin(beta_prime) / sin(gamma);
    //Compute the covariance
    double d_cov = 1.0/p_prime - nInvDepth; 

    return d_cov;
}




DepthFilter::DepthFilter(int nHeight, int nWidth){
	//Initialize params.
	this->m_mA = cv::Mat(nHeight, nWidth, CV_64F, 15.0);
	this->m_mB = cv::Mat(nHeight, nWidth, CV_64F, 15.0);

	this->m_mInvDepthMu = cv::Mat(nHeight, nWidth, CV_64F, 0.5);
	this->m_mInvDepthCov = cv::Mat(nHeight, nWidth, CV_64F, 100);

	this->m_nMinInvDepth = 0.01;
	this->m_nMaxInvDepth = 100;
	this->m_nInvDepthRange = this->m_nMaxInvDepth - this->m_nMinInvDepth;

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

	cv::Mat mLastDepthMap = mLastMu;
	//Update the depth filter.
	for (int u=0;u<nWidth;u++){
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
						this->m_mInvDepthMu.at<double>(vv+vs , uu+us) = 1.0/nNewDepth;
						double nScalar = (nDepth/nNewDepth);
						nScalar *= nScalar;
						nScalar *= nScalar;
						this->m_mInvDepthCov.at<double>(vv+vs , uu+us) = nScalar * nCov + 0.1;
						this->m_mA.at<double>(vv+vs , uu+us) +=0.1;
		    		}		
    			}
    		}
    		
    //                 if (mInvDepth2.at<double>(vv , uu) <0.01){
    //                     mInvDepth2.at<double>(vv , uu) = 1.0/nDepth;
    //                     mInvDepth2.at<double>(vv-1 , uu) = 1.0/nDepth;
    //                     mInvDepth2.at<double>(vv+1 , uu) = 1.0/nDepth;
    //                     mInvDepth2.at<double>(vv , uu-1) = 1.0/nDepth;
    //                     mInvDepth2.at<double>(vv-1 , uu-1) = 1.0/nDepth;
    //                     mInvDepth2.at<double>(vv+1 , uu-1) = 1.0/nDepth;
    //                     mInvDepth2.at<double>(vv , uu+1) = 1.0/nDepth;
    //                     mInvDepth2.at<double>(vv-1 , uu+1) = 1.0/nDepth;
    //                     mInvDepth2.at<double>(vv+1 , uu+1) = 1.0/nDepth;
    //                 }


		}
	}


}


void DepthFilter::Update(	cv::Mat mUpdateMu, 
							cv::Mat mUpdateCov, 
							bool bOutlier){

	int nWidth = mUpdateMu.cols;
	int nHeight = mUpdateMu.rows;

	for (int u=0;u<nWidth;u++){
		for (int v=0;v<nHeight;v++){



			double nA = this->m_mA.at<double>(v , u);
			double nB = this->m_mB.at<double>(v , u);
			double nOldMu = this->m_mInvDepthMu.at<double>(v , u);
			double nOldSigma = sqrt(this->m_mInvDepthCov.at<double>(v , u));
			//Square
			double nOldSigmaSq = nOldSigma * nOldSigma;

			double nNewMu = mUpdateMu.at<double>(v , u);
			double nNewSigma = sqrt(mUpdateCov.at<double>(v , u));

			// ComputeUncertainty( Eigen::Vector2d(u , v), nNewMu,  )



			if (nNewMu < 0.01 || nNewMu > 100){
				//Outlier.
				nB += 1;
				this->m_mB.at<double>(v , u) = nB;
				continue;
			}

			//Square
			double nNewSigmaSq = nNewSigma * nNewSigma;

			double nM = (nNewSigmaSq * nOldMu + nOldSigmaSq * nNewMu)/(nOldSigmaSq + nNewSigmaSq);
			//Square of s
			double nS = (nNewSigmaSq * nOldSigmaSq)/(nNewSigmaSq + nOldSigmaSq);
			
			double nC1 = (nA/(nA + nB)) * NormPdf(nNewMu, nOldMu, nNewSigmaSq + nOldSigmaSq);
			double nC2 = (nB/(nA + nB)) * 1.0 / this->m_nInvDepthRange;

			//Normalize c1 and c2
			double nNorm = nC1 + nC2;
			nC1 /= nNorm;
			nC2 /= nNorm;

			// if (nC1 < 0.9){
			// 	//Outlier.
			// 	nB += 1;
			// 	this->m_mB.at<double>(v , u) = nB;
			// 	continue;
			// }

			//C_pi1
			double nF = nC1 * ((nA+1.0)/(nA+nB+1.0)) + nC2 * (nA/(nA+nB+1.0));
			//C_pi2
			double nE = nC1 * ((nA + 1.0) * (nA + 2.0)) / ((nA + nB + 1.0) * (nA + nB + 2.0)) +
						nC2 * ((nA) * (nA + 1.0)) / ((nA + nB + 1.0) * (nA + nB + 2.0));


			if (isnan(nC1 * nM)){
				continue;
			}

			//Get the final fused result.
			double nFusedMu = nC1 * nM + nC2 * nOldMu;
			double nFusedSigma = nC1 * (nS + nM * nM) + nC2 * (nOldSigmaSq + nOldMu * nOldMu) - nFusedMu * nFusedMu;
			double nFusedA = (nE - nF) / (nF - nE/nF);
			double nFusedB = nFusedA * (1.0 - nF) / nF;

			//Assign the fused value to the matrix.
			this->m_mA.at<double>(v , u) = nFusedA;
			this->m_mB.at<double>(v , u) = nFusedB;
			this->m_mInvDepthMu.at<double>(v , u) = nFusedMu;
			this->m_mInvDepthCov.at<double>(v , u) = nFusedSigma*nFusedSigma;


    		
		}
	}
}





void DepthFilter::Update(	cv::Mat mUpdateMu, 
							cv::Mat mUpdateCov, 
							Sophus::SE3& mT_CR,
							bool bOutlier){

	int nWidth = mUpdateMu.cols;
	int nHeight = mUpdateMu.rows;

	for (int u=0;u<nWidth;u++){
		for (int v=0;v<nHeight;v++){



			double nA = this->m_mA.at<double>(v , u);
			double nB = this->m_mB.at<double>(v , u);
			double nOldMu = this->m_mInvDepthMu.at<double>(v , u);
			double nOldSigma = sqrt(this->m_mInvDepthCov.at<double>(v , u));
			//Square
			double nOldSigmaSq = nOldSigma * nOldSigma;

			double nNewMu = mUpdateMu.at<double>(v , u);
			double nNewSigma = sqrt(mUpdateCov.at<double>(v , u));

			nNewSigma = ComputeUncertainty( Eigen::Vector2d(u , v), nNewMu,  mT_CR);



			if (nNewMu < 0.01 || nNewMu > 100){
				//Outlier.
				nB += 1;
				this->m_mB.at<double>(v , u) = nB;
				continue;
			}

			//Square
			double nNewSigmaSq = nNewSigma * nNewSigma;

			double nM = (nNewSigmaSq * nOldMu + nOldSigmaSq * nNewMu)/(nOldSigmaSq + nNewSigmaSq);
			//Square of s
			double nS = (nNewSigmaSq * nOldSigmaSq)/(nNewSigmaSq + nOldSigmaSq);
			
			double nC1 = (nA/(nA + nB)) * NormPdf(nNewMu, nOldMu, nNewSigmaSq + nOldSigmaSq);
			double nC2 = (nB/(nA + nB)) * 1.0 / this->m_nInvDepthRange;

			//Normalize c1 and c2
			double nNorm = nC1 + nC2;
			nC1 /= nNorm;
			nC2 /= nNorm;

			// if (nC1 < 0.9){
			// 	//Outlier.
			// 	nB += 1;
			// 	this->m_mB.at<double>(v , u) = nB;
			// 	continue;
			// }

			//C_pi1
			double nF = nC1 * ((nA+1.0)/(nA+nB+1.0)) + nC2 * (nA/(nA+nB+1.0));
			//C_pi2
			double nE = nC1 * ((nA + 1.0) * (nA + 2.0)) / ((nA + nB + 1.0) * (nA + nB + 2.0)) +
						nC2 * ((nA) * (nA + 1.0)) / ((nA + nB + 1.0) * (nA + nB + 2.0));


			if (isnan(nC1 * nM)){
				continue;
			}

			//Get the final fused result.
			double nFusedMu = nC1 * nM + nC2 * nOldMu;
			double nFusedSigma = nC1 * (nS + nM * nM) + nC2 * (nOldSigmaSq + nOldMu * nOldMu) - nFusedMu * nFusedMu;
			double nFusedA = (nE - nF) / (nF - nE/nF);
			double nFusedB = nFusedA * (1.0 - nF) / nF;

			//Assign the fused value to the matrix.
			this->m_mA.at<double>(v , u) = nFusedA;
			this->m_mB.at<double>(v , u) = nFusedB;
			this->m_mInvDepthMu.at<double>(v , u) = nFusedMu;
			this->m_mInvDepthCov.at<double>(v , u) = nFusedSigma*nFusedSigma;


    		
		}
	}
}