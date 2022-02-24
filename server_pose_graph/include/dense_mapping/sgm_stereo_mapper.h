#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>

#include "./dense_mapping_parameters.h"
#include "./calc_cost.h"

#include <vector>
using namespace std;

class StereoMapper
{
  public:
    StereoMapper(int nRealWidth, int nRealHeight);
    void InitIntrinsic(const cv::Mat &mK1, const cv::Mat &mD1, const cv::Mat &mK2, const cv::Mat &mD2);

    void InitReference(const cv::Mat &mRefImage);

    void Update(const cv::Mat &mMatchImage, 
                const cv::Mat &mRefRotation, 
                const cv::Mat &mRefTranslation, 
                const cv::Mat &mMatchRotation, 
                const cv::Mat &mMatchTranslation);

    void ClearRawCost();

    cv::Mat Output();

    void ReleaseSpaces();

    cv::cuda::GpuMat m_mRefImage, m_mMatchImage;

    cv::cuda::GpuMat m_mPhotometricCost, m_mSgmCost;
    cv::cuda::GpuMat m_mDepthMap;
    cv::cuda::GpuMat m_mGradient;


    cv::cuda::GpuMat m_mSparseDepth, m_mSparseDistance;

    cv::Mat m_mGradientX, m_mGradientY;

    //Intrinsics
    cv::Mat m_mK1, m_mK2;
    cv::Mat m_mD1, m_mD2;

    cv::Mat m_mR, m_mT;


    float m_nMean1, m_nMean2;



    int m_nMeasurementCount;

    int m_nRealWidth, m_nRealHeight;


    vector<double> m_gSparseDepth;
    vector<cv::Point2d> m_gSparsePoints2D;

    void BindSparsePoints(
        vector<double> & gSparseDepth, 
        vector<cv::Point2d> & gSparsePoints){
        this->m_gSparseDepth = gSparseDepth;
        this->m_gSparsePoints2D = gSparsePoints;
    }
};
