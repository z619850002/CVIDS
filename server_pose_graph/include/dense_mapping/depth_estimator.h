#ifndef DEPTH_ESTIMATOR_H_
#define DEPTH_ESTIMATOR_H_

#include <iostream>
#include <vector>
#include <fstream>
using namespace std; 
#include <boost/timer.hpp>

// for sophus 
#include <sophus/se3.h>


// for eigen 
#include <Eigen/Core>
#include <Eigen/Geometry>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include "./sgm_stereo_mapper.h"
#include "./depth_filter.h"

using Sophus::SE3;
using namespace Eigen;
using namespace cv;



class DepthEstimator
{
public:
    DepthEstimator();


    DepthEstimator( cv::Mat & mRefImage,
                    Sophus::SE3 & mRefPose_wc,
                    double nFx,
                    double nFy,
                    double nCx,
                    double nCy,
                    double nD1,
                    double nD2,
                    double nD3, 
                    double nD4);

    void FuseNewFrame(  cv::Mat & mMatchImage,
                        Sophus::SE3 & mMatchPose_wc);

    void AddNewFrame(   cv::Mat & mMatchImage,
                        Sophus::SE3 & mMatchPose_wc);


    void FinalizeMapPoints(
        vector<cv::Point3d> & gMapPoints,
        vector<cv::Point3d> & gMapPointsColor);

    void PlotDepth();



    void Initialize(cv::Mat & mRefImage,
                    Sophus::SE3 & mRefPose_wc,
                    double nFx,
                    double nFy,
                    double nCx,
                    double nCy,      
                    double nD1,
                    double nD2,
                    double nD3, 
                    double nD4);



    //Use the average photometric error to check the validity of inverse depth.   
    bool Validate();

    //Post process, smooth the depth map.
    //This is still not satisfactory now.
    bool RegularizeDepthMap();

    void LoadMaps(
        cv::Mat & mInvDepthMap,
        cv::Mat & mInvCovMap,
        cv::Mat & mConvergeMap){
        mInvDepthMap = this->m_mInvDepthMap;
        mInvCovMap = this->m_mInvCovMap;
        mConvergeMap = this->m_mCoverageMap;
    }

    bool ReleaseSpaces(){       
        this->m_mGradientMap.release();
        this->m_mGradientMap_x.release();
        this->m_mGradientMap_y.release();
    }

    vector<double> m_gSparseDepth;
    vector<cv::Point2d> m_gSparsePoints2D;

    void BindSparsePoints(
        vector<double> & gSparseDepth, 
        vector<cv::Point2d> & gSparsePoints){
        this->m_gSparseDepth = gSparseDepth;
        this->m_gSparsePoints2D = gSparsePoints;
        this->m_pSGMMapper->BindSparsePoints(gSparseDepth, gSparsePoints);
    }
    

    bool PropogateFromPreviousFrame(
        DepthEstimator * pPreviousEstimator,
        Sophus::SE3 mRelativePose_co);


private:




    bool UpdateDepthFilter(
            const Eigen::Vector2d& mRefPoint,  
            const double nEstimatedDepth,
            const Sophus::SE3& mT_cr,    //curr ref
            cv::Mat& mDepthMap, 
            cv::Mat& mDepthCovMap,
            bool bFirst);


    void FuseNewFrameSGM(  cv::Mat & mMatchImage,
                        Sophus::SE3 & mMatchPose_wc);


    DepthFilter * m_pDepthFilter;

    StereoMapper * m_pSGMMapper;

    //The inverse depth map is this image's
    cv::Mat m_mRefImage;
    vector<cv::Mat> m_gMatchImages;

    Sophus::SE3 m_mRefPose_wc;
    vector<Sophus::SE3> m_gMatchPoses;

    //The inverse depth map.
    cv::Mat m_mInvDepthMap;
    //The map of the covariance of inverse depth.
    cv::Mat m_mInvCovMap;
    //The gradient in two vertical directions and the gradient in sum.
    cv::Mat m_mGradientMap_x, m_mGradientMap_y, m_mGradientMap;

    cv::Mat m_mObservationMap;

    //Which pixels' depth are coverage.
    //0 not selected
    //1 not coverage
    //2 coverage
    //3 bad
    cv::Mat m_mCoverageMap;
    double m_nCoverageCovThres; //Default 0.01
    double m_nNotCoverageCovThres; //Default 50;


    //Intrinsics.
    double m_nFx;
    double m_nFy;
    double m_nCx;
    double m_nCy;
    Eigen::Matrix3d m_mK;
    //The boundary of the image won't be computed.
    int m_nBorder;
    int m_nObservations;

    void SelectPixels(){
        double nGradientThres = 0;
        int nWidth = this->m_mRefImage.cols;
        int nHeight= this->m_mRefImage.rows;

        for (int u=m_nBorder;u<nWidth-m_nBorder;u++){
            for (int v=m_nBorder;v<nHeight-m_nBorder;v++){
                if (this->m_mGradientMap.at<double>(v , u) >= nGradientThres){
                    this->m_mCoverageMap.at<uchar>(v , u) = 1;
                }
            }
        }
    }


    inline Eigen::Vector3d LiftProject(Eigen::Vector2d mPixelPoint){
        Eigen::Matrix3d mK = this->m_mK;
        Eigen::Vector3d mPixelHomo( mPixelPoint(0),
                                    mPixelPoint(1),
                                    1.0);
        Eigen::Vector3d mNormalizedPoint = mK.inverse() * mPixelHomo;
        mNormalizedPoint /= mNormalizedPoint(2);
        return mNormalizedPoint;
    }

    inline Eigen::Vector2d Project(Eigen::Vector3d mPoint){
        Eigen::Matrix3d mK = this->m_mK;

        mPoint /= mPoint(2);
        Eigen::Vector3d mPixelPointHomo = mK * mPoint;


        Eigen::Vector2d mPixel( mPixelPointHomo(0)/mPixelPointHomo(2),
                                mPixelPointHomo(1)/mPixelPointHomo(2));

        return mPixel;
    }

    //Check if the pixel is in valid region of the img
    inline bool CheckInside(Eigen::Vector2d mPos,
                        double nBorder,
                        int nImageWidth,
                        int nImageHeight){
    if (mPos(0) < nBorder || mPos(0) > nImageWidth - nBorder ||
        mPos(1) < nBorder || mPos(1) > nImageHeight - nBorder){
        return false;
    }
    return true;
}

    // Interpolate, used for the grayscale image.
    inline double GetBilinearInterpolatedValue( const cv::Mat& img, const Eigen::Vector2d& pt ) {
        uchar* d = & img.data[ int(pt(1,0))*img.step+int(pt(0,0)) ];
        double xx = pt(0,0) - floor(pt(0,0)); 
        double yy = pt(1,0) - floor(pt(1,0));
        return  (( 1-xx ) * ( 1-yy ) * double(d[0]) +
                xx* ( 1-yy ) * double(d[1]) +
                ( 1-xx ) *yy* double(d[img.step]) +
                xx*yy*double(d[img.step+1]));
    }

    //Interpolate, used for the gradient map.
    inline double GetDoubleBilinearInterpolatedValue( const cv::Mat& img, const Eigen::Vector2d& pt ) {
        double d = img.at<double>(pt(1), pt(0));
        double d2 = img.at<double>(pt(1), pt(0)+1);
        double d3 = img.at<double>(pt(1)+1, pt(0));
        double d4 = img.at<double>(pt(1)+1, pt(0)+1);

        double xx = pt(0,0) - floor(pt(0,0)); 
        double yy = pt(1,0) - floor(pt(1,0));
        return  (( 1-xx ) * ( 1-yy ) * d +
                xx* ( 1-yy ) * d2 +
                ( 1-xx ) *yy* d3 +
                xx*yy*d4);
    }




};

#endif