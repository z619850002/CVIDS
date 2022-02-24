#include "../../include/dense_mapping/sgm_stereo_mapper.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <sstream>
#include <fstream>
using namespace std;


StereoMapper::StereoMapper(int nRealWidth, int nRealHeight)
    : m_mPhotometricCost(1, HEIGHT * ALIGN_WIDTH * DEP_CNT, CV_32F),
      m_mSgmCost(1, HEIGHT * ALIGN_WIDTH * DEP_CNT, CV_32F),
      m_mDepthMap(HEIGHT, WIDTH, CV_32F)
{
    this->m_nRealHeight = nRealHeight;
    this->m_nRealWidth = nRealWidth;
}

void StereoMapper::InitIntrinsic(   const cv::Mat &mK1, 
                                    const cv::Mat &mD1, 
                                    const cv::Mat &mK2, 
                                    const cv::Mat &mD2)
{

    this->m_mK1 = mK1.clone();
    this->m_mK2 = mK2.clone();
    this->m_mD1 = mD1.clone();
    this->m_mD2 = mD2.clone();

    double nScaleX = (double)this->m_nRealWidth/(double)WIDTH;
    double nScaleY = (double)this->m_nRealHeight/(double)HEIGHT;

// #if DOWNSAMPLE
    this->m_mK1.at<double>(0 , 0) = this->m_mK1.at<double>(0 , 0)/nScaleX;
    this->m_mK1.at<double>(0 , 2) = this->m_mK1.at<double>(0 , 2)/nScaleX;

    this->m_mK1.at<double>(1 , 1) = this->m_mK1.at<double>(1 , 1)/nScaleY;
    this->m_mK1.at<double>(1 , 2) = this->m_mK1.at<double>(1 , 2)/nScaleY;

    this->m_mK2.at<double>(0 , 0) = this->m_mK2.at<double>(0 , 0)/nScaleX;
    this->m_mK2.at<double>(0 , 2) = this->m_mK2.at<double>(0 , 2)/nScaleX;

    this->m_mK2.at<double>(1 , 1) = this->m_mK2.at<double>(1 , 1)/nScaleY;
    this->m_mK2.at<double>(1 , 2) = this->m_mK2.at<double>(1 , 2)/nScaleY;

    // this->m_mK1 /= 2;
    // this->m_mK1.at<double>(2, 2) = 1;
    // this->m_mK2 /= 2;
    // this->m_mK2.at<double>(2, 2) = 1;
// #endif

}

void StereoMapper::InitReference(const cv::Mat &mRefImage)
{

    cv::Mat mUndistortedImage;
    int nWidth = mRefImage.cols;
    int nHeight = mRefImage.rows;


    cv::Mat mDownSampledRefImage = mRefImage.clone();

// #if DOWNSAMPLE
    cv::resize(mRefImage, mDownSampledRefImage, cv::Size(WIDTH, HEIGHT));
// #endif
    
    cv::undistort(mDownSampledRefImage, mUndistortedImage, this->m_mK1, this->m_mD1, this->m_mK1);
    mUndistortedImage.convertTo(mUndistortedImage, CV_32F);

    cv::Mat mGrayImage = mUndistortedImage.clone();
    mGrayImage.convertTo(mGrayImage, CV_64FC1);
    cv::Mat mGradient;
    cv::Sobel(mGrayImage, mGradient, CV_64FC1, 5, 5, 9);

    mGradient = cv::abs(mGradient);
    double nMeanValue = cv::mean(mGradient)[0];
    cv::pow(mGradient, 3   , mGradient);

    mGradient = 0.8 + 1.5 * nMeanValue * nMeanValue * nMeanValue/(1+mGradient);

    mGradient.convertTo(mGradient, CV_32F);

    // ShowColor(mGradient);
    this->m_mGradient.upload(mGradient);



    cv::Scalar     iMean;  
    cv::Scalar     iDev; 
    cv::Sobel(mGrayImage, this->m_mGradientX, CV_64FC1, 3, 0, 7);
    cv::meanStdDev(this->m_mGradientX , iMean , iDev);
    double nMeanGradientX = iMean.val[0];
    double nDevGradientX = iDev.val[0];
    for (int u=0;u<this->m_mGradientX.cols;u++){
        for (int v=0;v<this->m_mGradientX.rows;v++){
            if (this->m_mGradientX.at<double>(v , u) < nMeanGradientX+nDevGradientX){
                this->m_mGradientX.at<double>(v , u) = 0.0;
            }
        }
    }

    cv::Sobel(mGrayImage, this->m_mGradientY, CV_64FC1, 0, 3, 7);    
    cv::meanStdDev(this->m_mGradientY , iMean , iDev);
    double nMeanGradientY = iMean.val[0];
    double nDevGradientY = iDev.val[0];
    for (int u=0;u<this->m_mGradientY.cols;u++){
        for (int v=0;v<this->m_mGradientY.rows;v++){
            if (this->m_mGradientY.at<double>(v , u) < nMeanGradientY+nDevGradientY){
                this->m_mGradientY.at<double>(v , u) = 0.0;
            }
        }
    }


    this->m_nMean1 = cv::mean(mUndistortedImage)[0];

    this->m_mRefImage.upload(mUndistortedImage);
    
    this->m_nMeasurementCount = 0;

}

void StereoMapper::Update(  const cv::Mat &mMatchImage, 
                            const cv::Mat &mRefRotation, 
                            const cv::Mat &mRefTranslation, 
                            const cv::Mat &mMatchRotation, 
                            const cv::Mat &mMatchTranslation)
{
    

    // string aBaseString =  "/home/kyrie/Documents/DataSet/CoVins/Stereo/";
    // string aIndex = "";
    // stringstream sss;
    // sss << nStereoNumber;
    // sss >> aIndex;
    // cv::imwrite(aBaseString + aIndex + "_match.jpg", mMatchImage);
    // ofstream fOutFile(aBaseString + aIndex + "_pose.txt");
    // fOutFile << "RefRotation: " << endl << mRefRotation << endl;
    // fOutFile << "mRefTranslation: " << endl << mRefTranslation << endl;
    // fOutFile << "MatchRotation: " << endl << mMatchRotation << endl;
    // fOutFile << "mMatchTranslation: " << endl << mMatchTranslation << endl;





    int64 nStartClock = cv::getTickCount();

    cout << "Start to update: " << endl;
    
    this->m_nMeasurementCount++;

    cv::Mat mUndistortedImage;
   
    int nWidth = mMatchImage.cols;
    int nHeight = mMatchImage.rows;
    cv::Mat mDownSampledMatchedImage = mMatchImage.clone();

// #if DOWNSAMPLE
    cv::resize(mMatchImage, mDownSampledMatchedImage, cv::Size(WIDTH, HEIGHT));
// #endif
    



    cv::undistort(mDownSampledMatchedImage, mUndistortedImage, this->m_mK2, this->m_mD2, this->m_mK2);

    mUndistortedImage.convertTo(mUndistortedImage, CV_32F);


    this->m_nMean2 = cv::mean(mUndistortedImage)[0];

    this->m_mMatchImage.upload(mUndistortedImage);
    // img_r = raw_img_r.clone();


    this->m_mR = this->m_mK2 * mMatchRotation.t() * mRefRotation * this->m_mK1.inv();
    this->m_mT = this->m_mK2 * mMatchRotation.t() * (mRefTranslation - mMatchTranslation);
    
    this->m_mT = this->m_mT.t();

    ad_calc_cost(
        this->m_nMeasurementCount,
        this->m_mR.at<double>(0, 0), this->m_mR.at<double>(0, 1), this->m_mR.at<double>(0, 2),
        this->m_mR.at<double>(1, 0), this->m_mR.at<double>(1, 1), this->m_mR.at<double>(1, 2),
        this->m_mR.at<double>(2, 0), this->m_mR.at<double>(2, 1), this->m_mR.at<double>(2, 2),
        this->m_mT.at<double>(0, 0), this->m_mT.at<double>(0, 1), this->m_mT.at<double>(0, 2),
        this->m_mK1.at<double>(0, 0), this->m_mK1.at<double>(1, 1), 
        this->m_mK1.at<double>(0, 2), this->m_mK1.at<double>(1, 2),
        this->m_mRefImage.data, this->m_mRefImage.step,
        this->m_mMatchImage.data, this->m_mMatchImage.step,
        this->m_mPhotometricCost.data,
        0.0);

    printf("Cost 1 is: %fms\n", (cv::getTickCount() - nStartClock)/ cv::getTickFrequency() * 1000);

}


void StereoMapper::ClearRawCost(){


    // this->m_mPhotometricCost.release();
    // this->m_mSgmCost.release();
    // this->m_mDepthMap.release();

    cout << "Clear1" << endl;
    this->m_mPhotometricCost.setTo(cv::Scalar_<float>(0.0));
    cout << "Clear2" << endl;
    this->m_mSgmCost.setTo(cv::Scalar_<float>(0.0));
    cout << "Clear3" << endl;
    this->m_mDepthMap.setTo(cv::Scalar_<float>(0.0));
    cout << "Clear4" << endl;
}


cv::Mat StereoMapper::Output()
{


    int64 nStartClock = cv::getTickCount();


    double nScaleX = (double)this->m_nRealHeight/(double)HEIGHT;
    double nScaleY = (double)this->m_nRealWidth/(double)WIDTH;

    cv::Mat mSparseDepth = cv::Mat(HEIGHT, WIDTH, CV_32F, -1.0);
    cv::Mat mSparseDistance = cv::Mat(HEIGHT, WIDTH, CV_32F, 0.0);
    for (int i=0;i<this->m_gSparseDepth.size();i++){
        double nDepth = this->m_gSparseDepth[i];
        int nX = this->m_gSparsePoints2D[i].x;
        int nY = this->m_gSparsePoints2D[i].y;

        // #if DOWNSAMPLE
        nX /=nScaleX;
        nY /=nScaleY;
        // #endif
        int nWindowSize = 4;


        vector<int> gUpperBound, gBottomBound, gLeftBound, gRightBound;
        gUpperBound.reserve(nWindowSize*2+1);
        gBottomBound.reserve(nWindowSize*2+1);
        gLeftBound.reserve(nWindowSize*2+1);
        gRightBound.reserve(nWindowSize*2+1);

        for (int i=0;i<nWindowSize*2+1;i++){
            gUpperBound.push_back(nWindowSize);
            gBottomBound.push_back(-nWindowSize);
            gLeftBound.push_back(-nWindowSize);
            gRightBound.push_back(nWindowSize);
        } 


        for (int us=-nWindowSize;us<=nWindowSize;us++){
            for (int vs=-nWindowSize; vs<=nWindowSize;vs++){
                
                if (this->m_mGradientX.at<double>(nY+vs, nX+us) >0.0){
                    
                    if (gLeftBound[vs+nWindowSize] < us && us <0){
                        gLeftBound[vs+nWindowSize] = us;
                    }

                    if (gRightBound[vs+nWindowSize] > us && us >0){
                        gRightBound[vs+nWindowSize] = us;
                    }
                }


                if (this->m_mGradientY.at<double>(nY+vs, nX+us) >0.0){
                    if (gUpperBound[us+nWindowSize] > vs && vs >0){
                        gUpperBound[us+nWindowSize] = vs;
                    }

                    if (gBottomBound[us+nWindowSize] < vs && vs <0){
                        gBottomBound[us+nWindowSize] = vs;
                    }
                }


            }
        }

        for (int us=1;us<=nWindowSize;us++){
            int nPosIndex = us + nWindowSize;
            int nNegIndex = -us+nWindowSize;
            //Upper
            if (gUpperBound[nPosIndex] > gUpperBound[nPosIndex-1]){
                gUpperBound[nPosIndex] = gUpperBound[nPosIndex-1];
            }


            if (gUpperBound[nNegIndex] > gUpperBound[nNegIndex+1]){
                gUpperBound[nNegIndex] = gUpperBound[nNegIndex+1];
            }

            //Bottom
            if (gBottomBound[nPosIndex] < gBottomBound[nPosIndex-1]){
                gBottomBound[nPosIndex] = gBottomBound[nPosIndex-1];
            }

            if (gBottomBound[nNegIndex] < gBottomBound[nNegIndex+1]){
                gBottomBound[nNegIndex] = gBottomBound[nNegIndex+1];
            }

            //Left

            if (gLeftBound[nPosIndex] < gLeftBound[nPosIndex-1]){
                gLeftBound[nPosIndex] = gLeftBound[nPosIndex-1];
            }

            if (gLeftBound[nNegIndex] < gLeftBound[nNegIndex+1]){
                gLeftBound[nNegIndex] = gLeftBound[nNegIndex+1];
            }

            //Right
            if (gRightBound[nPosIndex] > gRightBound[nPosIndex-1]){
                gRightBound[nPosIndex] = gRightBound[nPosIndex-1];
            }


            if (gRightBound[nNegIndex] > gRightBound[nNegIndex+1]){
                gRightBound[nNegIndex] = gRightBound[nNegIndex+1];
            }

        }






        for (int u=-nWindowSize;u<=nWindowSize;u++){
            for (int v=-nWindowSize;v<=nWindowSize;v++){
                if ((int)nY+v >= HEIGHT-1 || (int)nY+v < 1 || (int)nX+u >= WIDTH-1 || (int)nX+u < 1){
                    continue;
                }

                if (    u >= gLeftBound[v+nWindowSize] && 
                        u <= gRightBound[v+nWindowSize] && 
                        v >= gBottomBound[u+nWindowSize] &&
                        v <= gUpperBound[u+nWindowSize]){
                    
                    double nDistRatio = (1.0-(sqrt(u*u+v*v)/(nWindowSize*1.414)));
                    nDistRatio  = nDistRatio *nDistRatio;
                    if (mSparseDistance.at<float>((int)nY+v, (int)nX+u) < nDistRatio){
                        mSparseDepth.at<float>((int)nY+v, (int)nX+u) = nDepth;
                        mSparseDistance.at<float>((int)nY+v, (int)nX+u) = nDistRatio * nDistRatio;   
                    }       

                }         
            }
        }

    }

    this->m_mSparseDepth.upload(mSparseDepth);
    this->m_mSparseDistance.upload(mSparseDistance);




    cout << "Begin to fuse: " << endl;
    FuseSparseInfo(this->m_mSparseDepth.data, this->m_mSparseDepth.step,
                    this->m_mSparseDistance.data, this->m_mSparseDistance.step, 
                    this->m_mPhotometricCost.data);


    this->m_mSgmCost.setTo(cv::Scalar_<float>(0.0));

    sgm2(this->m_mRefImage.data, this->m_mRefImage.step,
         this->m_mMatchImage.data, this->m_mMatchImage.step,
         this->m_mGradient.data, this->m_mGradient.step,
         this->m_mPhotometricCost.data,
         this->m_mSgmCost.data);


    filter_cost(this->m_mSgmCost.data,
                this->m_mDepthMap.data, 
                this->m_mDepthMap.step);
  
    cv::Mat mResult;
    this->m_mDepthMap.download(mResult);



    // string aBaseString =  "/home/kyrie/Documents/DataSet/CoVins/Stereo/";
    // string aIndex = "";
    // stringstream sss;
    // sss << nStereoNumber;
    // sss >> aIndex;
    // cv::Mat mDepthMapWrite = mResult * 0.1*255;
    // cv::imwrite(aBaseString + aIndex + "_depth.jpg", mDepthMapWrite);



    int nWidth = mResult.cols;
    int nHeight = mResult.rows;


    // cv::Mat mWrite = mResult*0.1*255;
    // cv::imwrite("/home/kyrie/Documents/DataSet/CoVins/Depth2.jpg", mWrite);



// #if DOWNSAMPLE
    cv::resize(mResult, mResult, cv::Size(this->m_nRealWidth, this->m_nRealHeight));
// #endif

    // cv::Mat mDepthMapWrite2 = mResult * 0.1*255.0;
    // cv::imwrite(aBaseString + aIndex + "_depth2.jpg", mDepthMapWrite2);

    // nStereoNumber++;



    printf("Cost 2 is: %fms\n", (cv::getTickCount() - nStartClock)/ cv::getTickFrequency() * 1000);

    return mResult;
}



void StereoMapper::ReleaseSpaces(){

    this->m_mRefImage.release();
    this->m_mMatchImage.release();



    this->m_mPhotometricCost.release();
    this->m_mSgmCost.release();
    this->m_mDepthMap.release();




    this->m_mK1.release();
    this->m_mK2.release();
    this->m_mD1.release();
    this->m_mD2.release();

    this->m_mSparseDepth.release();

}


