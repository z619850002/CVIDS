#include "../../include/dense_mapping/depth_estimator.h"


using namespace std; 


using Sophus::SE3;
using namespace Eigen;
using namespace cv;






// bool DepthEstimator::UpdateDepthFilter(
//     const Vector2d& pt_ref,  
//     const double nDepth,
//     const SE3& T_C_R,
//     Mat& depth, 
//     Mat& depth_cov,
//     bool bFirst)
// {
//     // 我是一只喵
//     // 不知道这段还有没有人看
//     // 用三角化计算深度
//     SE3 T_R_C = T_C_R.inverse();
//     Vector3d f_ref = this->LiftProject( pt_ref );
//     f_ref.normalize();
    
    
//     // 方程
//     // d_ref * f_ref = d_cur * ( R_RC * f_cur ) + t_RC
//     // => [ f_ref^T f_ref, -f_ref^T f_cur ] [d_ref] = [f_ref^T t]
//     //    [ f_cur^T f_ref, -f_cur^T f_cur ] [d_cur] = [f_cur^T t]
//     // 二阶方程用克莱默法则求解并解之
//     Vector3d t = T_R_C.translation();
    
//     double depth_estimation = nDepth;   // 深度值
//     // cout << "Depth estimation2 is: " << depth_estimation << endl;
    
//     // 计算不确定性（以一个像素为误差）
//     Vector3d p = f_ref*depth_estimation;
//     Vector3d a = p - t; 
//     double t_norm = t.norm();
//     double a_norm = a.norm();
//     double alpha = acos( f_ref.dot(t)/t_norm );
//     double beta = acos( -a.dot(t)/(a_norm*t_norm));
//     double beta_prime = beta + atan(1/this->m_nFx);
//     double gamma = M_PI - alpha - beta_prime;
//     double p_prime = t_norm * sin(beta_prime) / sin(gamma);
//     double d_cov = p_prime - depth_estimation; 
//     double d_cov2 = d_cov*d_cov;
    
//     // 高斯融合
//     double mu = depth.ptr<double>( int(pt_ref(1,0)) )[ int(pt_ref(0,0)) ];
//     double sigma2 = depth_cov.ptr<double>( int(pt_ref(1,0)) )[ int(pt_ref(0,0)) ];

//     double mu_fuse, sigma_fuse2;

//     if (bFirst){
//         mu_fuse = depth_estimation;
//         sigma_fuse2 = d_cov2;
//     }else{
//         mu_fuse = (d_cov2*mu+sigma2*depth_estimation) / ( sigma2+d_cov2);
//         sigma_fuse2 = ( sigma2 * d_cov2 ) / ( sigma2 + d_cov2 );
//     }


//     depth.ptr<double>( int(pt_ref(1,0)) )[ int(pt_ref(0,0)) ] = mu_fuse; 
//     depth_cov.ptr<double>( int(pt_ref(1,0)) )[ int(pt_ref(0,0)) ] = sigma_fuse2;
    
//     return true;
// }



inline double SmoothKernel(double nCov2){
    if (1/nCov2 > 500){
        nCov2 = 1.0/(sqrt(1.0/nCov2-500)+500);
    }
    return nCov2;
}

bool DepthEstimator::UpdateDepthFilter(
    const Vector2d& pt_ref,  
    const double nInvDepth,
    const SE3& T_C_R,
    Mat& mInvDepth, 
    Mat& mInvDepthCov,
    bool bFirst)
{





    // 我是一只喵
    // 不知道这段还有没有人看
    // 用三角化计算深度
    SE3 T_R_C = T_C_R.inverse();
    Vector3d f_ref = this->LiftProject( pt_ref );
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
    double beta_prime = beta + atan(1/this->m_nFx);
    double gamma = M_PI - alpha - beta_prime;
    double p_prime = t_norm * sin(beta_prime) / sin(gamma);
    //Compute the covariance
    double d_cov = 1.0/p_prime - nInvDepth;

    d_cov = abs(d_cov) + DEP_SAMPLE; 

    double d_cov2 = d_cov*d_cov;

    d_cov2 = SmoothKernel(d_cov2);

    
    // 高斯融合
    double mu = mInvDepth.ptr<double>( int(pt_ref(1,0)) )[ int(pt_ref(0,0)) ];
    double sigma2 = mInvDepthCov.ptr<double>( int(pt_ref(1,0)) )[ int(pt_ref(0,0)) ];

    double mu_fuse, sigma_fuse2;

    if (bFirst){
        mu_fuse = nInvDepth;
        sigma_fuse2 = d_cov2;
    }else{
        mu_fuse = (d_cov2*mu+sigma2*nInvDepth) / ( sigma2+d_cov2);
        sigma_fuse2 = ( sigma2 * d_cov2 ) / ( sigma2 + d_cov2 );
    }


    mInvDepth.ptr<double>( int(pt_ref(1,0)) )[ int(pt_ref(0,0)) ] = mu_fuse; 
    mInvDepthCov.ptr<double>( int(pt_ref(1,0)) )[ int(pt_ref(0,0)) ] = sigma_fuse2;
    
    return true;
}


int nEstimatorIndex = 0;

DepthEstimator::DepthEstimator(){
    nEstimatorIndex++;
}


DepthEstimator::DepthEstimator( 
    cv::Mat & mRefImage,
    Sophus::SE3 & mRefPose_wc,
    double nFx,
    double nFy,
    double nCx,
    double nCy,
    double nD1,
    double nD2,
    double nD3, 
    double nD4){
    nEstimatorIndex++;
    this->m_nObservations = 0;

    cout << "Begin to initialize" << endl;
    this->Initialize(mRefImage, mRefPose_wc, nFx, nFy, nCx, nCy, nD1, nD2, nD3, nD4);  

    cout << "Finish to initialize" << endl;

    cout << "Create Depth Filter" << endl;
    this->m_pDepthFilter = new DepthFilter(mRefImage.rows, mRefImage.cols);
    cout << "Finish Depth Filter" << endl;


}

void DepthEstimator::FuseNewFrame(  
    cv::Mat & mMatchImage,
    Sophus::SE3 & mMatchPose_wc){
    
    this->FuseNewFrameSGM(mMatchImage, mMatchPose_wc);
    this->m_pSGMMapper->ClearRawCost();
}



void DepthEstimator::AddNewFrame(   cv::Mat & mMatchImage,
                        Sophus::SE3 & mMatchPose_wc){

    //Add the frame to the list.
    this->m_gMatchImages.push_back(mMatchImage);
    this->m_gMatchPoses.push_back(mMatchPose_wc);
}


void DepthEstimator::FuseNewFrameSGM(  
    cv::Mat & mMatchImage,
    Sophus::SE3 & mMatchPose_wc){

    int64 nStartClock = cv::getTickCount();

    //Add the frame to the list.
    this->m_gMatchImages.push_back(mMatchImage);
    this->m_gMatchPoses.push_back(mMatchPose_wc);

    Eigen::Matrix3d mRefRotation_wc = this->m_mRefPose_wc.rotation_matrix(); 
    Eigen::Matrix3d mCurrRotation_wc = mMatchPose_wc.rotation_matrix();
    Eigen::Vector3d mRefTranslation_wc = this->m_mRefPose_wc.translation();
    Eigen::Vector3d mCurrTranslation_wc = mMatchPose_wc.translation();

    cv::Mat mRefCVRotation_wc, mRefCVTranslation_wc;
    cv::Mat mCurrCVRotation_wc, mCurrCVTranslation_wc;

    cv::eigen2cv(mRefRotation_wc, mRefCVRotation_wc);
    cv::eigen2cv(mRefTranslation_wc, mRefCVTranslation_wc);
    cv::eigen2cv(mCurrRotation_wc, mCurrCVRotation_wc);
    cv::eigen2cv(mCurrTranslation_wc, mCurrCVTranslation_wc);


    printf("Cost first is: %fms\n", (cv::getTickCount() - nStartClock)/ cv::getTickFrequency() * 1000);



    this->m_pSGMMapper->Update(
        mMatchImage, 
        mRefCVRotation_wc, 
        mRefCVTranslation_wc,
        mCurrCVRotation_wc, 
        mCurrCVTranslation_wc);


    printf("Cost second is: %fms\n", (cv::getTickCount() - nStartClock)/ cv::getTickFrequency() * 1000);



    int nWidth = this->m_mRefImage.cols;
    int nHeight= this->m_mRefImage.rows;

    cv::Mat mRawResultMap = this->m_pSGMMapper->Output();



    // cv::Mat mRawWrite = mRawResultMap * 0.1 * 255.0;
    // stringstream ss;
    // ss << nStereoNumber;
    // string aIndex = "";
    // ss >> aIndex;
    // stringstream ss2;
    // ss2 << nEstimatorIndex;
    // string aEstimatorIndex;
    // ss2 >> aEstimatorIndex;
    // cv::imwrite("/home/kyrie/Documents/DataSet/CoVins/Mean/single/" + aEstimatorIndex + "_" + aIndex + ".jpg",  mRawWrite);




    printf("Cost third is: %fms\n", (cv::getTickCount() - nStartClock)/ cv::getTickFrequency() * 1000);



    cv::Mat mResultMap;
    // cv::bilateralFilter(mRawResultMap, mResultMap, 16/2, 32/2, 8/2);
    // mRawResultMap.release();

    // mResultMap.convertTo(mResultMap, CV_32F);
    // cv::bilateralFilter(mResultMap, mResultMap, 25, 25*2, 25.0/2);
    mRawResultMap.convertTo(mResultMap, CV_64FC1);

    

    // Sophus::SE3 mRelativePose_cr = mMatchPose_wc.inverse() * this->m_mRefPose_wc;

    mResultMap = 1.0/mResultMap;


    


    cv::Mat mCovMap = cv::Mat(nHeight, nWidth, CV_64F, (3 * DEP_SAMPLE) * (3 * DEP_SAMPLE));
    printf("Cost forth is: %fms\n", (cv::getTickCount() - nStartClock)/ cv::getTickFrequency() * 1000);


    this->m_pDepthFilter->Update(mResultMap, mCovMap);


    printf("Cost fifth is: %fms\n", (cv::getTickCount() - nStartClock)/ cv::getTickFrequency() * 1000);


    this->m_mInvDepthMap = this->m_pDepthFilter->GetInvDepth();




    // // delete this->m_pSGMMapper;
    // for (int x=0;x<nWidth;x++){
    //     for (int y=0;y<nHeight;y++){
    //         int nObservations = this->m_mObservationMap.at<uchar>(y , x);
    //         double nEstimatedDepth = mResultMap.at<double>(y , x);
    //         if (nEstimatedDepth <=0.01){
    //             continue;                
    //         }
    //         if (nEstimatedDepth >=100){
    //             continue;
    //         }
    //         bool bFirst = false;
    //         // if (nObservations ==0)
    //         // {
    //         //     bFirst = true;
    //         // }

    //         // if (x>10 && x<nWidth-10 && y>10 && y<nHeight-10){
    //         //     double nTop = nEstimatedDepth - mResultMap.at<double>(y-10,x);
    //         //     double nBottom = nEstimatedDepth - mResultMap.at<double>(y+10,x);
    //         //     double nLeft = nEstimatedDepth - mResultMap.at<double>(y,x-10);
    //         //     double nRight = nEstimatedDepth - mResultMap.at<double>(y,x+10);
    //         //     if (nTop * nBottom >0 && abs(nTop) > 3 && abs(nBottom) >3){
    //         //         nEstimatedDepth = (mResultMap.at<double>(y-10,x) + mResultMap.at<double>(y+10,x))/2;    
    //         //     }
    //         //     if (nLeft * nRight >0 && abs(nLeft) > 3 && abs(nRight) >3){
    //         //         nEstimatedDepth = (mResultMap.at<double>(y,x-10) + mResultMap.at<double>(y,x+10))/2;
    //         //     }
    //         // }

    //         Eigen::Vector2d mRefPoint(x , y);
    //         this->UpdateDepthFilter(
    //             mRefPoint, 
    //             1.0/nEstimatedDepth, 
    //             mRelativePose_cr, 
    //             this->m_mInvDepthMap, 
    //             this->m_mInvCovMap, 
    //             bFirst);
    //         this->m_mObservationMap.at<uchar>(y , x) = nObservations+1;
    //         // this->m_mInvCovMap.at<double>(y , x) = 0.0001;
    //         if (this->m_mCoverageMap.at<uchar>(y, x)!=0){       
    //             this->m_mCoverageMap.at<uchar>(y, x) = 2;   
    //         }
    //     }
    // }
    // 
    cout << "Finish fusion" << endl;
    this->m_nObservations++;
    mResultMap.release();


    printf("Cost all is: %fms\n", (cv::getTickCount() - nStartClock)/ cv::getTickFrequency() * 1000);

}



void DepthEstimator::FinalizeMapPoints(
    vector<cv::Point3d> & gMapPoints,
    vector<cv::Point3d> & gMapPointsColor){

    // this->m_mInvDepthMap = 1.0/this->m_mInvDepthMap;

    int nWidth = this->m_mRefImage.cols;
    int nHeight= this->m_mRefImage.rows;

    // for (int x=0;x<nWidth;x++){
    //     for (int y=0;y<nHeight;y++){
    //         if ( 
    //             this->m_mInvCovMap.at<double>(y , x) > 0.005){
    //             this->m_mInvDepthMap.at<double>(y , x) = 0.001;
    //         }
    //     }
    // }





    cv::Mat mRatio = this->m_pDepthFilter->GetRatio();
    this->m_mInvDepthMap = this->m_pDepthFilter->GetInvDepth();




    for (int u=0;u<nWidth;u++){
        for (int v=0;v<nHeight;v++){
            if (mRatio.at<double>(v , u) <0.5){
                this->m_mInvDepthMap.at<double>(v , u) = 0.00001;
            }
        }
    }


    // cv::Scalar     iMean;  
    // cv::Scalar     iDev; 
    // cv::meanStdDev(this->m_mGradientMap , iMean , iDev);
    // double nMeanGradient = iMean.val[0];
    // double nDevGradient = iDev.val[0];
    // for (int u=0;u<nWidth;u++){
    //     for (int v=0;v<nHeight;v++){
    //         if (this->m_mGradientMap.at<double>(v , u) < nMeanGradient){
    //             this->m_mInvDepthMap.at<double>(v , u) = 0.00001;
    //         }
    //     }
    // }


    // cv::Mat mDepthMap =  255 * 0.1/m_mInvDepthMap;
    // stringstream ss;
    // ss << nStereoNumber;
    // string aIndex = "";
    // ss >> aIndex;
    // stringstream ss2;
    // ss2 << nEstimatorIndex;
    // string aEstimatorIndex;
    // ss2 >> aEstimatorIndex;
    // cv::imwrite("/home/kyrie/Documents/DataSet/CoVins/Mean/single/" + aEstimatorIndex + "_" + aIndex + "_final.jpg",  mDepthMap);


    


    //The mappoints should be in the camera cs.

    cv::Mat mColorImage;
    cv::cvtColor(this->m_mRefImage, mColorImage, COLOR_GRAY2BGR);

    // cout << "Begin to validate" << endl;
    // this->Validate();
    // cout << "Finish to validate" << endl;
    // this->RegularizeDepthMap();

    cout << "Finish to regulatization" << endl;

    // gMapPoints.reserve(nHeight * nWidth);

    // for (int u=m_nBorder;u<nWidth-m_nBorder;u++){
    //     for (int v=m_nBorder;v<nHeight-m_nBorder;v++){
            
    //         if (this->m_mCoverageMap.at<uchar>(v , u) != 2){
    //             continue;
    //         }

    //         Eigen::Vector2d mPixel(u , v);
    //         Eigen::Vector3d mCameraPoint = this->LiftProject(mPixel);
    //         double nDepth = 1/this->m_mInvDepthMap.at<double>(v , u);

    //         if (nDepth > 10 || nDepth < 0.1){
    //             continue;
    //         }

    //         mCameraPoint *= nDepth;
    //         Sophus::SE3 mPose_wc_1 = this->m_mRefPose_wc;
    //         // Eigen::Vector3d mWorldPoint = mPose_wc_1 * mCameraPoint;


    //         // cv::Point3d mWorldPointCV(   mWorldPoint(0),
    //         //                             mWorldPoint(1),
    //         //                             mWorldPoint(2));


    //         cv::Point3d iCameraPoint(   mCameraPoint(0),
    //                                     mCameraPoint(1),
    //                                     mCameraPoint(2));
    //         // gMapPoints.push_back(mWorldPointCV);
    //         gMapPoints.push_back(iCameraPoint);

    //         cv::Vec3b iColor = mColorImage.at<cv::Vec3b>(v , u);
    //         gMapPointsColor.push_back(cv::Point3d(
    //                 iColor[0],
    //                 iColor[1],
    //                 iColor[2]));


    //     }
    // }
    mColorImage.release();
    this->m_mRefImage.release();

    this->m_pSGMMapper->ReleaseSpaces();
    delete this->m_pSGMMapper;


}


void DepthEstimator::PlotDepth(){
    cv::imshow( "depth", (1.0/this->m_mInvDepthMap)*0.4 );
    cv::imshow("ref", this->m_mRefImage);
    waitKey(1);
}



void DepthEstimator::Initialize(cv::Mat & mRefImage,
                Sophus::SE3 & mRefPose_wc,
                double nFx,
                double nFy,
                double nCx,
                double nCy,
                double nD1,
                double nD2,
                double nD3, 
                double nD4){
    int nMaxImages = 60;

    this->m_mRefImage = mRefImage;
    this->m_mRefPose_wc = mRefPose_wc;

    this->m_gMatchImages.clear();
    this->m_gMatchPoses.clear();
    // this->m_gMatchImages.reserve(nMaxImages);
    // this->m_gMatchPoses.reserve(nMaxImages);


    //Compute gradient.
    //Must grayscale!
    cv::Mat mGrayImage = mRefImage.clone();
    // mGrayImage.convertTo(mGrayImage, CV_64FC1);
    
    cv::Sobel(mGrayImage, this->m_mGradientMap, CV_64FC1, 3, 3, 7);
    cv::Sobel(mGrayImage, this->m_mGradientMap_x, CV_64FC1, 1, 0, 3);
    cv::Sobel(mGrayImage, this->m_mGradientMap_y, CV_64FC1, 0, 1, 3);
    int nWidth = mRefImage.cols;
    int nHeight= mRefImage.rows;
    //Create the coverage mask.
    this->m_mCoverageMap = cv::Mat(nHeight, nWidth, CV_8UC1, cv::Scalar(1));
    this->m_mObservationMap = cv::Mat(nHeight, nWidth, CV_8UC1, cv::Scalar(0));
    this->m_nBorder = 5;

    // //Cut the border.
    // for (int u=0;u<nWidth;u++){
    //     for (int v=0;v<nHeight;v++){
    //         if (u< this->m_nBorder || u>= nWidth - this->m_nBorder || 
    //             v< this->m_nBorder || v>= nHeight - this->m_nBorder){
    //             //Set bad.
    //             this->m_mCoverageMap.at<uchar>(v , u) = 0;
    //         }
    //     }
    // }


    // this->SelectPixels();

    //Initialize the inverse depth and cov map.
    double nInitialInvDepth   = 0.2;    // initial inverse depth
    double nInitialCov    = 10.0;    // initial cov
    this->m_mInvDepthMap = cv::Mat( nHeight, nWidth, CV_64F, nInitialInvDepth );             // 深度图
    this->m_mInvCovMap = cv::Mat( nHeight, nWidth, CV_64F, nInitialCov );          // 深度图方差 



    //Set the intrinsics.
    this->m_nFx = nFx;
    this->m_nFy = nFy;
    this->m_nCx = nCx;
    this->m_nCy = nCy;

    //Initialize the intrinsic matrix.
    this->m_mK = Eigen::Matrix3d();
    this->m_mK *=0.0;
    this->m_mK(0 , 0) = nFx;
    this->m_mK(1 , 1) = nFy;
    this->m_mK(0 , 2) = nCx;
    this->m_mK(1 , 2) = nCy;
    this->m_mK(2 , 2) = 1.0;

    this->m_nCoverageCovThres = 0.01;
    this->m_nNotCoverageCovThres = 50;


    cout << "Create SGM" << endl;
    //Initialize the SGM mapper
    this->m_pSGMMapper = new StereoMapper(nWidth, nHeight);

    //TODO:Fix the code here
    cv::Mat mK1 = (cv::Mat_<double>(3 , 3) <<   nFx ,  0.0 , nCx,
                                                0.0,  nFy  , nCy,
                                                0.0,  0.0 , 1.0);
    cv::Mat mD1 = (cv::Mat_<double>(5 , 1) << nD1, nD2, nD3, nD4, 0.0);


    cv::Mat mCopyImage = mRefImage.clone();


    cout << "Init intrinsics" << endl;
    this->m_pSGMMapper->InitIntrinsic(mK1, mD1, mK1, mD1);

    cout << "Init reference" << endl;
    this->m_pSGMMapper->InitReference(mCopyImage);

    cout << "Release" << endl;
    mGrayImage.release();


}





bool DepthEstimator::PropogateFromPreviousFrame(
    DepthEstimator * pPreviousEstimator,
    Sophus::SE3 mRelativePose_co){


    cv::Mat mLastMu = pPreviousEstimator->m_pDepthFilter->GetInvDepth();
    cv::Mat mLastRatio = pPreviousEstimator->m_pDepthFilter->GetRatio();
    cv::Mat mLastCov = pPreviousEstimator->m_pDepthFilter->GetCov();
    cv::Mat mLastA = pPreviousEstimator->m_pDepthFilter->GetA();
    cv::Mat mLastB = pPreviousEstimator->m_pDepthFilter->GetB();

    this->m_pDepthFilter->PropogateDepth(
        mLastRatio, 
        mLastMu, 
        mLastCov, 
        mLastA, 
        mLastB, 
        this->m_mK, 
        mRelativePose_co);

    pPreviousEstimator->m_pDepthFilter->ReleaseSpace();
    delete pPreviousEstimator->m_pDepthFilter;


    return true;
}


//Use the average photometric error to check the validity of inverse depth.   
bool DepthEstimator::Validate(){
    //The threshold for validation.        
    const int nBorder = this->m_nBorder;

    const double nThreshold = 2.5;

    cv::Mat mInvDepthMap = this->m_mInvDepthMap;
    cv::Mat mInvCovMap = this->m_mInvCovMap;
    int nDepthWidth = mInvDepthMap.cols, nDepthHeight = mInvDepthMap.rows;
    for (int u=nBorder;u<nDepthWidth-nBorder;u++){
        for (int v=nBorder;v<nDepthHeight-nBorder;v++){
            double nCurrentInvDepth = mInvDepthMap.at<double>(v , u);
            //Check if the point is bad.
            if (this->m_mCoverageMap.at<uchar>(v , u)!=2){
                continue;
            }

            Eigen::Vector2d mPixel(u , v);
            Eigen::Vector3d mPointCamera = this->LiftProject(mPixel);
            double nTotalError = 0.0;
            if (nCurrentInvDepth != nCurrentInvDepth){
                continue;
            }
            mPointCamera *= 1/nCurrentInvDepth;
            int nImageSize = 0;
            //Check the reprojection error.
            for (int i=0;i<this->m_gMatchImages.size();i++){
                cv::Mat mImage = this->m_gMatchImages[i];
                Sophus::SE3 mPose_wc = this->m_gMatchPoses[i];
                Sophus::SE3 mRelativePose = mPose_wc.inverse() * this->m_mRefPose_wc;
                Eigen::Vector3d mPointCamera_2 = mRelativePose * mPointCamera;
                Eigen::Vector2d mProjection = Project(mPointCamera_2);
                double up = mProjection(0), vp = mProjection(1);
                if (up <nBorder || up > nDepthWidth-nBorder ||
                    vp < nBorder || vp > nDepthHeight - nBorder){
                    continue;
                }
                double nRefValue = (double)this->m_mRefImage.at<uchar>(v , u);
                double nProjectValue = (double)mImage.at<uchar>(vp , up);
                nTotalError += abs(nRefValue - nProjectValue);
                nImageSize +=1;
            }
            nTotalError /= (double) (nImageSize);
            
            if (nTotalError>nThreshold){
                //Set bad.
                this->m_mCoverageMap.at<uchar>(v , u) = 3;
            }


        }
    }
}


//Post process, smooth the depth map.
//This is still not satisfactory now.
bool DepthEstimator::RegularizeDepthMap(){

    int nDepthWidth = this->m_mInvDepthMap.cols;
    int nDepthHeight = this->m_mInvDepthMap.rows;
    const int nBorder = this->m_nBorder;
    //The local window to smooth the depth map.
    int nWindowSize = 2;

    cv::Mat mCopyDepthMap = this->m_mInvDepthMap.clone();
    cv::Mat mCopyDepthCov = this->m_mInvCovMap.clone();

    // #pragma omp parallel for
    for (int u=nBorder;u<nDepthWidth-nBorder;u++){
        // #pragma omp parallel for
        for (int v=nBorder;v<nDepthHeight-nBorder;v++){
            //Check if the point has coverage
            if (this->m_mCoverageMap.at<uchar>(v , u) != 2){
                continue;
            }

            double nInvDepth = 0.0;
            double nInvDepthCov = 0.0;
            double nMinCov = this->m_mInvCovMap.at<double>(v , u);
            double nCurrentInvDepth = this->m_mInvDepthMap.at<double>(v , u);
            double nCurrentCov = this->m_mInvCovMap.at<double>(v , u);
            
            for (int i=-nWindowSize;i<=nWindowSize;i++){
                for (int j=-nWindowSize;j<=nWindowSize;j++){
                    if (this->m_mCoverageMap.at<uchar>(v-i , u-j) != 2){
                        continue;
                    }
                    double nLocalInvDepth = mCopyDepthMap.at<double>(v-i , u-j);
                    double nLocalCov = this->m_mInvCovMap.at<double>(v-i , u-j);

                    if (abs(nLocalInvDepth - nCurrentInvDepth) >= 2 * nCurrentCov){
                        continue;
                    }

                    if (nLocalCov < nMinCov){
                        nMinCov = nLocalCov;
                    }
                    //Prevent from divide by zero.
                    if (nLocalCov < 0.001){
                        nLocalCov = 0.001;
                    }
                    nInvDepth += nLocalInvDepth * 1/nLocalCov;
                    nInvDepthCov += 1/nLocalCov;
                    
                }
            }

            nInvDepth /= nInvDepthCov;
            mCopyDepthMap.at<double>(v , u) = nInvDepth;
            mCopyDepthCov.at<double>(v , u) = nMinCov;
        }
    }

    this->m_mInvDepthMap = mCopyDepthMap.clone();
    this->m_mInvCovMap = mCopyDepthCov.clone();


    return true;
}


