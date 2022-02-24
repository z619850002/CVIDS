
#include "../include/server_keyframe.h"
using namespace std;

template <typename Derived>
static void reduceVector(vector<Derived> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

//Constructor.
ServerKeyFrame::ServerKeyFrame(double nTimeStamp, int nClient, int nLocalIndex, 
					Sophus::SE3 mVIOPose, 
					Eigen::Matrix3d mRotation_ic,
					Eigen::Vector3d mTranslation_ic,
					cv::Mat &mImage,
			 		vector<cv::Point3f> & gPoints3D, vector<cv::Point2f> &gPoints2D, 
			 		vector<int> & gPointsID,
			 		ServerCamera * pCamera){

	this->m_bHasBeenDisturbed = false;
	this->m_bHasLoop = false;
	this->m_bFreeSpace = false;

	this->m_pServerCamera = pCamera;

	//Set the identifier.
	this->m_nTimeStamp = nTimeStamp;
	this->m_nClientID = nClient;
	this->m_nLocalIndex = nLocalIndex;
	//FIXME: The index here should be modified in the future.
	this->m_nGlobalIndex = nLocalIndex;


	//Set the pose.
	this->m_mLocalPose_iw = mVIOPose;
	this->m_mLocalTranslation_wi = mVIOPose.inverse().translation();
	this->m_mLocalRotation_wi = mVIOPose.inverse().rotation_matrix();
	//In the begining the global pose should be set to the same to the local pose.
	this->m_mGlobalPose_iw = this->m_mLocalPose_iw;
	this->m_mGlobalTranslation_wi = this->m_mLocalTranslation_wi;
	this->m_mGlobalRotation_wi = this->m_mLocalRotation_wi;

	this->m_mLocalR_Backup_wi = this->m_mLocalRotation_wi;
	this->m_mLocalT_Backup_wi = this->m_mLocalTranslation_wi;
	


	//Set the extrinsics.
	this->m_mRotation_ic = mRotation_ic;
	this->m_mTranslation_ic = mTranslation_ic;


	//Copy the image.
	if (mImage.type() == CV_8UC1)    
	{

		this->m_mImage = mImage.clone();
		this->m_mColoredImage = mImage.clone();
	} 
	else if (mImage.type() == CV_8UC3)
	{
		this->m_mColoredImage = mImage.clone();
		cv::cvtColor(this->m_mColoredImage, this->m_mImage, CV_BGR2GRAY);

	}
	
	mImage.release();
	cv::resize(this->m_mImage, this->m_mThumbnailImage, cv::Size(80, 60));

	//Copy those points.
	this->m_gPoints3D = gPoints3D;
	this->m_gPoints2D = gPoints2D;

	this->m_gWindowBriefDescriptors.reserve(gPoints2D.size());
	// this->m_gWindowKeyPoints.reserve(gPoints2D.size());
	this->m_gNormalizedWindowPoints.reserve(gPoints2D.size());
	//Add normalized point
	for (auto iPoint2D : gPoints2D){
		Eigen::Vector2d mPointPixel(iPoint2D.x, iPoint2D.y);
		Eigen::Vector3d mTempPoint = this->m_pServerCamera->LiftProject(mPointPixel);
		this->m_gNormalizedWindowPoints.push_back(cv::Point2f(mTempPoint.x()/mTempPoint.z(), mTempPoint.y()/mTempPoint.z()));
	}
	//TODO: point_2d_norm = _point_2d_norm; not set yet!
	this->m_gPointsID = gPointsID;


	//Info about the loop closure.
	this->m_nLoopIndex = -1;

	this->m_mLoopInfo << 0, 0, 0, 0, 0, 0, 0, 0;


	this->ComputeWindowBRIEFPoint();
	this->ComputeBRIEFPoint();	

	this->m_pPreviousKeyFrame = NULL;
	this->m_pNextKeyFrame = NULL;

	//Image can be released now.
	// this->m_mImage.release();

	//Initialize the depth estimator.
	//Firstly compute the pose wc
	this->m_bDepthRefFrame = false;
	this->m_bFinalizeDepthMap = false;
	this->m_pRefKeyFrame = NULL;
}


//Constructor without image.
ServerKeyFrame::ServerKeyFrame(	double nTimeStamp, int nClient, int nLocalIndex, 
								Sophus::SE3 mVIOPose,
								Eigen::Matrix3d mRotation_ic,
    							Eigen::Vector3d mTranslation_ic, 
			 					vector<cv::Point3f> & gWindowPoints3D, 
			 					vector<cv::Point2f> & gWindowPoints2D, 
			 					vector<int> & gWindowPointsID,
			 					vector<DVision::BRIEF::bitset> & gWindowDescriptors,
			 					vector<cv::Point2f> & gProjectedPoints2D, 
			 					vector<DVision::BRIEF::bitset> & gProjectedDescriptors,
			 					bool bFromVisionSLAM,
			 					ServerCamera * pCamera){


	this->m_bHasBeenDisturbed = false;
	this->m_bHasLoop = false;
	this->m_bFreeSpace = false;

	this->m_pServerCamera = pCamera;

	//Set the identifier.
	this->m_nTimeStamp = nTimeStamp;
	this->m_nClientID = nClient;
	this->m_nLocalIndex = nLocalIndex;
	//FIXME: The index here should be modified in the future.
	this->m_nGlobalIndex = nLocalIndex;

	//Set the pose.
	this->m_mLocalPose_iw = mVIOPose;
	this->m_mLocalTranslation_wi = mVIOPose.inverse().translation();
	this->m_mLocalRotation_wi = mVIOPose.inverse().rotation_matrix();
	//In the begining the global pose should be set to the same to the local pose.
	this->m_mGlobalPose_iw = this->m_mLocalPose_iw;
	this->m_mGlobalTranslation_wi = this->m_mLocalTranslation_wi;
	this->m_mGlobalRotation_wi = this->m_mLocalRotation_wi;


	this->m_mLocalR_Backup_wi = this->m_mLocalRotation_wi;
	this->m_mLocalT_Backup_wi = this->m_mLocalTranslation_wi;
	


	//Set the extrinsics.
	this->m_mRotation_ic = mRotation_ic;
	this->m_mTranslation_ic = mTranslation_ic;

	
	//If mappoints can be obtained.
	this->m_bFromVisionSLAM = bFromVisionSLAM;


	//Copy those map points.
	if (bFromVisionSLAM){
		this->m_gPoints3D = gWindowPoints3D;
		this->m_gPoints2D = gWindowPoints2D;
		this->m_gWindowBriefDescriptors = gWindowDescriptors;
		//Compute the normalized point.
		this->m_gNormalizedWindowPoints.reserve(gWindowPoints2D.size());
		//Add normalized points.
		for (auto iPoint2D : gWindowPoints2D){
			Eigen::Vector2d mPointPixel(iPoint2D.x, iPoint2D.y);
			Eigen::Vector3d mTempPoint = this->m_pServerCamera->LiftProject(mPointPixel);
			this->m_gNormalizedWindowPoints.push_back(cv::Point2f(mTempPoint.x()/mTempPoint.z(), mTempPoint.y()/mTempPoint.z()));
		}
		//ID of map points.
		this->m_gPointsID = gWindowPointsID;	
	}

	
	//Copy of those points to be projected.
	this->m_gProjectedPoints = gProjectedPoints2D;
	this->m_gBriefDescriptors = gProjectedDescriptors;
	//Add normalized points.
	for (auto iPoint2D : gProjectedPoints2D){
		Eigen::Vector2d mPointPixel(iPoint2D.x, iPoint2D.y);
		Eigen::Vector3d mTempPoint = this->m_pServerCamera->LiftProject(mPointPixel);
		this->m_gNormalizedProjectedPoints.push_back(cv::Point2f(mTempPoint.x()/mTempPoint.z(), mTempPoint.y()/mTempPoint.z()));
	}



	//Info about the loop closure.
	this->m_nLoopIndex = -1;

	this->m_mLoopInfo << 0, 0, 0, 0, 0, 0, 0, 0;


	this->m_pPreviousKeyFrame = NULL;
	this->m_pNextKeyFrame = NULL;

	// cout << "Create Debug!" << endl;	
	
	// vector<cv::Point2f> gCurrentPoints2D, gOldPoints2D;
	// vector<cv::Point2f> gCurrentNorm2D, gOldNorm2D;
	// vector<cv::Point3f> gMatchedPoints3D;
	// vector<int> gMatchedID;
	// vector<uchar> gStatus;

	// gMatchedPoints3D = this->m_gPoints3D;
	// gCurrentPoints2D = this->m_gPoints2D;
	// gCurrentNorm2D = this->m_gNormalizedWindowPoints;
	// gMatchedID = this->m_gPointsID;



	// Eigen::Matrix3d mRotationDebug;
	// Eigen::Vector3d mTranslationDebug;


	// PnPRANSAC(gCurrentNorm2D, gMatchedPoints3D, gStatus, mTranslationDebug, mRotationDebug);
	// cout << "Debug rotation is: " << endl << mRotationDebug << endl;
	// cout << "Debug translation is: " << endl << mTranslationDebug << endl;
	// cout << "GT rotation is: " << endl << m_mLocalRotation_wi << endl;
	// cout << "GT translation is: " << endl << m_mLocalTranslation_wi << endl;


	//Initialize the depth estimator.
	//Firstly compute the pose wc
	this->m_bDepthRefFrame = false;
	this->m_bFinalizeDepthMap = false;
	this->m_pRefKeyFrame = NULL;
	this->m_bHasBeenAligned = false;

}


void ServerKeyFrame::UndistortImage(){
	cv::Mat mK, mD;
	cv::eigen2cv(this->m_pServerCamera->GetK(), mK);
	cv::eigen2cv(this->m_pServerCamera->GetD(), mD);
    cv::undistort(m_mColoredImage, m_mUndistortedImage, mK, mD, mK);
}



//Here may be another constructor to load old keyframes from the disk.
void ServerKeyFrame::ComputeWindowBRIEFPoint(){
	ServerBriefExtractor iExtractor(SERVER_BRIEF_PATTERN_FILE.c_str());
	//Iterate for all 2d points.
	vector<cv::KeyPoint> gWindowKeyPoints;
	gWindowKeyPoints.reserve(this->m_gPoints2D.size());
	for (auto iPoint2D : this->m_gPoints2D){
		cv::KeyPoint iKeyPoint;
		iKeyPoint.pt = iPoint2D;
		gWindowKeyPoints.push_back(iKeyPoint);
	}
	iExtractor(this->m_mImage, gWindowKeyPoints, this->m_gWindowBriefDescriptors);
}



void ServerKeyFrame::ComputeBRIEFPoint(){
	ServerBriefExtractor iExtractor(SERVER_BRIEF_PATTERN_FILE.c_str());
	//Fast corner detection.
	vector<cv::KeyPoint> gKeyPoints;
	const int nFastThresh = 20;
	cv::FAST(this->m_mImage, gKeyPoints, nFastThresh, true);
	//Extract features.
	iExtractor(this->m_mImage, gKeyPoints, this->m_gBriefDescriptors);

	this->m_gProjectedPoints.reserve(gKeyPoints.size());
	this->m_gNormalizedProjectedPoints.reserve(gKeyPoints.size());
	//Get the normalized coordinates of these points.
	for (int i = 0; i < (int)gKeyPoints.size(); i++)
	{
		Eigen::Vector3d mTempPoint;
		cv::KeyPoint iKeyPoint = gKeyPoints[i];
		//Add it to this keyframe.
		this->m_gProjectedPoints.push_back(iKeyPoint.pt);

		mTempPoint = this->m_pServerCamera->LiftProject(Eigen::Vector2d(iKeyPoint.pt.x, iKeyPoint.pt.y));
		cv::Point2f iNormalizedPoint = cv::Point2f(mTempPoint.x()/mTempPoint.z(), mTempPoint.y()/mTempPoint.z());
		this->m_gNormalizedProjectedPoints.push_back(iNormalizedPoint);
	}
}


//Find one matched point from the old descriptors.
bool ServerKeyFrame::SearchInArea(		const DVision::BRIEF::bitset iWindowDescriptor,
                            			const vector<DVision::BRIEF::bitset> & gOldDescriptors,
                            			const vector<cv::Point2f> & gOldPoints,
                            			const vector<cv::Point2f> & gNormOldPoints,
                            			cv::Point2f & iBestMatchedPoint,
                            			cv::Point2f & iBestMatchedNormPoint){
    int nBestDistance = 128;
    int nSecondBestDistance = 128;
    bool bFirstBest = true;
    int nBestIndex = -1;
    for(int i = 0; i < (int)gOldDescriptors.size(); i++)
    {
        int nDistance = this->HammingDistance(iWindowDescriptor, gOldDescriptors[i]);
        
        if(nDistance < nBestDistance)
        {
        	if (bFirstBest){
        		bFirstBest = false;
        	}else{
        		nSecondBestDistance = nBestDistance;
        	}
            nBestDistance = nDistance;
            nBestIndex = i;
        }
    }

    //printf("best dist %d", bestDist);
    if (nBestIndex != -1 && nBestDistance < 80 && nBestDistance < 0.7 * nSecondBestDistance)
    {
      iBestMatchedPoint = gOldPoints[nBestIndex];
      iBestMatchedNormPoint = gNormOldPoints[nBestIndex];

      return true;
    }
    else{

      return false;
  }
}



int ServerKeyFrame::HammingDistance(	const DVision::BRIEF::bitset & iDescriptorA, 
										const DVision::BRIEF::bitset & iDescriptorB)
{
    DVision::BRIEF::bitset iOrDescript  = iDescriptorA ^ iDescriptorB;
    int nDistance = iOrDescript.count();
    return nDistance;
}



//Find correspondences of points in this frame and another frame.
void ServerKeyFrame::SearchByBRIEFDes(	const vector<DVision::BRIEF::bitset> & gOldDescriptors,
                                		const vector<cv::Point2f> & gOldPoints,
                                		const vector<cv::Point2f> & gOldNormPoints,
                                		vector<uchar> & gStatus,
										vector<cv::Point2f> & gMatchedPoints2D,
										vector<cv::Point2f> & gNormMatchedPoints2D)
{
	//Reserve
	gStatus.reserve(m_gWindowBriefDescriptors.size());
	gMatchedPoints2D.reserve(m_gWindowBriefDescriptors.size());
	gNormMatchedPoints2D.reserve(m_gWindowBriefDescriptors.size());



    for(int i = 0; i < (int)this->m_gWindowBriefDescriptors.size(); i++)
    {
        cv::Point2f iPoint2D(0.f, 0.f);
        cv::Point2f iNormPoint2D(0.f, 0.f);

        if (this->SearchInArea(this->m_gWindowBriefDescriptors[i], gOldDescriptors, gOldPoints, gOldNormPoints, iPoint2D, iNormPoint2D)){
          gStatus.push_back(1);
        }
        else{
          gStatus.push_back(0);
        }

        gMatchedPoints2D.push_back(iPoint2D);
        gNormMatchedPoints2D.push_back(iNormPoint2D);
    }


}


//Not used yet.
void ServerKeyFrame::FundmantalMatrixRANSAC(const vector<cv::Point2f> & gCurrentNormPoints,
                                    		const vector<cv::Point2f> & gOldNormPoints,
                                    		vector<uchar> & gStatus)
{
	int nPointsNumber = (int) gCurrentNormPoints.size();
	gStatus.reserve(nPointsNumber);
	for (int i = 0; i < nPointsNumber; i++){
		gStatus.push_back(0);
	}
	//Only with more than 8 points the algorithm can be activated.
    if (nPointsNumber >= 8)
    {
        vector<cv::Point2f> gTempCurrent(nPointsNumber), gTempOld(nPointsNumber);
        for (int i = 0; i < (int)nPointsNumber; i++)
        {
        	//TODO: The focal length has been modified!
        	// Eigen::Matrix3d mK = this->m_pServerCamera->GetK();

            //Convert to pixel coordinate!!!
            Eigen::Vector3d mCurrentNormPoint(gCurrentNormPoints[i].x, gCurrentNormPoints[i].y, 1.0);
            Eigen::Vector3d mOldNormPoint(gOldNormPoints[i].x, gOldNormPoints[i].y, 1.0);

            Eigen::Vector2d mCurrentPixel = this->m_pServerCamera->Project(mCurrentNormPoint);
            Eigen::Vector2d mOldPixel = this->m_pServerCamera->Project(mOldNormPoint);

            gTempCurrent[i] = cv::Point2f(mCurrentPixel.x(), mCurrentPixel.y());
            gTempOld[i] = cv::Point2f(mOldPixel.x(), mOldPixel.y());
            
        }
        cv::findFundamentalMat(gTempCurrent, gTempOld, cv::FM_RANSAC, 3.0, 0.9, gStatus);
    }
}


//Maybe something wrong in this function??
//The guessed keyframe should be old keyframe's
void ServerKeyFrame::PnPRANSAC(		const vector<cv::Point2f> & gOldNormPoints2D,
                        			const vector<cv::Point3f> & gMatchedPoints3D,
                        			vector<uchar> & gStatus,
                        			Eigen::Vector3d & mOldTranslation_wi, 
                        			Eigen::Matrix3d & mOldRotation_wi)
{
    cv::Mat mRotationCV, mRotationVecCV, mTranslationCV, mDistortionCV, mTempRotationCV;
    //The 2d points are normalized, thus the intrinsic matrix should be set to identity matrix.
    cv::Mat mK = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
    mDistortionCV = (cv::Mat_<double>(4 , 1) << 0.0, 0.0, 0.0, 0.0);
    
    //Extrinsics
    Eigen::Matrix3d mRotation_ic = this->m_mRotation_ic;
    Eigen::Vector3d mTranslation_ic = this->m_mTranslation_ic;

    //Get the pose of the camera.
    //Current Frame
    Eigen::Matrix3d mRotation_wc = this->m_mGlobalRotation_wi * mRotation_ic;
    Eigen::Vector3d mTranslation_wc = this->m_mGlobalTranslation_wi + this->m_mGlobalRotation_wi * mTranslation_ic;

    //Now we obtain the pose of the camera.
    //The pose of current frame
    Eigen::Matrix3d mInitialRotation_cw = mRotation_wc.inverse();
    Eigen::Vector3d mInitialTranslation_cw = -(mInitialRotation_cw * mTranslation_wc);
    //Convert them to cv type.
    cv::eigen2cv(mInitialRotation_cw, mTempRotationCV);
    cv::Rodrigues(mTempRotationCV, mRotationVecCV);
    cv::eigen2cv(mInitialTranslation_cw, mTranslationCV);

    cv::Mat mInliers;

    //FIXME:Change the initial value of the pose.
    //Use PnP to find outliers.
    if (CV_MAJOR_VERSION < 3){
        cv::solvePnPRansac(gMatchedPoints3D, gOldNormPoints2D, mK, mDistortionCV, mRotationVecCV, mTranslationCV, false, 100, 10.0 / 460.0, 100, mInliers);
    }
    else
    {
        if (CV_MINOR_VERSION < 2){
            cv::solvePnPRansac(gMatchedPoints3D, gOldNormPoints2D, mK, mDistortionCV, mRotationVecCV, mTranslationCV, false, 100, sqrt(10.0 / 460.0), 0.99, mInliers);
		    
        }
        else{
            cv::solvePnPRansac(gMatchedPoints3D, gOldNormPoints2D, mK, mDistortionCV, mRotationVecCV, mTranslationCV, false, 100, 10.0 / 460.0, 0.99, mInliers);
        }
    }


    //The pose has also be updated.
    cv::Rodrigues(mRotationVecCV, mRotationCV);

    gStatus.reserve(gOldNormPoints2D.size());
    for (int i = 0; i < (int)gOldNormPoints2D.size(); i++){
        gStatus.push_back(0);
    }

    for( int i = 0; i < mInliers.rows; i++)
    {
        int n = mInliers.at<int>(i);
        gStatus[n] = 1;
    }

    // 4.655261,-1.613830,0.669651
    // 4.594507,-1.643386,0.653808


    Eigen::Matrix3d mRotationPnP_cw, mRotationPnP_wc;
    cv::cv2eigen(mRotationCV, mRotationPnP_cw);


    mRotationPnP_wc = mRotationPnP_cw.transpose();

    Eigen::Vector3d mTranslationPnP_cw, mTranslationPnP_wc;
    cv::cv2eigen(mTranslationCV, mTranslationPnP_cw);
    mTranslationPnP_wc = mRotationPnP_wc * (-mTranslationPnP_cw);

    //We have solved the inverse pose of the old keyframe.
    mOldRotation_wi = mRotationPnP_wc * mRotation_ic.transpose();
    mOldTranslation_wi = mTranslationPnP_wc - mOldRotation_wi * mTranslation_ic;
}


//Find the loop info, T i_old i_current
bool ServerKeyFrame::FindConnection(ServerKeyFrame* pOldKeyFrame, bool bAlign)
{
	int nMinLoop = BIG_LOOP_NUM;
	if (bAlign){
		nMinLoop = MIN_LOOP_NUM;
	}
	
	vector<cv::Point2f> gCurrentPoints2D, gOldPoints2D;
	vector<cv::Point2f> gCurrentNorm2D, gOldNorm2D;
	vector<cv::Point3f> gMatchedPoints3D;
	vector<int> gMatchedID;
	vector<uchar> gStatus;

	gMatchedPoints3D = this->m_gPoints3D;
	gCurrentPoints2D = this->m_gPoints2D;
	gCurrentNorm2D = this->m_gNormalizedWindowPoints;
	gMatchedID = this->m_gPointsID;



	//Find matches.
	this->SearchByBRIEFDes( pOldKeyFrame->m_gBriefDescriptors, 
							pOldKeyFrame->m_gProjectedPoints, 
							pOldKeyFrame->m_gNormalizedProjectedPoints, 
							gStatus,
							gOldPoints2D, gOldNorm2D);
	

	//Remove unmatched points.
	reduceVector(gCurrentPoints2D, gStatus);
	reduceVector(gOldPoints2D, gStatus);
	reduceVector(gCurrentNorm2D, gStatus);
	reduceVector(gOldNorm2D, gStatus);
	reduceVector(gMatchedPoints3D, gStatus);
	reduceVector(gMatchedID, gStatus);
	gStatus.clear();


	this->FundmantalMatrixRANSAC(gCurrentNorm2D, gOldNorm2D, gStatus);
	//Remove unmatched points.
	reduceVector(gCurrentPoints2D, gStatus);
	reduceVector(gOldPoints2D, gStatus);
	reduceVector(gCurrentNorm2D, gStatus);
	reduceVector(gOldNorm2D, gStatus);
	reduceVector(gMatchedPoints3D, gStatus);
	reduceVector(gMatchedID, gStatus);
	gStatus.clear();


	Eigen::Vector3d mOldTranslation_wi;
	Eigen::Matrix3d mOldRotation_wi;
	Eigen::Vector3d mRelativeTranslation;
	Eigen::Quaterniond mRelativeQuanternion;


	double nRelativeYaw;

	//Debug
	Eigen::Matrix3d mCurrentRotationDebug;
	Eigen::Vector3d mCurrentTranslationDebug;



	//Firstly use PnP to remove outliers.
	if ((int)gCurrentPoints2D.size() > nMinLoop)
	{
		gStatus.clear();
	    PnPRANSAC(gOldNorm2D, gMatchedPoints3D, gStatus, mOldTranslation_wi, mOldRotation_wi);
	 	
	 	// PnPRANSAC(gCurrentNorm2D, gMatchedPoints3D, gStatus, mOldTranslation_wi, mOldRotation_wi);

	    reduceVector(gCurrentPoints2D, gStatus);
		reduceVector(gOldPoints2D, gStatus);
		reduceVector(gCurrentNorm2D, gStatus);
		reduceVector(gOldNorm2D, gStatus);
		reduceVector(gMatchedPoints3D, gStatus);
		reduceVector(gMatchedID, gStatus);
		
		//Remove the plotting module.
	}else{
		return false;
	}

	//Plot the matching result.
   //Now we can draw the matching correspondence.
    vector<cv::KeyPoint> gCurrentKeyPoints, gOldKeyPoints;
    vector<cv::DMatch> gMatches;
    gCurrentKeyPoints.reserve(gCurrentPoints2D.size());
    gOldKeyPoints.reserve(gCurrentPoints2D.size());
    gMatches.reserve(gCurrentPoints2D.size());

    for (int ii=0;ii<gCurrentPoints2D.size();ii++){
    	cv::KeyPoint iCurrentKeyPoint, iOldKeyPoint;
    	iCurrentKeyPoint.pt = gCurrentPoints2D[ii];
    	iOldKeyPoint.pt = gOldPoints2D[ii];
    	gCurrentKeyPoints.push_back(iCurrentKeyPoint);
    	gOldKeyPoints.push_back(iOldKeyPoint);
    	cv::DMatch iMatch;
    	iMatch.trainIdx = ii;
    	iMatch.queryIdx = ii;
    	gMatches.push_back(iMatch);
    }
    cv::Mat mMatchedImage;

    cv::drawMatches(this->m_mImage, gCurrentKeyPoints, pOldKeyFrame->m_mImage, gOldKeyPoints, gMatches, mMatchedImage);
    this->m_mMatchedImage = mMatchedImage.clone();
    // cv::imwrite("/home/kyrie/Documents/DataSet/CoVins/loop_closure.jpg", mMatchedImage);
    // // cv::waitKey(5);


	//Remove outliers with the epipolar constraints.
	// cout << "Matched size: " << gCurrentPoints2D.size() << endl;
	if ((int)gCurrentPoints2D.size() >= nMinLoop)
	{
	    mRelativeTranslation = mOldRotation_wi.transpose() * (this->m_mGlobalTranslation_wi - mOldTranslation_wi);
	    mRelativeQuanternion = mOldRotation_wi.transpose() * this->m_mGlobalRotation_wi;
	    

	 //    // mRelativeQuanternion.toRotationMatrix();
	 //    Eigen::Vector3d mEulerGlobal = ServerUtility::R2ypr(this->m_mGlobalRotation_wi);
	 //    Eigen::Vector3d mEulerOld    = ServerUtility::R2ypr(mOldRotation_wi);

		// Eigen::Vector3d mTransformOld(0.0, mEulerOld(1), mEulerOld(2));
		// Eigen::Vector3d mTransformGlobal(0.0, mEulerGlobal(1), mEulerGlobal(2));

		// Eigen::Matrix3d mMatrixOld = ServerUtility::ypr2R(mTransformOld);
		// Eigen::Matrix3d mMatrixGlobal = ServerUtility::ypr2R(mTransformGlobal);


	 // 	double nRelativeYaw1 = (ServerUtility::R2ypr(mMatrixOld * mRelativeQuanternion.toRotationMatrix() * mMatrixGlobal.inverse())).x();
	 // 	double nRelativeYaw2 = (ServerUtility::R2ypr(this->m_mGlobalRotation_wi).x() - ServerUtility::R2ypr(mOldRotation_wi).x());

	 // 	for (int ind =0; ind<10;ind++){
	 // 		cout << "Relative yaw 1 is: " << nRelativeYaw1 << endl;
		//  	cout << "Relative yaw 2 is: " << nRelativeYaw2 << endl;	
	 // 	}
	    


	    // mRelativeQuanternion.toRotationMatrix();
	 	
	 	// double nRelativeYaw1 = ServerUtility::R2ypr(mRelativeQuanternion).x();
	 	// double nRelativeYaw2 = (ServerUtility::R2ypr(this->m_mGlobalRotation_wi).x() - ServerUtility::R2ypr(mOldRotation_wi).x());

	 	// cout << "Relative yaw 1 is: " << nRelativeYaw1 << endl;
	 	// cout << "Relative yaw 2 is: " << nRelativeYaw2 << endl;

	    nRelativeYaw = (ServerUtility::R2ypr(this->m_mGlobalRotation_wi).x() - ServerUtility::R2ypr(mOldRotation_wi).x());
	    // for (int i=0;i<100;i++){
	    // 	cout << endl;
	    // 	cout << fixed << setprecision(6);
	    // 	cout << "Old Timestamp: " << pOldKeyFrame->m_nTimeStamp << endl;
	    // 	cout << "Current Timestamp: " << this->m_nTimeStamp << endl;
	    // 	cout << "RelativeYaw is: " << nRelativeYaw << endl;
	    // 	cout << "RelativeTranslation is: " << mRelativeTranslation << endl;
	    // 	cout << "Local Rotation" << endl << this->m_mLocalRotation_wi << endl;
	    // 	cout << "Local Translation" << endl << this->m_mLocalT_Backup_wi << endl;
	    // 	cout << "Rotation ic is: " << endl << m_mRotation_ic << endl;
	    // 	cout << "Translation ic is: " << endl << m_mTranslation_ic << endl;
	    // 	cout << "points are: " << endl;
	    // 	for (int i=0;i<gMatchedPoints3D.size();i++){
	    // 		cout << "Point3d: " << gMatchedPoints3D[i] << endl;
	    // 		// cout << "Norm2d: " << gOldNorm2D[i] << endl; 
	    // 	}

	    // 	for (int i=0;i<gMatchedPoints3D.size();i++){
	    // 		cout << "Point2d: " << gCurrentPoints2D[i] << endl;
	    // 		// cout << "Norm2d: " << gOldNorm2D[i] << endl; 
	    // 	}


	    // 	for (int i=0;i<gMatchedPoints3D.size();i++){
	    // 		cout << "OldPoint2d" << gOldPoints2D[i] << endl;
	    // 		// cout << "Norm2d: " << gOldNorm2D[i] << endl; 
	    // 	}


	    // 	for (int i=0;i<gMatchedPoints3D.size();i++){
	    // 		cout << "OldNorm" << gOldNorm2D[i] << endl;
	    // 		// cout << "Norm2d: " << gOldNorm2D[i] << endl; 
	    // 	}


	    // 	for (int i=0;i<gMatchedPoints3D.size();i++){
	    // 		cout << "CurrentNorm" << gCurrentNorm2D[i] << endl;
	    // 		// cout << "Norm2d: " << gOldNorm2D[i] << endl; 
	    // 	}

	    // 	cout << endl;
	    // }
	 
	    if (abs(nRelativeYaw) < 40.0 && mRelativeTranslation.norm() < 40.0)
	    {

	    	this->m_bHasLoop = true;
	    	this->m_nLoopIndex = pOldKeyFrame->m_nGlobalIndex;
	    	this->m_mLoopInfo << 	mRelativeTranslation.x(), mRelativeTranslation.y(), mRelativeTranslation.z(),
	    	             			mRelativeQuanternion.w(), mRelativeQuanternion.x(), mRelativeQuanternion.y(), mRelativeQuanternion.z(),
	    	             			nRelativeYaw;

	    	bool bExist = false;
	    	for (int ii=0;ii<this->m_gLoopIndices.size();ii++){
	    		if (this->m_gLoopIndices[ii] == pOldKeyFrame->m_nGlobalIndex){
	    			bExist = true;
	    			break;
	    		}
	    	}

	    	if (!bExist){    		
		    	this->m_gLoopIndices.push_back(this->m_nLoopIndex);
		    	this->m_gLoopInfos.push_back(this->m_mLoopInfo);
	    	}
	    	//Fast relocalization is not required!!!
	        return true;
	    }
	}
	return false;
}

//Get Local Pose.
void ServerKeyFrame::GetVIOPose(Eigen::Vector3d & mTranslation_wi , Eigen::Matrix3d & mRotation_wi)
{

	this->m_mPoseMutex.lock();
    mTranslation_wi = this->m_mLocalTranslation_wi;
    mRotation_wi = this->m_mLocalRotation_wi;

	this->m_mPoseMutex.unlock();
}

//Get Global Pose.
void ServerKeyFrame::GetPose(Eigen::Vector3d & mTranslation_wi, Eigen::Matrix3d & mRotation_wi)
{

	this->m_mPoseMutex.lock();
    mTranslation_wi = this->m_mGlobalTranslation_wi;
    mRotation_wi = this->m_mGlobalRotation_wi;

	this->m_mPoseMutex.unlock();
}

void ServerKeyFrame::GetCameraPose(Eigen::Vector3d & mTranslation_wc, Eigen::Matrix3d & mRotation_wc){

	this->m_mPoseMutex.lock();
	Sophus::SE3 mPose_wi = Sophus::SE3(
		this->m_mGlobalRotation_wi,
		this->m_mGlobalTranslation_wi); 
	
	Sophus::SE3 mPose_ic(this->m_mRotation_ic, this->m_mTranslation_ic);
	Sophus::SE3 mPose_wc = mPose_wi * mPose_ic;
	mTranslation_wc = mPose_wc.translation();
	mRotation_wc = mPose_wc.rotation_matrix();
	this->m_mPoseMutex.unlock();
}


void ServerKeyFrame::UpdatePose(const Eigen::Vector3d & mTranslation_wi, const Eigen::Matrix3d & mRotation_wi)
{
	this->m_mPoseMutex.lock();
    this->m_mGlobalTranslation_wi = mTranslation_wi;
    this->m_mGlobalRotation_wi = mRotation_wi;

    this->m_mGlobalPose_iw = Sophus::SE3(mRotation_wi, mTranslation_wi).inverse();
	this->m_mPoseMutex.unlock();

}


void ServerKeyFrame::ShiftPose(const Eigen::Vector3d & mRelativeT, const Eigen::Matrix3d & mRelativeR){
	this->m_mPoseMutex.lock();
	//Compute the shift.
	// Sophus::SE3 mPose_ni(mRotation_wi, mTranslation_wi);
	// Sophus::SE3 mPose_wi(this->m_mLocalRotation_wi, this->m_mLocalTranslation_wi);

	Sophus::SE3 mPose_nw(mRelativeR, mRelativeT);

    for (int i=0;i<this->m_gPoints3D.size();i++){
        cv::Point3f iPoint3D = this->m_gPoints3D[i];
        Eigen::Vector3d mPoint3D(
        	iPoint3D.x, 
        	iPoint3D.y, 
        	iPoint3D.z);
        mPoint3D = mPose_nw * mPoint3D;
        iPoint3D.x = mPoint3D(0);
        iPoint3D.y = mPoint3D(1);
        iPoint3D.z = mPoint3D(2);
        this->m_gPoints3D[i] = iPoint3D;
    }


	this->m_mLocalTranslation_wi = mRelativeR * this->m_mLocalTranslation_wi + mRelativeT;
    this->m_mLocalRotation_wi = mRelativeR * this->m_mLocalRotation_wi;

    this->m_mLocalPose_iw = Sophus::SE3(this->m_mLocalRotation_wi, this->m_mLocalTranslation_wi).inverse();

    //The global pose should also be set.
    this->m_mGlobalTranslation_wi = mRelativeR * this->m_mGlobalTranslation_wi + mRelativeT;
    this->m_mGlobalRotation_wi = mRelativeR * this->m_mGlobalRotation_wi;

    this->m_mGlobalPose_iw = Sophus::SE3(this->m_mGlobalRotation_wi, this->m_mGlobalTranslation_wi).inverse();

	this->m_mPoseMutex.unlock();
}





//The global pose should also be modified.
void ServerKeyFrame::UpdateVIOPose(const Eigen::Vector3d & mTranslation_wi, const Eigen::Matrix3d & mRotation_wi)
{

	this->m_mPoseMutex.lock();
	//Compute the shift.
	Sophus::SE3 mPose_ni(mRotation_wi, mTranslation_wi);
	Sophus::SE3 mPose_wi(this->m_mLocalRotation_wi, this->m_mLocalTranslation_wi);

	Sophus::SE3 mPose_nw = mPose_ni * mPose_wi.inverse();

    for (int i=0;i<this->m_gPoints3D.size();i++){
        cv::Point3f iPoint3D = this->m_gPoints3D[i];
        Eigen::Vector3d mPoint3D(iPoint3D.x, iPoint3D.y, iPoint3D.z);
        mPoint3D = mPose_nw * mPoint3D;
        iPoint3D.x = mPoint3D(0);
        iPoint3D.y = mPoint3D(1);
        iPoint3D.z = mPoint3D(2);
        this->m_gPoints3D[i] = iPoint3D;
    }


	this->m_mLocalTranslation_wi = mTranslation_wi;
    this->m_mLocalRotation_wi = mRotation_wi;

    this->m_mLocalPose_iw = Sophus::SE3(mRotation_wi, mTranslation_wi).inverse();

    //The global pose should also be set.
    this->m_mGlobalTranslation_wi = this->m_mLocalTranslation_wi;
    this->m_mGlobalRotation_wi = this->m_mLocalRotation_wi;

    this->m_mGlobalPose_iw = this->m_mLocalPose_iw;

	this->m_mPoseMutex.unlock();
}


Eigen::Vector3d ServerKeyFrame::GetLoopRelativeT()
{
    return Eigen::Vector3d(m_mLoopInfo(0), m_mLoopInfo(1), m_mLoopInfo(2));
}

//Old Local
Eigen::Quaterniond ServerKeyFrame::GetLoopRelativeQ()
{
    return Eigen::Quaterniond(m_mLoopInfo(3), m_mLoopInfo(4), m_mLoopInfo(5), m_mLoopInfo(6));
}



Eigen::Vector3d ServerKeyFrame::GetLoopRelativeT(int nIndex)
{
	Eigen::Matrix<double, 8, 1 > mLoopInfo = this->m_gLoopInfos[nIndex];
    return Eigen::Vector3d(mLoopInfo(0), mLoopInfo(1), mLoopInfo(2));
}

//Old Local
Eigen::Quaterniond ServerKeyFrame::GetLoopRelativeQ(int nIndex)
{
	Eigen::Matrix<double, 8, 1 > mLoopInfo = this->m_gLoopInfos[nIndex];
    return Eigen::Quaterniond(mLoopInfo(3), mLoopInfo(4), mLoopInfo(5), mLoopInfo(6));
}


double ServerKeyFrame::GetLoopRelativeYaw(int nIndex)
{
    Eigen::Matrix<double, 8, 1 > mLoopInfo = this->m_gLoopInfos[nIndex];
    return mLoopInfo(7);
}


double ServerKeyFrame::GetLoopRelativeYaw()
{
    return this->m_mLoopInfo(7);
}

void ServerKeyFrame::UpdateLoop(Eigen::Matrix<double, 8, 1 > & mLoopInfo)
{
	if (abs(mLoopInfo(7)) < 30.0 && Eigen::Vector3d(mLoopInfo(0), mLoopInfo(1), mLoopInfo(2)).norm() < 20.0)
	{
		//printf("update loop info\n");
		this->m_mLoopInfo = mLoopInfo;
	}
}


void ServerKeyFrame::InitializeDepthEstimator(){
	this->m_mDepthMutex.lock();

	this->m_mPoseMutex.lock();
	this->m_bDepthRefFrame = true;

	// this->m_mLocalR_Backup_wi = this->m_mLocalRotation_wi;
	// this->m_mLocalT_Backup_wi = this->m_mLocalTranslation_wi;
	Sophus::SE3 mBackupPose_wi = Sophus::SE3(
		this->m_mLocalR_Backup_wi,
		this->m_mLocalT_Backup_wi); 
	
	Sophus::SE3 mPose_ic(this->m_mRotation_ic, this->m_mTranslation_ic);
	Sophus::SE3 mPose_wc = mBackupPose_wi * mPose_ic;
	Eigen::Matrix3d mK = this->m_pServerCamera->GetK();
	Eigen::Vector4d mD = this->m_pServerCamera->GetD();


	int nWidth = this->m_mImage.cols;
	int nHeight = this->m_mImage.rows;

	//Create the estimator.
	this->m_pEstimator = new DepthEstimator(
		this->m_mImage,
		mPose_wc,
		mK(0 , 0),
		mK(1 , 1),
		mK(0 , 2),
		mK(1 , 2),
		mD(0 , 0),
		mD(1 , 0),
		mD(2 , 0),
		mD(3 , 0));
	this->m_pEstimator->LoadMaps(
		this->m_mInvDepthMap, 
		this->m_mInvCovMap, 
		this->m_mConvergeMap);


	vector<cv::Point2d> gSparsePoints2D;
	vector<double> gSparseDepth;
	gSparseDepth.reserve(this->m_gPoints2D.size());
	gSparsePoints2D.reserve(this->m_gPoints2D.size());
    for (int i=0;i<this->m_gPoints2D.size();i++){

    	cv::Point2d iPoint2d = this->m_gPoints2D[i];
        cv::Point2d iUndistortedPoint2d = this->m_gNormalizedWindowPoints[i];
        Eigen::Vector3d mUndistortedPoint2d(iUndistortedPoint2d.x , iUndistortedPoint2d.y , 1.0);
        mUndistortedPoint2d = this->m_pServerCamera->GetK() * mUndistortedPoint2d;

        if ( mUndistortedPoint2d(0) < 20.5 || mUndistortedPoint2d(0) > nWidth-20.5 || 
             mUndistortedPoint2d(1) < 20.5 || mUndistortedPoint2d(1) > nHeight-20.5){
            continue;
        }

        int nX = mUndistortedPoint2d(0), nY = mUndistortedPoint2d(1);
        cv::Point3d iPoint3d = this->m_gPoints3D[i];
        Eigen::Vector3d mPoint3d(iPoint3d.x, iPoint3d.y, iPoint3d.z);
        
        mPoint3d = mPose_wc.inverse() * mPoint3d;
        double nSparseDepth = mPoint3d(2);
        
        gSparsePoints2D.push_back(cv::Point2d(nX, nY));
        gSparseDepth.push_back(nSparseDepth);

    }

    this->m_pEstimator->BindSparsePoints(gSparseDepth, gSparsePoints2D);

	this->m_mPoseMutex.unlock();
	this->m_mDepthMutex.unlock();

}



void ServerKeyFrame::PropogateDepthFilter(ServerKeyFrame * pPreviousKeyFrame){
	this->m_mDepthMutex.lock();

	this->m_mPoseMutex.lock();
	//Compute the pose wc of the matched frame.

	Sophus::SE3 mBackupPose_wi = Sophus::SE3(
		pPreviousKeyFrame->m_mLocalR_Backup_wi,
		pPreviousKeyFrame->m_mLocalT_Backup_wi); 

	Sophus::SE3 mPose_ic(pPreviousKeyFrame->m_mRotation_ic, pPreviousKeyFrame->m_mTranslation_ic);
	Sophus::SE3 mPreviousPose_wc = mBackupPose_wi * mPose_ic;

	Sophus::SE3 mCurrentPose_wi = Sophus::SE3(
		this->m_mLocalR_Backup_wi,
		this->m_mLocalT_Backup_wi);
	Sophus::SE3 mCurrentPose_ic(
		this->m_mRotation_ic, 
		this->m_mTranslation_ic);
	Sophus::SE3 mCurrentPose_wc = mCurrentPose_wi * mCurrentPose_ic;

	Sophus::SE3 mRelativePose_co = mCurrentPose_wc.inverse() * mPreviousPose_wc;

	this->m_pEstimator->PropogateFromPreviousFrame(
		pPreviousKeyFrame->m_pEstimator, 
		mRelativePose_co);



	this->m_mPoseMutex.unlock();
	this->m_mDepthMutex.unlock();
}

void ServerKeyFrame::FuseFrame(ServerKeyFrame * pMatchedKeyFrame, bool bFuse){
	this->m_mDepthMutex.lock();

	this->m_mPoseMutex.lock();
	//Double bind.
	this->m_gMatchedKeyFrames.push_back(pMatchedKeyFrame);
	pMatchedKeyFrame->m_pRefKeyFrame = this;
	//Compute the pose wc of the matched frame.

	Sophus::SE3 mBackupPose_wi = Sophus::SE3(
		pMatchedKeyFrame->m_mLocalR_Backup_wi,
		pMatchedKeyFrame->m_mLocalT_Backup_wi); 

	Sophus::SE3 mPose_ic(pMatchedKeyFrame->m_mRotation_ic, pMatchedKeyFrame->m_mTranslation_ic);
	Sophus::SE3 mPose_wc = mBackupPose_wi * mPose_ic;

	if (bFuse){
		this->m_pEstimator->FuseNewFrame(
			pMatchedKeyFrame->m_mImage, 
			mPose_wc);
		// this->m_pEstimator->LoadMaps(
		// 	this->m_mInvDepthMap, 
		// 	this->m_mInvCovMap, 
		// 	this->m_mConvergeMap);		
	}else{
		this->m_pEstimator->AddNewFrame(
			pMatchedKeyFrame->m_mImage, 
			mPose_wc);
	}

	this->m_mPoseMutex.unlock();
	this->m_mDepthMutex.unlock();
}


void ServerKeyFrame::FinalizeDepthMap(){
	this->m_mDepthMutex.lock();

	this->m_mPoseMutex.lock();
	// vector<cv::Point3d> gMapPoints;
	this->m_pEstimator->FinalizeMapPoints(this->m_gMapPoints, this->m_gMapPointsColor);
	this->m_pEstimator->LoadMaps(
		this->m_mInvDepthMap, 
		this->m_mInvCovMap, 
		this->m_mConvergeMap);
	this->m_bFinalizeDepthMap = true;

	

    // //Use sparse point cloud to thres the depth map
    // int nTotalCheckNumber = 0;
    // double nTotalCheckError = 0.0;
    // Eigen::Vector3d mOldRefTranslation_wc;
    // Eigen::Matrix3d mOldRefRotation_wc;
    // this->GetCameraPose(mOldRefTranslation_wc, mOldRefRotation_wc);
    // int nHeight = this->m_mImage.rows, nWidth = this->m_mImage.cols;
    // for (int i=0;i<this->m_gPoints2D.size();i++){
    //     cv::Point2d iUndistortedPoint2d = this->m_gNormalizedWindowPoints[i];
    //     Eigen::Vector3d mUndistortedPoint2d(iUndistortedPoint2d.x , iUndistortedPoint2d.y , 1.0);
    //     mUndistortedPoint2d = this->m_pServerCamera->GetK() * mUndistortedPoint2d;

    //     if ( mUndistortedPoint2d(0) < 2.5 || mUndistortedPoint2d(0) >  nWidth-2.5 || 
    //          mUndistortedPoint2d(1) < 2.5 || mUndistortedPoint2d(1) > nHeight-2.5){
    //         continue;
    //     }

    //     int nX = mUndistortedPoint2d(0), nY = mUndistortedPoint2d(1);
    //     double nDenseDepth = this->m_mInvDepthMap.at<double>(nY, nX);
    //     nDenseDepth = 1.0/nDenseDepth;
    //     if (nDenseDepth < 0.1 || nDenseDepth >= 100.0){
    //     	continue;
    //     }

    //     cv::Point3d iPoint3d = this->m_gPoints3D[i];
    //     Eigen::Vector3d mPoint3d(iPoint3d.x, iPoint3d.y, iPoint3d.z);
    //     Sophus::SE3 mPose_wc(mOldRefRotation_wc, mOldRefTranslation_wc);
    //     mPoint3d = mPose_wc.inverse() * mPoint3d;
    //     double nSparseDepth = mPoint3d(2);
    //     if (abs(nSparseDepth - nDenseDepth) >2){
    //     	for (int i=-18;i<=18;i++){
    //     		for (int j=-18;j<=18;j++){
    //     			if (nY+j < 1 || nY+j > nHeight-2 ||
    //     			 	nX+i < 1 || nX+i>nWidth){
    //     				continue;
    //     			}
    //     			if (abs(this->m_mInvDepthMap.at<double>(nY+j, nX+i) - nDenseDepth)<1.5){
    //     				this->m_mInvDepthMap.at<double>(nY+j, nX+i) = nSparseDepth;
    //     			}
    //     		}
    //     	}
    //     }
    // }

	this->m_mPoseMutex.unlock();
	this->m_mDepthMutex.unlock();
}




//The info published to open chisel.
void ServerKeyFrame::LoadRefInfo(	Sophus::SE3 & mRefPose_wc,
                    				std_msgs::Header & iHeader,
			                        cv::Mat & mDepthMap,
			                        cv::Mat & mColorMap,
			                        cv::Mat & mK,
			                        int & nWidth, 
			                        int & nHeight){
	Eigen::Matrix3d mRefRotation_wc;
	Eigen::Vector3d mRefTranslation_wc;
	this->GetCameraPose(mRefTranslation_wc, mRefRotation_wc);
	mRefPose_wc = Sophus::SE3(mRefRotation_wc, mRefTranslation_wc);
	iHeader = this->m_iHeader;
	mDepthMap = 1.0/this->m_mInvDepthMap;
	mColorMap = this->m_mUndistortedImage;
	Eigen::Matrix3d mK_eigen = this->m_pServerCamera->GetK();
	cv::eigen2cv(mK_eigen, mK);
	nWidth = mColorMap.cols;
	nHeight = mColorMap.rows;
}

bool ServerKeyFrame::FreeSpace(){
	cout << "Free space!" << endl; 
	this->m_mDepthMutex.lock();

	this->m_mPoseMutex.lock();
	if (!this->m_bFreeSpace){
		if (!this->m_bDepthRefFrame ){
			this->m_mColoredImage.release();

			this->m_mUndistortedImage.release();

			this->m_bFreeSpace = true;
		}else if (this->m_bFinalizeDepthMap){

			this->m_mInvDepthMap.release();
			// this->m_mImage.release();
			this->m_mColoredImage.release();

			this->m_mUndistortedImage.release();
			this->m_mInvCovMap.release();
			this->m_mConvergeMap.release();
			
			vector<cv::Point3d>().swap(this->m_gMapPoints);
			vector<cv::Point3d>().swap(this->m_gMapPointsColor);


			this->m_pEstimator->ReleaseSpaces();
			delete this->m_pEstimator;
			this->m_bFreeSpace = true;
		}
	}

	this->m_mDepthMutex.unlock();

	this->m_mPoseMutex.unlock();
	cout << "Free space!" << endl;
	return true;
	
}