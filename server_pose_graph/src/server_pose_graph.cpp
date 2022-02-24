#include "../include/server_pose_graph.h"
#include <iostream>
using namespace std;


ServerPoseGraph::ServerPoseGraph()
{
    cout << "Construction" << endl;
	//Initialize the visualizer.
	this->m_pPoseGraphVisualization = new ServerVisualization(1.0, 0.0, 1.0, 1.0);
	this->m_pPoseGraphVisualization->SetScale(0.1);
	this->m_pPoseGraphVisualization->SetLineWidth(0.01);
    
    //Initialize the optimization thread.
    //TODO: Not complemented yet!
    this->m_tOptimizationThread = std::thread(&ServerPoseGraph::Optimize4DoF, this);
	//t_optimization = std::thread(&PoseGraph::optimize4DoF, this);

	this->m_nEarliestLoopIndex = -1;
    this->m_nLastLoopIndex = -1;
    this->m_nGlobalIndex = 0;
    this->m_nFirstClient = -1;
    this->m_nLastOptimizationIndex = -1;

    this->m_nEarliestLoopIndex = 0;

    //TODO:Initialize the status of each client.
    // this->m_gIsLoopClosed

    this->m_pConnectionGraph = new ConnectionGraph(4);
}

//Deconstructor.
ServerPoseGraph::~ServerPoseGraph()
{
	this->m_tOptimizationThread.join();
}

//Relative Q is i_old i_current
void ServerPoseGraph::AlignSubMaps(ServerKeyFrame * pAlignedKeyFrame, ServerKeyFrame * pNotAlignedKeyFrame,
									Eigen::Matrix3d mRelativeQ, Eigen::Vector3d mRelativeT){
	//Old keyframe has been aligned with the world coordinate.
    ROS_WARN("Align %d sequence and %d sequence to world frame", pNotAlignedKeyFrame->m_nClientID, pAlignedKeyFrame->m_nClientID);


    for (int i=0;i<10;i++){
        cout << "Align sequence x to y: " << pNotAlignedKeyFrame->m_nClientID << "   " <<  pAlignedKeyFrame->m_nClientID << endl;
    }

    //Align the unaligned submap
    m_dAligned[pNotAlignedKeyFrame->m_nClientID] = 1;

    Eigen::Vector3d mGlobalAlignedT_wi, mGlobalNotAlignedT_wi, mLocalNotAlignedT_wi;
    Eigen::Matrix3d mGlobalAlignedR_wi, mGlobalNotAlignedR_wi, mLocalNotAlignedR_wi;

    pAlignedKeyFrame->GetVIOPose(mGlobalAlignedT_wi, mGlobalAlignedR_wi);
    pNotAlignedKeyFrame->GetVIOPose(mLocalNotAlignedT_wi, mLocalNotAlignedR_wi);

    //Relative Q is  I Not Aligned - I Aligned
    //Computed the aligned pose of the keyframe that is not aligned.
    //Compute the global pose of the keyframe that is not aligned.
    mGlobalNotAlignedT_wi = mGlobalAlignedR_wi * mRelativeT + mGlobalAlignedT_wi;
    mGlobalNotAlignedR_wi = mGlobalAlignedR_wi * mRelativeQ;

    double nShiftYaw = ServerUtility::R2ypr(mGlobalNotAlignedR_wi).x() - ServerUtility::R2ypr(mLocalNotAlignedR_wi).x();

    double nShiftRoll = ServerUtility::R2ypr(mGlobalNotAlignedR_wi).y() - ServerUtility::R2ypr(mLocalNotAlignedR_wi).y();

    double nShiftPitch = ServerUtility::R2ypr(mGlobalNotAlignedR_wi).z() - ServerUtility::R2ypr(mLocalNotAlignedR_wi).z();
    //Fix the other rotation.
    Eigen::Matrix3d mShiftRotation = ServerUtility::ypr2R(Eigen::Vector3d(nShiftYaw, 0.0, 0.0));
    Eigen::Vector3d mShiftTranslation = mGlobalNotAlignedT_wi - mGlobalNotAlignedR_wi * mLocalNotAlignedR_wi.transpose() * mLocalNotAlignedT_wi;

    // cout << "Shift Rotation is: " << endl  << mShiftRotation << endl;
    // cout << "Shift Translation is: " << endl << mShiftTranslation << endl;

    //Shift is from the not aligned to the world.
 //    this->m_dLocalRotation_wr[pNotAlignedKeyFrame->m_nClientID] = mShiftRotation;
 //    this->m_dLocalTranslation_wr[pNotAlignedKeyFrame->m_nClientID] = mShiftTranslation;

 //    //The global pose can't be sent to the keyframe since the updation is not of 4 DOF.
 //    mLocalNotAlignedT_wi = mShiftRotation * mLocalNotAlignedT_wi + mShiftTranslation;
	// mLocalNotAlignedR_wi = mShiftRotation *  mLocalNotAlignedR_wi;
	


    this->UpdateSubMaps(nShiftYaw, mShiftTranslation, pNotAlignedKeyFrame, pNotAlignedKeyFrame->m_nClientID);

 //    //Update the pose.
 //    ///Global Pose will also be updated.
	// pNotAlignedKeyFrame->UpdateVIOPose(mLocalNotAlignedT_wi, mLocalNotAlignedR_wi);

 //    this->InsertUpdatedKeyFrame(pNotAlignedKeyFrame);

	// //Update the pose in this submap.
 //    //The keyframes in this submap will all be updated.
	// for (ServerKeyFrame * pKeyFrame : this->m_gAllKeyFrames){
	// 	if (pKeyFrame->m_nClientID == pNotAlignedKeyFrame->m_nClientID && 
 //            pKeyFrame->m_nGlobalIndex != pNotAlignedKeyFrame->m_nGlobalIndex){
	// 		Eigen::Vector3d mLocalTranslation_wi;
	// 		Eigen::Matrix3d mLocalRotation_wi;
	// 		pKeyFrame->GetVIOPose(mLocalTranslation_wi, mLocalRotation_wi);
	// 		mLocalTranslation_wi = mShiftRotation * mLocalTranslation_wi + mShiftTranslation;
	// 		mLocalRotation_wi = mShiftRotation * mLocalRotation_wi;
	// 		pKeyFrame->UpdateVIOPose(mLocalTranslation_wi, mLocalRotation_wi);

 //            this->InsertUpdatedKeyFrame(pKeyFrame);
	// 	}
	// }
}

void ServerPoseGraph::RecordConnection(int nFirstLoopIndex, int nCurrentIndex){

    this->m_pConnectionGraph->ClearGraph();

    int nOptimizationIndex = 0;
    for (int k = nFirstLoopIndex; k < this->m_gAllKeyFrames.size(); k++)
    {   
        ServerKeyFrame * pKeyFrame = this->m_gAllKeyFrames[k];
        if (pKeyFrame->m_nGlobalIndex > nCurrentIndex){
            break;
        }

        nOptimizationIndex = k-nFirstLoopIndex;
        
        
        if (pKeyFrame->m_bHasLoop){
            if (pKeyFrame->m_bHasLoop && this->m_dAligned[pKeyFrame->m_nClientID] == 1 &&
                this->m_dAligned[this->GetKeyFrame(pKeyFrame->m_nLoopIndex)->m_nClientID] == 1){

                for (int li=0;li<pKeyFrame->m_gLoopIndices.size();li++){
                    //Check the index.
                    int nLoopIndex = pKeyFrame->m_gLoopIndices[li];

                    if(nLoopIndex < nFirstLoopIndex){
                        continue;
                    }

                    int nConnectedIndex = nLoopIndex - nFirstLoopIndex;

                    ServerKeyFrame * pConnectedKeyFrame = this->GetKeyFrame(nLoopIndex);
                    Eigen::Vector3d mRelativeT = pKeyFrame->GetLoopRelativeT(li);
                    double nRelativeYaw = pKeyFrame->GetLoopRelativeYaw(li);
                    Eigen::Matrix3d mRelativeRotation = ServerUtility::ypr2R(Eigen::Vector3d(nRelativeYaw, 0.0, 0.0));
                    Sophus::SE3 mRelativePose(mRelativeRotation, mRelativeT);
                    MapConnection * pConnection = new MapConnection();
                    pConnection->m_pKeyFrame1 = pConnectedKeyFrame;
                    pConnection->m_pKeyFrame2 = pKeyFrame;
                    pConnection->m_mTransform_12 = mRelativePose;
                    this->m_pConnectionGraph->AddConnection(pConnection);

                }   
            }
                

        }


    }

    // this->m_pConnectionGraph->ComputeGraph();


    // for (ServerKeyFrame * pKeyFrame : this->m_gAllKeyFrames){
    //     for (int i=0; i<pKeyFrame->m_gLoopIndices.size();i++){
    //         //Check the index.
    //         int nLoopIndex = pKeyFrame->m_gLoopIndices[i];
    //         ServerKeyFrame * pLoopKeyFrame = this->GetKeyFrame(nLoopIndex);
    //         if (pLoopKeyFrame == nullptr){
    //             continue;
    //         }

    //         Eigen::Matrix3d mRelativeR = (pKeyFrame->GetLoopRelativeQ()).toRotationMatrix();
    //         Eigen::Vector3d mRelativeT = pKeyFrame->GetLoopRelativeT();
    //         // loopframe_keyframe
    //         Sophus::SE3 mRelativePose_lk(mRelativeR, mRelativeT);

    //         MapConnection * pConnection = new MapConnection();
    //         pConnection->m_pKeyFrame1 = pLoopKeyFrame;
    //         pConnection->m_pKeyFrame2 = pKeyFrame;
    //         pConnection->m_mTransform_12 = mRelativePose_lk;

    //         this->m_pConnectionGraph->AddConnection(pConnection); 

    //     }
    // }

    this->m_pConnectionGraph->ComputeGraph();

    // // Eigen::Vector3d mT1_wi, mAlignedT2_wi, mT2_wi;
    // // Eigen::Matrix3d mR1_wi, mAlignedR2_wi, mR2_wi;

    // // //Now we need to get the poses of these two frames in the original CS.
    // // pKeyFrame1->GetVIOPose(mT1_wi, mR1_wi);
    // // pKeyFrame2->GetVIOPose(mT2_wi, mR2_wi);

    // // Sophus::SE3 mPose1_wi(mR1_wi, mT1_wi);
    // // Sophus::SE3 mPose2_wi(mR2_wi, mT2_wi);

    // // int nClientID1 = pKeyFrame1->m_nClientID;
    // // int nClientID2 = pKeyFrame2->m_nClientID;

    // // Sophus::SE3 mAgent1_wr(
    // //     this->m_dLocalRotation_wr[nClientID1],
    // //     this->m_dLocalTranslation_wr[nClientID1]);


    // // Sophus::SE3 mAgent2_wr(
    // //     this->m_dLocalRotation_wr[nClientID2],
    // //     this->m_dLocalTranslation_wr[nClientID2]);

    // // mPose1_wi = mAgent1_wr.inverse() * mPose1_wi;
    // // mPose1_wi = mAgent1_wr.inverse() * mPose2_wi;

    // // Sophus::SE3 mRelativePose_12(mRelativeQ_12, mRelativeT_12);
    // // Sophus::SE3 mAlignedPose2_wi = mPose1_wi * mRelativePose_12;

    // // mAlignedR2_wi = mAlignedPose2_wi.rotation_matrix();
    // // mAlignedT2_wi = mAlignedPose2_wi.translation();

    // // //Relative Q is  I Not Aligned - I Aligned
    // // //Computed the aligned pose of the keyframe that is not aligned.
    // // //Compute the global pose of the keyframe that is not aligned.
    // // // mAlignedT2_wi = mR1_wi * mRelativeT_12 + mT1_wi;
    // // // mAlignedR2_wi = mR1_wi * mRelativeQ_12;

    // // double nShiftYaw = ServerUtility::R2ypr(mAlignedR2_wi).x() - ServerUtility::R2ypr(mR2_wi).x();

    // // //Fix the other rotation.
    // // Eigen::Matrix3d mShiftRotation_12 = ServerUtility::ypr2R(Eigen::Vector3d(nShiftYaw, 0.0, 0.0));
    // // Eigen::Vector3d mShiftTranslation_12 = mAlignedT2_wi - mAlignedR2_wi * mR2_wi.transpose() * mT2_wi;

    // // Sophus::SE3 mRelativeShift_12(mShiftRotation_12, mShiftTranslation_12);

    // // MapConnection iConnection(nClientID1, nClientID2, mRelativeShift_12);
    // // this->m_pConnectionGraph->AddConnection(iConnection);
    // // this->m_pConnectionGraph->SaveGraph("/home/kyrie/Documents/DataSet/Connections/Connection.txt");
}


//nShiftYaw _  w2 w1
void ServerPoseGraph::UpdateSubMaps( 
    double nShiftYaw, 
    Eigen::Vector3d mRelativeT,
    ServerKeyFrame * pUpdateKeyFrame,
    int nClientID){

    Eigen::Matrix3d mRelativeRotation = ServerUtility::ypr2R(Eigen::Vector3d(nShiftYaw, 0.0, 0.0));
    this->m_dLocalRotation_wr[nClientID] = mRelativeRotation * this->m_dLocalRotation_wr[nClientID];
    this->m_dLocalTranslation_wr[nClientID] = mRelativeRotation * this->m_dLocalTranslation_wr[nClientID] + mRelativeT;

    Eigen::Vector3d mUpdateTranslation_wi;
    Eigen::Matrix3d mUpdateRotation_wi;

    pUpdateKeyFrame->GetVIOPose(mUpdateTranslation_wi, mUpdateRotation_wi);

    
    //The global pose can't be sent to the keyframe since the updation is not of 4 DOF.
    mUpdateTranslation_wi = mRelativeRotation * mUpdateTranslation_wi + mRelativeT;
    mUpdateRotation_wi = mRelativeRotation *  mUpdateRotation_wi;


    //Update the pose.
    ///Global Pose will also be updated.
    pUpdateKeyFrame->ShiftPose(mRelativeT, mRelativeRotation);


    for (ServerKeyFrame * pKeyFrame : this->m_gAllKeyFrames){
        if (pKeyFrame->m_nClientID == nClientID && 
            pKeyFrame->m_nGlobalIndex != pUpdateKeyFrame->m_nGlobalIndex){
            Eigen::Vector3d mLocalTranslation_wi;
            Eigen::Matrix3d mLocalRotation_wi;
            pKeyFrame->GetVIOPose(mLocalTranslation_wi, mLocalRotation_wi);
            mLocalTranslation_wi = mRelativeRotation * mLocalTranslation_wi + mRelativeT;
            mLocalRotation_wi = mRelativeRotation * mLocalRotation_wi;
            pKeyFrame->ShiftPose(mRelativeT, mRelativeRotation);
        }
    }

}


void ServerPoseGraph::RegisterClient(int nClientID){
	if (this->m_dAligned.empty()){
		//No clients yet!
		//World!
		this->m_dAligned[nClientID] = 1;
		this->m_nFirstClient = nClientID;
		// cout << "First sequence is: " << nClientID << endl;
	} else{
		//Not the first client;
		this->m_dAligned[nClientID] = 0;
	}
    this->m_dTranslationDrift[nClientID] = Eigen::Vector3d(0, 0, 0);
    this->m_dLocalTranslation_wr[nClientID] = Eigen::Vector3d(0, 0, 0);
    this->m_dRotationDrift[nClientID] = Eigen::Matrix3d::Identity();
    this->m_dLocalRotation_wr[nClientID] = Eigen::Matrix3d::Identity();

    this->m_dLocalIndices[nClientID] = 0;

    this->m_dKeyFramesEachMap[nClientID] = vector<ServerKeyFrame *>();
    

}


void ServerPoseGraph::AddKeyFrame(ServerKeyFrame* pKeyFrame){

    int64 nStartClock_Localization = cv::getTickCount();



    pKeyFrame->m_nGlobalIndex = this->m_nGlobalIndex;    
    this->m_nGlobalIndex++;

  // cout << "Client ID is: " << pKeyFrame->m_nClientID << endl;
    //Set the previous keyframe.
    pKeyFrame->m_pPreviousKeyFrame = NULL;
    ServerKeyFrame * pPreviousKeyFrame = this->GetPreviousKeyFrame(pKeyFrame);
    if (pPreviousKeyFrame != NULL){
        pKeyFrame->m_pPreviousKeyFrame = pPreviousKeyFrame;
        pPreviousKeyFrame->m_pNextKeyFrame = pKeyFrame;
        pKeyFrame->GenerateFG();
    }

    // ServerKeyFrame * pCovRefFrame = NULL;
    // int nPreviousNumber = 0;
    // for (int i= this->m_gAllKeyFrames.size()-1;i>=0;i--){
    //     if (this->m_gAllKeyFrames[i]->m_nClientID == pKeyFrame->m_nClientID){
    //         pCovRefFrame = this->m_gAllKeyFrames[i];
    //         nPreviousNumber +=1;
    //         if (nPreviousNumber >=5){
    //             break;
    //         }
    //     }
    // }
    // if (pCovRefFrame != NULL){
    //     pKeyFrame->ComputeCov(pCovRefFrame);
    // }
    



    cout << "KeyFrame size: "  << this->m_gAllKeyFrames.size() << endl;


    //Find the previous keyframe.

    // //Show the image in the keyframe.
    // cv::Mat mShownImage = pKeyFrame->m_mImage.clone();
    // for (cv::Point2f iPoint : pKeyFrame->m_gPoints2D){
    //     cv::circle(mShownImage, iPoint, 3, cv::Scalar(100 , 0 , 0));
    // }
    // stringstream ss;
    // ss << pKeyFrame->m_nClientID;
    // string aWindowName = "";
    // ss >> aWindowName;
    // aWindowName = aWindowName + "_window";
    // cv::imshow(aWindowName, mShownImage);
    // cv::waitKey(5);


   

	//The client id of this keyframe.
	int nClientID = pKeyFrame->m_nClientID;

	if (!this->m_dAligned.count(nClientID)){
		//This is a new client.
		this->RegisterClient(nClientID);
	}


    if (this->m_dAligned[pKeyFrame->m_nClientID]){
        pKeyFrame->m_bHasBeenAligned = true;
    }


    //Set the local index.
    pKeyFrame->m_nLocalIndex = this->m_dLocalIndices[nClientID];
    this->m_dLocalIndices[nClientID]++;

    stringstream ss1;
    ss1 << pKeyFrame->m_nGlobalIndex;
    string aIndex1;
    ss1 >> aIndex1;
    string aImageFilename = "/home/kyrie/Documents/DataSet/CoVins/Loop/image_"+  aIndex1   + ".jpg";
    string aPoseFilename = "/home/kyrie/Documents/DataSet/CoVins/Loop/pose_"+  aIndex1   + ".txt";
                

    Eigen::Vector3d mCurrTranslation_wc;
    Eigen::Matrix3d mCurrRotation_wc;
    pKeyFrame->GetCameraPose(mCurrTranslation_wc, mCurrRotation_wc);
    Sophus::SE3 mPose_wc(mCurrRotation_wc, mCurrTranslation_wc);
    
    // cv::imwrite(aImageFilename, pKeyFrame->m_mImage);
    // ofstream fOut(aPoseFilename);
    // fOut << mPose_wc;    





    std::chrono::milliseconds nDura(1);
    std::this_thread::sleep_for(nDura);


	//Convert the pose of this keyframe to the world coordinate system.
	Eigen::Vector3d mLocalTranslation_wi;
    Eigen::Matrix3d mLocalRotation_wi;
    pKeyFrame->GetVIOPose(mLocalTranslation_wi, mLocalRotation_wi);
    Eigen::Vector3d mGlobalTranslation_wi = m_dLocalRotation_wr[nClientID] * mLocalTranslation_wi + m_dLocalTranslation_wr[nClientID];
    Eigen::Matrix3d mGlobalRotation_wi = m_dLocalRotation_wr[nClientID] * mLocalRotation_wi;
    pKeyFrame->UpdateVIOPose(mGlobalTranslation_wi, mGlobalRotation_wi);

    //Loop detection.
    int nLoopIndex = -1;
    nLoopIndex = this->DetectLoop(pKeyFrame, pKeyFrame->m_nGlobalIndex);





    bool bFindConnection = false;
    bool bNeedUpdatePath = false;

    int nPublishDepth = -1;

    if (nLoopIndex != -1 && (this->m_nLastLoopIndex == -1 || pKeyFrame->m_nGlobalIndex - m_nLastLoopIndex > 10)){
    	
    	//The loop closure has been detected.
    	ServerKeyFrame * pOldKeyFrame = nullptr;
        pOldKeyFrame = this->GetKeyFrame(nLoopIndex);
    	// pOldKeyFrame = this
    	// if (pOldKeyFrame != nullptr){
    	// 	//TODO:KeyFrame not found!!!
    		
    	// }




        if (pOldKeyFrame->m_nGlobalIndex +100 < pKeyFrame->m_nGlobalIndex || (pOldKeyFrame->m_nClientID != pKeyFrame->m_nClientID) ||
            (this->m_dAligned[pOldKeyFrame->m_nClientID] !=  this->m_dAligned[pKeyFrame->m_nClientID]
             && (this->m_dAligned[pOldKeyFrame->m_nClientID] ==1 || this->m_dAligned[pKeyFrame->m_nClientID] ==1)))
        {



            //Find connections between the matched keyframe and this keyframe.
            //Then the relative pose between pKeyFrame and pOldKeyFrame has been stored.
            if (this->m_dAligned[pOldKeyFrame->m_nClientID] !=  this->m_dAligned[pKeyFrame->m_nClientID]){
                bFindConnection = pKeyFrame->FindConnection(pOldKeyFrame, true);    
            }else{
                bFindConnection = pKeyFrame->FindConnection(pOldKeyFrame, false);
            }
            
            if (bFindConnection){


                this->m_nLastLoopIndex = pKeyFrame->m_nGlobalIndex;
                //Set the earliest loop index.
                //Update the earliest loop index.
                if (this->m_nEarliestLoopIndex > nLoopIndex || 
                    this->m_nEarliestLoopIndex == -1){
                    this->m_nEarliestLoopIndex = nLoopIndex;
                }
                cv::Mat mMatchedImage = pKeyFrame->m_mMatchedImage;
                this->m_pPlotter->UpdateLoopClosureImage(mMatchedImage);

                //Not in the same map
                if (pOldKeyFrame->m_nClientID != pKeyFrame->m_nClientID){
                    int nUpdate = 0;
                    // cout << "Enter loop closure" << endl;
                    //Intra loop closure.
                    if (m_dAligned[pKeyFrame->m_nClientID] == 0 && 
                        m_dAligned[pOldKeyFrame->m_nClientID] == 1){
                        Eigen::Matrix3d mRelativeQ = (pKeyFrame->GetLoopRelativeQ()).toRotationMatrix();
                        Eigen::Vector3d mRelativeT = pKeyFrame->GetLoopRelativeT();
                        //Align current submap to the world coordinate system.
                        this->AlignSubMaps(pOldKeyFrame, pKeyFrame, mRelativeQ, mRelativeT);
                        //The path needs to be updated.
                        bNeedUpdatePath = true;
                        nUpdate = 1;
                        
                        nPublishDepth = pKeyFrame->m_nClientID;
                    } else if ( m_dAligned[pKeyFrame->m_nClientID] == 1 && 
                                m_dAligned[pOldKeyFrame->m_nClientID] == 0){
                        //Align the old submap to the world coordinate system.
                        Eigen::Vector3d mInverseRelativeT = pKeyFrame->GetLoopRelativeT();
                        Eigen::Quaterniond mInverseRelativeQ = pKeyFrame->GetLoopRelativeQ();
                        Eigen::Matrix3d mRotationMatrix = mInverseRelativeQ.toRotationMatrix();

                        // cout << "Relative T is: " << mInverseRelativeT << endl;
                        // cout << "Relative Q is: " << mRotationMatrix << endl;

                        Eigen::Matrix3d mRelativeQ = mRotationMatrix.inverse();
                        Eigen::Vector3d mRelativeT = - (mRotationMatrix * mInverseRelativeT);

                        // cout << "Finish computation" << endl;
                        // cout << "Relative T is: " << mRelativeT << endl;
                        // cout << "Relative Q is: " << mRelativeQ << endl;
                        
                        //Align old submap to the world coordinate system.
                        this->AlignSubMaps(pKeyFrame, pOldKeyFrame, mRelativeQ, mRelativeT);
                        //The path needs to be updated.
                        bNeedUpdatePath = true;
                        nUpdate = 2;

                        nPublishDepth = pOldKeyFrame->m_nClientID;

                    } else if ( m_dAligned[pKeyFrame->m_nClientID] == 0 && 
                                m_dAligned[pOldKeyFrame->m_nClientID] == 0){
                        //Not processed yet!
                    }



                    //Find loop closure again.
                    ServerKeyFrame * pUpdateKeyFrame = NULL;
                    if (nUpdate == 1){
                        pUpdateKeyFrame = pKeyFrame;
                    }else if (nUpdate == 2){
                        pUpdateKeyFrame = pOldKeyFrame;
                    }

                    // //Find connections.
                    // vector<ServerKeyFrame *> gCurrentKeyFrames, gOldKeyFrames;
                    // ServerKeyFrame * pCurrentKeyFrame = pKeyFrame;
                    // ServerKeyFrame * pOldConnectKeyFrame = pOldKeyFrame;
                    // cout << "Bind connection!" << endl;
                    // for (int i=0;i<3;i++){
                    //     if (pCurrentKeyFrame != NULL){
                    //         gCurrentKeyFrames.push_back(pCurrentKeyFrame);    
                    //         pCurrentKeyFrame = pCurrentKeyFrame->m_pPreviousKeyFrame;
                    //     }

                    //     if (pOldConnectKeyFrame != NULL){
                    //         gOldKeyFrames.push_back(pOldConnectKeyFrame);
                    //         pOldConnectKeyFrame = pOldConnectKeyFrame->m_pPreviousKeyFrame;
                    //     }
                    // }

                    // for (int i=0;i<gCurrentKeyFrames.size();i++){
                    //     for (int j=0;j<gOldKeyFrames.size();j++){
                    //         ServerKeyFrame * pCurr = gCurrentKeyFrames[i];
                    //         ServerKeyFrame * pOld = gOldKeyFrames[i];
                    //         // if ((!pCurr->m_bHasLoop) && (!pOld->m_bHasLoop)){
                            
                    //             bool bFind = pCurr->FindConnection(pOld);
                    //             if (bFind){
                    //                 int nUpdateLoopIndex = pOld->m_nGlobalIndex;
                    //                 if (this->m_nEarliestLoopIndex > nUpdateLoopIndex || 
                    //                     this->m_nEarliestLoopIndex == -1){
                    //                     this->m_nEarliestLoopIndex = nUpdateLoopIndex;
                    //                 }
                    //             }

                    //         }
                    //     }


                    // while (pUpdateKeyFrame != NULL){
                    //     // if (!pUpdateKeyFrame->m_bHasLoop){
                    //     {
                    //         Eigen::Vector3d mCurrentTranslation;
                    //         Eigen::Matrix3d mCurrentRotation;
                    //         pUpdateKeyFrame->GetCameraPose(mCurrentTranslation, mCurrentRotation);



                    //         int nUpdateLoopIndex = this->DetectLoop(pUpdateKeyFrame, pUpdateKeyFrame->m_nGlobalIndex, false);
                    //         if (nUpdateLoopIndex != -1){
                    //             ServerKeyFrame * pUpdatedOldKeyFrame = NULL;
                    //             pUpdatedOldKeyFrame = this->GetKeyFrame(nUpdateLoopIndex);
                    //             if (pUpdatedOldKeyFrame != NULL){
                    //                 bool bFind = pUpdateKeyFrame->FindConnection(pUpdatedOldKeyFrame);
                    //                 if (bFind){
                    //                      if (this->m_nEarliestLoopIndex > nUpdateLoopIndex || 
                    //                             this->m_nEarliestLoopIndex == -1){
                    //                             this->m_nEarliestLoopIndex = nUpdateLoopIndex;
                    //                     }
                    //                 }

                    //             }  

                    //         }else{
                    //             for (ServerKeyFrame * pCovisibleKeyFrame : this->m_gAllKeyFrames){
                    //                 if (pCovisibleKeyFrame->m_nClientID == pUpdateKeyFrame->m_nClientID){
                    //                     continue;
                    //                 }
                    //                 Eigen::Vector3d mCovisibleTranslation;
                    //                 Eigen::Matrix3d mCovisibleRotation;
                    //                 pCovisibleKeyFrame->GetCameraPose(mCovisibleTranslation, mCovisibleRotation);
                    //                 Sophus::SE3 mRelativePose = Sophus::SE3(mCurrentRotation, mCurrentTranslation).inverse() * Sophus::SE3(mCovisibleRotation, mCovisibleTranslation);
                    //                 Eigen::Matrix3d mRelativeRotation = mRelativePose.rotation_matrix();
                    //                 Eigen::Vector3d mRelativeTranslation = mRelativePose.translation();

                    //                 Eigen::AngleAxisd mRotationVec;
                    //                 mRotationVec.fromRotationMatrix(mRelativeRotation);
                    //                 double nRotationRadian = mRotationVec.angle();

                    //                 if (nRotationRadian < 0.5 && mRelativeTranslation.norm() < 1){
                    //                     bool bFind = pUpdateKeyFrame->FindConnection(pCovisibleKeyFrame);
                    //                     if (bFind){
                    //                         if (this->m_nEarliestLoopIndex > pCovisibleKeyFrame->m_nGlobalIndex || 
                    //                             this->m_nEarliestLoopIndex == -1){
                    //                             this->m_nEarliestLoopIndex = pCovisibleKeyFrame->m_nGlobalIndex;
                    //                         }
                    //                         break;
                    //                     }
                    //                 }
                    //             }
                    //         }
                    //     }
                    //     pUpdateKeyFrame = pUpdateKeyFrame->m_pPreviousKeyFrame;
                    // }


                }else{
                //Inter loop closure.
                //Not processed yet.
                //If they are in the same map, no need to align now.

                    //Find connections.
                    vector<ServerKeyFrame *> gCurrentKeyFrames, gOldKeyFrames;
                    ServerKeyFrame * pCurrentKeyFrame = pKeyFrame;
                    ServerKeyFrame * pOldConnectKeyFrame = pOldKeyFrame;
                    gCurrentKeyFrames.push_back(pCurrentKeyFrame);
                    gOldKeyFrames.push_back(pOldConnectKeyFrame);

                    // for (int i=0;i<3;i++){
                    //     if (pCurrentKeyFrame != NULL){
                    //         gCurrentKeyFrames.push_back(pCurrentKeyFrame);    
                    //         pCurrentKeyFrame = pCurrentKeyFrame->m_pPreviousKeyFrame;
                    //     }

                    //     if (pOldConnectKeyFrame != NULL){
                    //         gOldKeyFrames.push_back(pOldConnectKeyFrame);
                    //         pOldConnectKeyFrame = pOldConnectKeyFrame->m_pPreviousKeyFrame;
                    //     }
                    // }

                    for (int i=0;i<gCurrentKeyFrames.size();i++){
                        for (int j=0;j<gOldKeyFrames.size();j++){
                            ServerKeyFrame * pCurr = gCurrentKeyFrames[i];
                            ServerKeyFrame * pOld = gOldKeyFrames[i];
                            if ((!pCurr->m_bHasLoop) && (!pOld->m_bHasLoop)){
                            // {
                                bool bFind = pCurr->FindConnection(pOld);
                                if (bFind){
                                    int nUpdateLoopIndex = pOld->m_nGlobalIndex;
                                    if (this->m_nEarliestLoopIndex > nUpdateLoopIndex || 
                                        this->m_nEarliestLoopIndex == -1){
                                        this->m_nEarliestLoopIndex = nUpdateLoopIndex;
                                    }
                                }

                            }
                        }
                    }


                }

            } 
        }


    }else{
        //Check covisible.
        if (this->m_dAligned[pKeyFrame->m_nClientID] == 1 ){
            Eigen::Vector3d mCurrentTranslation;
            Eigen::Matrix3d mCurrentRotation;
            pKeyFrame->GetCameraPose(mCurrentTranslation, mCurrentRotation);
            for (ServerKeyFrame * pCovisibleKeyFrame : this->m_gAllKeyFrames){
                if (pCovisibleKeyFrame->m_nClientID == pKeyFrame->m_nClientID){
                    continue;
                }
                Eigen::Vector3d mCovisibleTranslation;
                Eigen::Matrix3d mCovisibleRotation;
                pCovisibleKeyFrame->GetCameraPose(mCovisibleTranslation, mCovisibleRotation);
                Sophus::SE3 mRelativePose = Sophus::SE3(mCurrentRotation, mCurrentTranslation).inverse() * Sophus::SE3(mCovisibleRotation, mCovisibleTranslation);
                Eigen::Matrix3d mRelativeRotation = mRelativePose.rotation_matrix();
                Eigen::Vector3d mRelativeTranslation = mRelativePose.translation();

                Eigen::AngleAxisd mRotationVec;
                mRotationVec.fromRotationMatrix(mRelativeRotation);
                double nRotationRadian = mRotationVec.angle();

                if (nRotationRadian < 0.5 && mRelativeTranslation.norm() < 1){
                    bool bFind = pKeyFrame->FindConnection(pCovisibleKeyFrame);
                    if (bFind){
                        if (this->m_nEarliestLoopIndex > pCovisibleKeyFrame->m_nGlobalIndex || 
                            this->m_nEarliestLoopIndex == -1){
                            this->m_nEarliestLoopIndex = pCovisibleKeyFrame->m_nGlobalIndex;
                        }
                        break;
                    }
                }
            }
        }
    }



        printf("Cost Loop is: %fms\n", (cv::getTickCount() - nStartClock_Localization)/ cv::getTickFrequency() * 1000);



    // if (nPublishDepth != -1){

    //     this->m_mKeyFrameListMutex.lock();
    //     cout << "Publish" << endl;
    //     for (ServerKeyFrame * pKeyFrame : this->m_gAllKeyFrames){
    //         if (pKeyFrame->m_nClientID == nPublishDepth){
    //             this->m_gDenseKeyFrames.push(pKeyFrame);
    //         }
    //     }
    //     cout << "Finish publish" << endl;

    //     this->m_mKeyFrameListMutex.unlock();
    // }

    std::this_thread::sleep_for(nDura);

    // this->RecordConnection();

    //Add the keyframe to the list.
    this->m_mKeyFrameListMutex.lock();

    Eigen::Vector3d mTranslation_wi;
    Eigen::Matrix3d mRotation_wi;

    pKeyFrame->GetVIOPose(mTranslation_wi, mRotation_wi);
    
    //Obtained by the previous keyframe
    //Update the drift.
    mTranslation_wi = m_dRotationDrift[nClientID] * mTranslation_wi + m_dTranslationDrift[nClientID];
    mRotation_wi = m_dRotationDrift[nClientID] * mRotation_wi;

    //Update the pose.
    pKeyFrame->UpdatePose(mTranslation_wi, mRotation_wi);

    //Publish the info.
    Eigen::Quaterniond mQuaterniond{mRotation_wi};

    //Save the path info.
    // geometry_msgs::PoseStamped iPoseStamped;
    // iPoseStamped.header.stamp = ros::Time(pKeyFrame->m_nTimeStamp);
    // iPoseStamped.header.frame_id = "world";
    // iPoseStamped.pose.position.x = mTranslation_wi.x() + SERVER_VISUALIZATION_SHIFT_X;
    // iPoseStamped.pose.position.y = mTranslation_wi.y() + SERVER_VISUALIZATION_SHIFT_Y;
    // iPoseStamped.pose.position.z = mTranslation_wi.z();
    // iPoseStamped.pose.orientation.x = mQuaterniond.x();
    // iPoseStamped.pose.orientation.y = mQuaterniond.y();
    // iPoseStamped.pose.orientation.z = mQuaterniond.z();
    // iPoseStamped.pose.orientation.w = mQuaterniond.w();
    // m_gPaths[nClientID].poses.push_back(iPoseStamped);
    // m_gPaths[nClientID].header = iPoseStamped.header;




    std::this_thread::sleep_for(nDura);

    
    //Add the keyframe to the list.
    this->m_gAllKeyFrames.push_back(pKeyFrame);
    this->m_dKeyFramesEachMap[nClientID].push_back(pKeyFrame);
    m_mKeyFrameListMutex.unlock();
    


        printf("Cost Localization is: %fms\n", (cv::getTickCount() - nStartClock_Localization)/ cv::getTickFrequency() * 1000);



    int64 nStartClock = cv::getTickCount();
    //Dense mapping
    if (m_dAligned[pKeyFrame->m_nClientID] == 1){
    if (!this->m_dMappingRefKeyFrames.count(nClientID)){
        //This is the first ref frame
        //TODO:Add more selection strategies
        // cout << "Initialize depth estimator" << endl;
        pKeyFrame->InitializeDepthEstimator();

        // cout << "Finish initialize depth estimator" << endl;
        this->m_dMappingRefKeyFrames[nClientID] = pKeyFrame;


        int nPreviousNum = 0;
        ServerKeyFrame * pPreviousKeyFrame = GetPreviousKeyFrame(pKeyFrame);
        while (nPreviousNum<2 && pPreviousKeyFrame!= NULL){
            pKeyFrame->FuseFrame(pPreviousKeyFrame);
            pPreviousKeyFrame = GetPreviousKeyFrame(pPreviousKeyFrame);
            nPreviousNum++;
        }
        ServerKeyFrame * pOldRefFrame = pKeyFrame;
       


    }else { 
        int nCurrentLocalIndex = pKeyFrame->m_nLocalIndex;
        ServerKeyFrame * pRefKeyFrame = this->m_dMappingRefKeyFrames[nClientID]; 
        int nRefLocalIndex = pRefKeyFrame->m_nLocalIndex;
        ServerKeyFrame * pOldRefFrame = this->m_dMappingRefKeyFrames[nClientID];
        
        pOldRefFrame->FuseFrame(pKeyFrame);


        printf("Cost Dense Mapping1 is: %fms\n", (cv::getTickCount() - nStartClock)/ cv::getTickFrequency() * 1000);

        if ( nCurrentLocalIndex - nRefLocalIndex >=2){
            pOldRefFrame->FinalizeDepthMap();

        printf("Cost Dense Mapping2 is: %fms\n", (cv::getTickCount() - nStartClock)/ cv::getTickFrequency() * 1000);

            // cout << "Begin to initialize" << endl;

                cv::Mat mDepthMap =  255 * 0.1/pOldRefFrame->m_mInvDepthMap;
                cv::Mat mDepthCovMap = 10.0 * pOldRefFrame->m_mInvCovMap;
                
                stringstream ss;
                ss << pOldRefFrame->m_nGlobalIndex;
                string aIndex;
                ss >> aIndex;

                string aFilename = "/home/kyrie/Documents/DataSet/CoVins/Mean/depth_" + aIndex + ".jpg";
                string aOriginalFilename = "/home/kyrie/Documents/DataSet/CoVins/Mean/undistorted_" + aIndex + ".jpg";
               
                string aUndistortedFilename = "/home/kyrie/Documents/DataSet/CoVins/Mean/original_" + aIndex + ".jpg";
                string aErrorPath = "/home/kyrie/Documents/DataSet/CoVins/Mean/error_" + aIndex + ".txt";
                cv::Mat mSaveImage = pOldRefFrame->m_mImage.clone();
                
                

                //Check the depth error.
                double nTotalCheckError = 0.0;
                int nTotalCheckNumber = 0;
                int nWidth = mDepthMap.cols, nHeight = mDepthMap.rows;
                Eigen::Vector3d mOldRefTranslation_wc;
                Eigen::Matrix3d mOldRefRotation_wc;
                pOldRefFrame->GetCameraPose(mOldRefTranslation_wc, mOldRefRotation_wc);
                Sophus::SE3 mPose_wc(mOldRefRotation_wc, mOldRefTranslation_wc);
                for (int i=0;i<pOldRefFrame->m_gPoints2D.size();i++){
                    cv::Point2d iPoint2d = pOldRefFrame->m_gPoints2D[i];
                    cv::Point2d iUndistortedPoint2d = pOldRefFrame->m_gNormalizedWindowPoints[i];
                    Eigen::Vector3d mUndistortedPoint2d(iUndistortedPoint2d.x , iUndistortedPoint2d.y , 1.0);
                    mUndistortedPoint2d = pOldRefFrame->m_pServerCamera->GetK() * mUndistortedPoint2d;

                    if ( mUndistortedPoint2d(0) < 20.5 || mUndistortedPoint2d(0) > nWidth-20.5 || 
                         mUndistortedPoint2d(1) < 20.5 || mUndistortedPoint2d(1) > nHeight-20.5){
                        continue;
                    }

                    int nX = mUndistortedPoint2d(0), nY = mUndistortedPoint2d(1);
                    cv::Point3d iPoint3d = pOldRefFrame->m_gPoints3D[i];
                    Eigen::Vector3d mPoint3d(iPoint3d.x, iPoint3d.y, iPoint3d.z);
                    
                    mPoint3d = mPose_wc.inverse() * mPoint3d;
                    double nSparseDepth = mPoint3d(2);
                    double nDenseDepth = pOldRefFrame->m_mInvDepthMap.at<double>(nY, nX);
                    nDenseDepth = 1.0/nDenseDepth;

                    if (nDenseDepth > 0.1 && nDenseDepth < 100){
                        nTotalCheckNumber++;
                        nTotalCheckError += abs(nSparseDepth- nDenseDepth);
                    }
                }
                if (nTotalCheckNumber >0){
                    nTotalCheckError /= nTotalCheckNumber;
                }

                // cv::Mat mLaplacian;
                // cv::Laplacian(pOldRefFrame->m_mImage, mLaplacian, int ddepth)


                pKeyFrame->InitializeDepthEstimator();

                //Publish
                if (m_dAligned[pOldRefFrame->m_nClientID] == 1){

                    if (pOldRefFrame->m_nLocalIndex >= 5){
                        cv::Mat mDepthMapWrite = mDepthMap.clone();
                        // cv::imwrite(aFilename, mDepthMapWrite);
                        
                        // cv::imwrite(aOriginalFilename, mSaveImage);
                        // ofstream fOut(aErrorPath);
                        // fOut << mPose_wc;
                        // pKeyFrame->PropogateDepthFilter(pOldRefFrame);
                        this->m_gDenseKeyFrames.push(pOldRefFrame);
                    }
                }else{
                    pOldRefFrame->FreeSpace();
                }


    printf("Cost Dense Mapping3 is: %fms\n", (cv::getTickCount() - nStartClock)/ cv::getTickFrequency() * 1000);

                // cout << "Finish to initialize" << endl;
                this->m_dMappingRefKeyFrames[nClientID] = pKeyFrame;

    printf("Cost Dense Mapping4 is: %fms\n", (cv::getTickCount() - nStartClock)/ cv::getTickFrequency() * 1000);


                int nPreviousNum = 0;
                ServerKeyFrame * pPreviousKeyFrame = GetPreviousKeyFrame(pKeyFrame);
                // while (nPreviousNum<2 && pPreviousKeyFrame!= NULL){
                //     pKeyFrame->FuseFrame(pPreviousKeyFrame);
                //     pPreviousKeyFrame = GetPreviousKeyFrame(pPreviousKeyFrame);
                //     nPreviousNum++;
                // }

    printf("Cost Dense Mapping5 is: %fms\n", (cv::getTickCount() - nStartClock)/ cv::getTickFrequency() * 1000);

            }
        }
    }


    printf("Cost Dense Mapping is: %fms\n", (cv::getTickCount() - nStartClock)/ cv::getTickFrequency() * 1000);

    if (bNeedUpdatePath)
    {
    	//TODO:Update Path!
        // updatePath();
    }
    
    //TODO:Publish not implemented!
    // publish();

    if (this->m_gAllKeyFrames.size() > 2000){
        this->AddDisturbance();
    }


    if (bFindConnection)
    {
        this->m_mOptimizationBufMutex.lock();
        this->m_gOptimizationBuf.push(pKeyFrame);
        // cout << "Add optimzation frame" << endl;
        this->m_mOptimizationBufMutex.unlock();
    }

    //This keyframe will be updated in the future.
    this->InsertUpdatedKeyFrame(pKeyFrame);

    this->m_pPlotter->AddKeyFrame(pKeyFrame);
    cout << "Finish add keyframe" << endl;
    std::this_thread::sleep_for(nDura);
}



ServerKeyFrame * ServerPoseGraph::PopDepthFrames(){
    if (this->m_gDenseKeyFrames.empty()){
        return NULL;
    }else{
        ServerKeyFrame * pResult = this->m_gDenseKeyFrames.front();
        if ( 
            this->m_gAllKeyFrames.size() < 15){
            return NULL;
        }
        this->m_gDenseKeyFrames.pop();
        return pResult;
    }
}


int ServerPoseGraph::DetectLoop(ServerKeyFrame* pKeyFrame, 
                                int nGlobalFrameIndex,
                                bool bNewKeyFrame)
{
    DBoW2::QueryResults iQueryResult;
    //Window is point, all is feature.
    //Search in the database.
    this->m_iBriefDB.query(pKeyFrame->m_gBriefDescriptors, iQueryResult, 4 , nGlobalFrameIndex-10);

    //Add to the database.
    if (bNewKeyFrame){
        this->AddKeyFrameIntoVOC(pKeyFrame);    
    }
    
    
    //If the loop closure has been found.
    bool bFindLoop = false;
    
    cv::Mat mLoopResult;

    //Check the quality of the loop closure.
    if (iQueryResult.size() >=1){
        // if (iQueryResult[0].Score > 0.003){
            for (unsigned int i = 1; i < iQueryResult.size(); i++)
            {
                if (iQueryResult[i].Score > 0.003){
                    bFindLoop = true;
                }
            }    
        // }
    }

    //If multiple results are found, return the oldest one.
    if (bFindLoop && m_nGlobalIndex > 3){


        int nMinIndex = -1;
        for (unsigned int i=0;i<iQueryResult.size();i++){
            ServerKeyFrame * pMatchedKeyFrame = this->GetKeyFrame(iQueryResult[i].Id);
            if (iQueryResult[i].Id >= this->m_gAllKeyFrames.size()){
                continue;
            }
            if ((nMinIndex == -1 || iQueryResult[i].Id < nMinIndex) )
            {
                if (bNewKeyFrame){

                    if (pKeyFrame->m_nClientID != pMatchedKeyFrame->m_nClientID && 
                        (m_dAligned[pKeyFrame->m_nClientID] == 1 || m_dAligned[pMatchedKeyFrame->m_nClientID] == 1) &&
                         m_dAligned[pKeyFrame->m_nClientID] != m_dAligned[pMatchedKeyFrame->m_nClientID]){
                        
                        if (iQueryResult[i].Score > 0.003){
                            nMinIndex = iQueryResult[i].Id;   
                        }
                    }

                }else{
                    if ((m_dAligned[pKeyFrame->m_nClientID] == 1 && 
                         m_dAligned[pMatchedKeyFrame->m_nClientID] == 1) || 
                         pKeyFrame->m_nClientID == pMatchedKeyFrame->m_nClientID){
                        if (iQueryResult[i].Score > 0.005){
                            nMinIndex = iQueryResult[i].Id;
                        }
                    }

                }
            }
        }
        if (nMinIndex == -1){
            for (unsigned int i=0;i<iQueryResult.size();i++){
                if (iQueryResult[i].Id >= this->m_gAllKeyFrames.size()){
                    continue;
                }
                ServerKeyFrame * pMatchedKeyFrame = this->GetKeyFrame(iQueryResult[i].Id);
                if ((nMinIndex == -1 || iQueryResult[i].Id < nMinIndex) )
                {
                    if ((m_dAligned[pKeyFrame->m_nClientID] == 1 && 
                         m_dAligned[pMatchedKeyFrame->m_nClientID] == 1) || 
                         pKeyFrame->m_nClientID == pMatchedKeyFrame->m_nClientID){
                        
                        if (iQueryResult[i].Score > 0.005){
                            nMinIndex = iQueryResult[i].Id;
                        }
                    }
                }
            }
        }

        return nMinIndex;   //This is the global pose.
    }

    return -1;
}



ServerKeyFrame* ServerPoseGraph::GetKeyFrame(int nGlobalIndex)
{
    if (nGlobalIndex >= (int)this->m_gAllKeyFrames.size())
        return nullptr;
    else
        return this->m_gAllKeyFrames[nGlobalIndex];
}


//Find the previous keyframe from the same client with the input keyframe.
ServerKeyFrame * ServerPoseGraph::GetPreviousKeyFrame(ServerKeyFrame * pKeyFrame){
    int nClientID = pKeyFrame->m_nClientID;
    int nId = pKeyFrame->m_nGlobalIndex;

    for (int i=nId-1;i>=0;i--){
        ServerKeyFrame * pPreviousKeyFrame = this->m_gAllKeyFrames[i];
        if (pPreviousKeyFrame->m_nClientID == nClientID){
            return pPreviousKeyFrame;
        }
    }
    return NULL;
}


void ServerPoseGraph::AddKeyFrameIntoVOC(ServerKeyFrame * pKeyFrame){
    this->m_iBriefDB.add(pKeyFrame->m_gBriefDescriptors);
}



Sophus::SE3 ConvertPoseToSophus(Vector6d mPose){
    Eigen::Matrix3d mRotation_wc;
    YawPitchRollToRotationMatrixDouble(mPose(0 , 0), mPose(1 , 0), mPose(2 , 0), mRotation_wc);
    Eigen::Vector3d mTranslation_wc(
            mPose(3 , 0),
            mPose(4 , 0),
            mPose(5 , 0)); 
    Sophus::SE3 mPoseSophus(mRotation_wc, mTranslation_wc);
    return mPoseSophus;
}

void ServerPoseGraph::Optimize4DoF()
{
    ofstream fOutFile("/home/kyrie/Documents/DataSet/CoVins/Optimization/smooth.txt");

    while (true){

        //Load keyframe from the optimization buf.
        ServerKeyFrame * pOptimizationKeyFrame = nullptr;
        int nCurrentIndex = -1;
        int nFirstLoopIndex = -1;
        
        //Load the Last frame from the buf
        //Lock firstly.
        this->m_mOptimizationBufMutex.lock();
        while(!this->m_gOptimizationBuf.empty()) {
            pOptimizationKeyFrame = this->m_gOptimizationBuf.front();
            nCurrentIndex = pOptimizationKeyFrame->m_nGlobalIndex;
            this->m_nLastOptimizationIndex = nCurrentIndex;
            nFirstLoopIndex = this->m_nEarliestLoopIndex;
            this->m_gOptimizationBuf.pop();
        }
        this->m_mOptimizationBufMutex.unlock();

        int64 nStartClock = cv::getTickCount();
        //Optimize with smooth.
        // The index of the optimization keyframe.
        // if (nCurrentIndex != -1){
            

        //     map<int,int> dLastSequenceIndices;


        //     this->m_mKeyFrameListMutex.lock();
        //     int nMaxLength = nCurrentIndex+3-nFirstLoopIndex;


        //     vector<Vector6d> gPoses_wi;
        //     vector<Eigen::Matrix3d> gRotations;
        //     vector<vector<int>> gConnections;
        //     gPoses_wi.reserve(nMaxLength);
        //     gRotations.reserve(nMaxLength);
        //     gConnections.reserve(nMaxLength);
            
        //     //Iterate for all keyframes.
        //     for (int k = nFirstLoopIndex; k < this->m_gAllKeyFrames.size(); k++)
        //     {   
        //         ServerKeyFrame * pKeyFrame = this->m_gAllKeyFrames[k];
        //         if (pKeyFrame->m_nGlobalIndex > nCurrentIndex){
        //             break;
        //         }

        //         int nOptimizationIndex = k-nFirstLoopIndex;
        //         //nOptimizationIndex+nFirstLoopIndex = k

        //         //The keyframe after the last sequence should be updated with the drift.
        //         dLastSequenceIndices[pKeyFrame->m_nClientID] = pKeyFrame->m_nGlobalIndex;
        //         //I don't want to set the local index of the keyframe.

        //         Eigen::Quaterniond mTempQ_wi;
        //         Eigen::Matrix3d mTempRotation_wi;
        //         Eigen::Vector3d mTempTranslation_wi;
        //         //Load the pose of the keyframe.
        //         pKeyFrame->GetPose(mTempTranslation_wi, mTempRotation_wi);
        //         mTempQ_wi = mTempRotation_wi;
        //         Vector6d mPoseAll_wi;

        //         //Send the parameters to the array.
        //         mPoseAll_wi(3 , 0) = mTempTranslation_wi(0);
        //         mPoseAll_wi(4 , 0) = mTempTranslation_wi(1);
        //         mPoseAll_wi(5 , 0) = mTempTranslation_wi(2);

        //         Eigen::Vector3d mEulerAngle = ServerUtility::R2ypr(mTempQ_wi.toRotationMatrix());
        //         mPoseAll_wi(0 , 0) = mEulerAngle.x();
        //         mPoseAll_wi(1 , 0) = mEulerAngle.y();
        //         mPoseAll_wi(2 , 0) = mEulerAngle.z();






        //         int nConnectSize = 3;
        //         vector<int> gPreviousIndices;
        //         gPreviousIndices.reserve(nConnectSize*2);
        //         int nRelevantSize = 0;
        //         int nCopyPreviousIndex = nOptimizationIndex-1;
        //         int nPreviousIndex = nCopyPreviousIndex;



        //         for (int n=0;n<nCurrentIndex;n++){
        //             //Enough
        //             if (nRelevantSize >=nConnectSize){
        //                 break;
        //             }

        //             //No more previous keyframes.
        //             //Out of the window.
        //             if (nCopyPreviousIndex-n <0){
        //                 break;
        //             }
        //             nPreviousIndex = nCopyPreviousIndex - n;
        //             ServerKeyFrame * pPreviousKeyFrame = this->m_gAllKeyFrames[nPreviousIndex + nFirstLoopIndex];
        //             //The constraint should be in the same submap.
        //             if (pPreviousKeyFrame->m_nClientID != pKeyFrame->m_nClientID){
        //                 continue;
        //             }
        //             nRelevantSize++;
        //             gPreviousIndices.push_back(nPreviousIndex);
        //         }

        //         //Following
        //         nRelevantSize = 0;
        //         nCopyPreviousIndex = nOptimizationIndex+1;
        //         nPreviousIndex = nCopyPreviousIndex;
        //         int nBoundary = nCurrentIndex-nFirstLoopIndex;
        //         for (int n=0;n<nBoundary;n++){
        //             //Enough
        //             if (nRelevantSize >=nConnectSize){
        //                 break;
        //             }

        //             //No more previous keyframes.
        //             //Out of the window.
        //             if (nCopyPreviousIndex+n >= nBoundary){
        //                 break;
        //             }
        //             nPreviousIndex = nCopyPreviousIndex + n;
        //             ServerKeyFrame * pPreviousKeyFrame = this->m_gAllKeyFrames[nPreviousIndex + nFirstLoopIndex];
        //             //The constraint should be in the same submap.
        //             if (pPreviousKeyFrame->m_nClientID != pKeyFrame->m_nClientID){
        //                 continue;
        //             }
        //             nRelevantSize++;
        //             gPreviousIndices.push_back(nPreviousIndex);
        //         }


        //         gRotations.push_back(mTempRotation_wi);
        //         gPoses_wi.push_back(mPoseAll_wi);
        //         gConnections.push_back(gPreviousIndices);
                
        //     }

        //     int nNodeSize = gPoses_wi.size();
        //     SmoothEulerOptimizer * pSmoothOptimizer = new SmoothEulerOptimizer(nNodeSize);
        //     //Bind poses
        //     for (int iii=0;iii<gPoses_wi.size();iii++){
        //         pSmoothOptimizer->AddKeyFramePose(gPoses_wi[iii]);
        //     }
        //     //Bind connections.
        //     for (int iii=nNodeSize-1;iii>=1;iii--){
        //         vector<int> gSingleConnections = gConnections[iii];
        //         for (int nConnectedIndex : gSingleConnections){
        //         // for (int nShift = -nConnectSize; nShift < nConnectSize;nShift++){
        //             // int nConnectedIndex = i+nShift;
        //             if (nConnectedIndex >= nNodeSize || 
        //                 nConnectedIndex <0 || 
        //                 nConnectedIndex == iii){
        //                 continue;
        //             }
        //             // cout << "nFirstLoopIndex2 is: " << nFirstLoopIndex << endl;
                    

        //             //o is i, p is connect
        //             double nRelativeYaw_po = gPoses_wi[nConnectedIndex](0 , 0) - gPoses_wi[iii](0 , 0);

        //             Eigen::Vector3d mRelativeTranslation_po(
        //                 gPoses_wi[iii](3 , 0) - gPoses_wi[nConnectedIndex](3 , 0),
        //                 gPoses_wi[iii](4 , 0) - gPoses_wi[nConnectedIndex](4 , 0),
        //                 gPoses_wi[iii](5 , 0) - gPoses_wi[nConnectedIndex](5 , 0));

        //             mRelativeTranslation_po = gRotations[nConnectedIndex].inverse() * mRelativeTranslation_po;
                    
        //             //Add Connections.
        //             Eigen::Vector4d mPose_po;
        //             mPose_po(0 , 0) = nRelativeYaw_po;
        //             mPose_po(1 , 0) = mRelativeTranslation_po(0 , 0);
        //             mPose_po(2 , 0) = mRelativeTranslation_po(1 , 0);
        //             mPose_po(3 , 0) = mRelativeTranslation_po(2 , 0);

        //             pSmoothOptimizer->AddConnections(iii, nConnectedIndex, mPose_po);
        //         }
        //     }

        //     //Bind loop connections.
        //     for (int k = nFirstLoopIndex; k < this->m_gAllKeyFrames.size(); k++)
        //     {   
        //         ServerKeyFrame * pKeyFrame = this->m_gAllKeyFrames[k];
        //         if (pKeyFrame->m_nGlobalIndex > nCurrentIndex){
        //             break;
        //         }

        //         int nOptimizationIndex = k-nFirstLoopIndex;
        //         //nOptimizationIndex+nFirstLoopIndex = k
        //         //Loop info.
        //         if (pKeyFrame->m_bHasLoop){
        //             if (pKeyFrame->m_bHasLoop && this->m_dAligned[pKeyFrame->m_nClientID] == 1 &&
        //                 this->m_dAligned[this->GetKeyFrame(pKeyFrame->m_nLoopIndex)->m_nClientID] == 1){

        //                 for (int li=0;li<pKeyFrame->m_gLoopIndices.size();li++){
        //                      //Check the index.
        //                     int nLoopIndex = pKeyFrame->m_gLoopIndices[li];

        //                     assert(nLoopIndex >= nFirstLoopIndex);

        //                     int nConnectedIndex = nLoopIndex - nFirstLoopIndex;

        //                     //Add loop edge.
        //                     Eigen::Vector3d mRelativeT = pKeyFrame->GetLoopRelativeT(li);
        //                     double nRelativeYaw = -pKeyFrame->GetLoopRelativeYaw(li);

        //                     Eigen::Vector4d mRelativePose_po(
        //                         nRelativeYaw,
        //                         mRelativeT(0 , 0),
        //                         mRelativeT(1 , 0),
        //                         mRelativeT(2 , 0));
                            
        //                     pSmoothOptimizer->AddConnections(nOptimizationIndex, nConnectedIndex, mRelativePose_po);

        //                 }
                       
        //             }
        //         }
        //     }
                


        //     //Start to optimize
        //     this->m_mKeyFrameListMutex.unlock();
        //     //Optimize.



        //     pSmoothOptimizer->Optimize();

        //     vector<Vector6d> gOptimizedPoses_wi = pSmoothOptimizer->LoadPoses();
        //     //Firstly update the pose.
        //     this->m_mKeyFrameListMutex.lock();
            
        //     for (int k=0; k <= nCurrentIndex; k++)
        //     {
        //         ServerKeyFrame * pKeyFrame = this->m_gAllKeyFrames[k];
        //         if (pKeyFrame->m_nGlobalIndex < nFirstLoopIndex){
        //             continue;
        //         }

        //         int nOptimizationIndex = k - nFirstLoopIndex;
        //         Vector6d mEulerPose_wi = gOptimizedPoses_wi[nOptimizationIndex];
                
        //         //TODO:Convert euler to sophus!
        //         Sophus::SE3 mSophusPose_wi = ConvertPoseToSophus(mEulerPose_wi);

        //         Eigen::Vector3d mTempTranslation_wi = mSophusPose_wi.translation();
        //         Eigen::Matrix3d mTempRotation_wi = mSophusPose_wi.rotation_matrix();
        //         Eigen::Matrix3d mOldRotation_wi;
        //         Eigen::Vector3d mOldTranslation_wi;

        //         //Update pose.
        //         pKeyFrame->GetPose(mOldTranslation_wi, mOldRotation_wi);
        //         Sophus::SE3 mRelativePose = Sophus::SE3(mOldRotation_wi, mOldTranslation_wi).inverse() * Sophus::SE3(mTempRotation_wi, mTempTranslation_wi);
        //         pKeyFrame->UpdatePose(mTempTranslation_wi, mTempRotation_wi);

        //         this->InsertUpdatedKeyFrame(pKeyFrame);
        //     }


        //     //Calculate the drift and update the latter poses.
        //     map<int, int>::iterator pIter;
        //     for(pIter = dLastSequenceIndices.begin(); pIter != dLastSequenceIndices.end(); pIter++)
        //     {
        //         int nClientID = pIter->first;
        //         int nLastIndex = pIter->second;

        //         ServerKeyFrame * pKeyFrame = this->GetKeyFrame(nLastIndex);

        //         Eigen::Vector3d mCurrentTranslation, mVIOTranslation;
        //         Eigen::Matrix3d mCurrentRotation, mVIORotation;
        //         pKeyFrame->GetPose(mCurrentTranslation, mCurrentRotation);
        //         pKeyFrame->GetVIOPose(mVIOTranslation, mVIORotation);

        //         //Modify the drift array.
        //         this->m_mDriftMutex.lock();
        //         double nYawDrift = ServerUtility::R2ypr(mCurrentRotation).x() - ServerUtility::R2ypr(mVIORotation).x();
        //         this->m_dRotationDrift[nClientID] = ServerUtility::ypr2R(Eigen::Vector3d(nYawDrift, 0 , 0));
        //         this->m_dTranslationDrift[nClientID] = mCurrentTranslation - this->m_dRotationDrift[nClientID] * mVIOTranslation;

        //         this->m_mDriftMutex.unlock();

        //         this->InsertUpdatedKeyFrame(pKeyFrame);
        //     }


        //     // update latter poses
        //     for (int k=nCurrentIndex+1; k < this->m_gAllKeyFrames.size(); k++)
        //     {
        //         ServerKeyFrame* pKeyFrame = this->m_gAllKeyFrames[k];
        //         int nClientID = pKeyFrame->m_nClientID;
        //         Eigen::Vector3d mTempTranslation_wi;
        //         Eigen::Matrix3d mTempRotation_wi;
        //         pKeyFrame->GetVIOPose(mTempTranslation_wi, mTempRotation_wi);
        //         mTempTranslation_wi =  this->m_dRotationDrift[nClientID]* mTempTranslation_wi + this->m_dTranslationDrift[nClientID];
        //         mTempRotation_wi = this->m_dRotationDrift[nClientID] * mTempRotation_wi;
        //         pKeyFrame->UpdatePose(mTempTranslation_wi, mTempRotation_wi);

        //         this->InsertUpdatedKeyFrame(pKeyFrame);
        //     }
        //     //TODO:Update path.
        //     // updatePath();
        //     m_mKeyFrameListMutex.unlock();

        // }


        // printf("Cost Smooth is: %fms\n", (cv::getTickCount() - nStartClock)/ cv::getTickFrequency() * 1000);


        //Optimize with Ceres.
        // //The index of the optimization keyframe.

        if (nCurrentIndex != -1){
            

            this->m_mKeyFrameListMutex.lock();




            int nMaxLength = nCurrentIndex+3-nFirstLoopIndex;



            ceres::Solver::Summary iSummary;

            //Prepare for the optimization.
            //Parameter blocks.
            double gTranslations[nMaxLength][3];
            Eigen::Quaterniond gQRotations[nMaxLength];
            double gEulers[nMaxLength][3];

            //Preparations for the ceres framework.
            ceres::Problem iProblem;
            ceres::Solver::Options iOptions;
            iOptions.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            iOptions.max_num_iterations = 20;

            //Kernel function.
            ceres::LossFunction * pLossFunction;
            pLossFunction = new ceres::HuberLoss(0.1);
            //loss_function = new ceres::CauchyLoss(1.0);

            //A theta, but I don't know what's it is currently.
            ceres::LocalParameterization* pAngleLocalParameterization =
                AngleLocalParameterization::Create();


            map<int,int> dLastSequenceIndices;
            int nOptimizationIndex = 0;
            bool bFixFirst = false;



            //Iterate for all keyframes.
            for (int k = nFirstLoopIndex; k < this->m_gAllKeyFrames.size(); k++)
            {   
                ServerKeyFrame * pKeyFrame = this->m_gAllKeyFrames[k];
                if (pKeyFrame->m_nGlobalIndex > nCurrentIndex){
                    break;
                }

                nOptimizationIndex = k-nFirstLoopIndex;
                //nOptimizationIndex+nFirstLoopIndex = k

                //The keyframe after the last sequence should be updated with the drift.
                dLastSequenceIndices[pKeyFrame->m_nClientID] = pKeyFrame->m_nGlobalIndex;
                //I don't want to set the local index of the keyframe.

                Eigen::Quaterniond mTempQ_wi;
                Eigen::Matrix3d mTempRotation_wi;
                Eigen::Vector3d mTempTranslation_wi;
                //Load the pose of the keyframe.
                pKeyFrame->GetVIOPose(mTempTranslation_wi, mTempRotation_wi);


                
                mTempQ_wi = mTempRotation_wi;

                //Send the parameters to the array.
                gTranslations[nOptimizationIndex][0] = mTempTranslation_wi(0);
                gTranslations[nOptimizationIndex][1] = mTempTranslation_wi(1);
                gTranslations[nOptimizationIndex][2] = mTempTranslation_wi(2);

                gQRotations[nOptimizationIndex] = mTempQ_wi;

                Eigen::Vector3d mEulerAngle = ServerUtility::R2ypr(mTempQ_wi.toRotationMatrix());
                gEulers[nOptimizationIndex][0] = mEulerAngle.x();
                gEulers[nOptimizationIndex][1] = mEulerAngle.y();
                gEulers[nOptimizationIndex][2] = mEulerAngle.z();



                //Just the yaw is optimized.
                iProblem.AddParameterBlock(gEulers[nOptimizationIndex], 1, pAngleLocalParameterization);
                //Translation is always optimized.
                iProblem.AddParameterBlock(gTranslations[nOptimizationIndex], 3);

                //Till now, the pose of this keyframe has been added to the ceres solver.
                //The first frame of the first client must be set fixed.
                if (bFixFirst == false && pKeyFrame->m_nClientID == m_nFirstClient){
                    bFixFirst = true;
                    iProblem.SetParameterBlockConstant(gEulers[nOptimizationIndex]);
                    iProblem.SetParameterBlockConstant(gTranslations[nOptimizationIndex]);
                }

                int nPreviousIndex = nOptimizationIndex-1;
                int nCopyPreviousIndex = nPreviousIndex;
                //Find the previous 5 frames.
                int nRelevantSize = 0;


                for (int n=0;n<nCopyPreviousIndex;n++){
                    //Enough
                    if (nRelevantSize >=6){
                        break;
                    }


                    //No more previous keyframes.
                    //Out of the window.
                    if (nCopyPreviousIndex-n <0){
                        break;
                    }
                    nPreviousIndex = nCopyPreviousIndex - n;
                    ServerKeyFrame * pPreviousKeyFrame = this->m_gAllKeyFrames[nPreviousIndex + nFirstLoopIndex];
                    //The constraint should be in the same submap.
                    if (pPreviousKeyFrame->m_nClientID != pKeyFrame->m_nClientID){
                        continue;
                    }

                    //Add the pose graph constraints.

                    Eigen::Vector3d mEulerConnected = ServerUtility::R2ypr(gQRotations[nPreviousIndex].toRotationMatrix());
                    Eigen::Vector3d mRelativeT;
                    mRelativeT(0) = gTranslations[nOptimizationIndex][0] - gTranslations[nPreviousIndex][0];
                    mRelativeT(1) = gTranslations[nOptimizationIndex][1] - gTranslations[nPreviousIndex][1];
                    mRelativeT(2) = gTranslations[nOptimizationIndex][2] - gTranslations[nPreviousIndex][2];


                    // if (mRelativeT.norm() > 0.5){
                    //     continue;
                    // }

                    mRelativeT = gQRotations[nPreviousIndex].inverse() * mRelativeT;
                    double nRelativeYaw = gEulers[nOptimizationIndex][0] - gEulers[nPreviousIndex][0];


                    nRelevantSize++;

                    ceres::CostFunction* pCostFunction = FourDOFError::Create( 
                                    mRelativeT.x(), 
                                    mRelativeT.y(), 
                                    mRelativeT.z(),
                                    nRelativeYaw, 
                                    mEulerConnected.y(), 
                                    mEulerConnected.z());


                    iProblem.AddResidualBlock(
                        pCostFunction, NULL, 
                        gEulers[nPreviousIndex], 
                        gTranslations[nPreviousIndex], 
                        gEulers[nOptimizationIndex], 
                        gTranslations[nOptimizationIndex]);

                }



                // if (this->GetKeyFrame(pKeyFrame->m_nLoopIndex) == nullptr){
                //     continue;
                // }


                // cout << "Current ID is: " << pKeyFrame->m_nGlobalIndex << endl;
                // cout << "Has loop: " << pKeyFrame->m_bHasLoop << endl;
                // cout << "Loop index is: " << pKeyFrame->m_nLoopIndex << endl;
                // cout << "Current Client ID" << pKeyFrame->m_nClientID << endl;
                // cout << "Loop index" << pKeyFrame->m_nLoopIndex << endl;
                // cout << "this->GetKeyFrame(pKeyFrame->m_nLoopIndex)->m_nClientID" << this->GetKeyFrame(pKeyFrame->m_nLoopIndex)->m_nClientID << endl;
                // cout << "Finish Log" << endl;

                //Add loop edges.
                //TODO: The condition here should also be adjusted.
                //Now only both of the submap has been aligned with the world coordinate system,
                //the optimization will be conducted.

                //Only when aligned with the world the edge will be considered.




                if (pKeyFrame->m_bHasLoop){
                    if (pKeyFrame->m_bHasLoop && this->m_dAligned[pKeyFrame->m_nClientID] == 1 &&
                        this->m_dAligned[this->GetKeyFrame(pKeyFrame->m_nLoopIndex)->m_nClientID] == 1){


                        for (int li=0;li<pKeyFrame->m_gLoopIndices.size();li++){
                             //Check the index.
                            int nLoopIndex = pKeyFrame->m_gLoopIndices[li];

                            assert(nLoopIndex >= nFirstLoopIndex);

                            int nConnectedIndex = nLoopIndex - nFirstLoopIndex;

                           //Add loop edge.
                            Eigen::Vector3d mEulerConnected = ServerUtility::R2ypr(gQRotations[nConnectedIndex].toRotationMatrix());
                            Eigen::Vector3d mRelativeT = pKeyFrame->GetLoopRelativeT(li);
                            double nRelativeYaw = pKeyFrame->GetLoopRelativeYaw(li);
                            ceres::CostFunction* pCostFunction = FourDOFWeightError::Create(
                            mRelativeT.x(), mRelativeT.y(), mRelativeT.z(),
                            nRelativeYaw, mEulerConnected.y(), mEulerConnected.z());



                            iProblem.AddResidualBlock(
                                pCostFunction, 
                                pLossFunction, 
                                gEulers[nConnectedIndex], 
                                gTranslations[nConnectedIndex], 
                                gEulers[nOptimizationIndex], 
                                gTranslations[nOptimizationIndex]);   

                        }
                       
                       
                    }   
                }
                




            }


            this->RecordConnection(nFirstLoopIndex, nCurrentIndex);


            ConnectionGraph * pGraph = this->m_pConnectionGraph;
            vector<MapConnection *> gConnections = pGraph->GetConnections();
            for (auto pConnection : gConnections){
                //Loop 1  Key 2
                ServerKeyFrame * pKeyFrame_1 = pConnection->m_pKeyFrame1;
                ServerKeyFrame * pKeyFrame_2 = pConnection->m_pKeyFrame2;
                Sophus::SE3 mRelativePose_12 = pConnection->m_mTransform_12;
                
                int nLoopIndex = pKeyFrame_1->m_nGlobalIndex;
                int nKeyIndex = pKeyFrame_2->m_nGlobalIndex;

                //Invalid index.
                if (nLoopIndex < nFirstLoopIndex || nLoopIndex > nCurrentIndex){
                    continue;
                }

                if (nKeyIndex < nFirstLoopIndex || nKeyIndex > nCurrentIndex){
                    continue;
                }

                int nConnectedIndex = nLoopIndex - nFirstLoopIndex;
                int nOptimizationIndex = nKeyIndex - nFirstLoopIndex;
                Eigen::Vector3d mRelativeQ = ServerUtility::R2ypr(mRelativePose_12.rotation_matrix());
                Eigen::Vector3d mRelativeT = mRelativePose_12.translation();

                //Add loop edge.
                Eigen::Vector3d mEulerConnected = ServerUtility::R2ypr(gQRotations[nConnectedIndex].toRotationMatrix());
                double nRelativeYaw = mRelativeQ.x();
                ceres::CostFunction* pCostFunction = FourDOFWeightError::Create(
                        mRelativeT.x(), mRelativeT.y(), mRelativeT.z(),
                        nRelativeYaw, mEulerConnected.y(), mEulerConnected.z());



                iProblem.AddResidualBlock(
                    pCostFunction, 
                    pLossFunction, 
                    gEulers[nConnectedIndex], 
                    gTranslations[nConnectedIndex], 
                    gEulers[nOptimizationIndex], 
                    gTranslations[nOptimizationIndex]);   


            }




            //Start to optimize
            this->m_mKeyFrameListMutex.unlock();
            //Optimize.
            ceres::Solve(iOptions, &iProblem, &iSummary);

            //Load the result.


            fOutFile << "Frame number: " << this->m_gAllKeyFrames.size() << endl;
            fOutFile << "Time cost: " << ((cv::getTickCount() - nStartClock)/ cv::getTickFrequency() * 1000) << endl;
            fOutFile << iSummary.BriefReport() << endl;


            //Firstly update the pose.
            this->m_mKeyFrameListMutex.lock();

            
            for (int k=0; k <= nCurrentIndex; k++)
            {
                ServerKeyFrame * pKeyFrame = this->m_gAllKeyFrames[k];
                if (pKeyFrame->m_nGlobalIndex < nFirstLoopIndex){
                    continue;
                }

                int nOptimizationIndex = k - nFirstLoopIndex;
                
                Eigen::Quaterniond mTempQ_wi;
                mTempQ_wi = ServerUtility::ypr2R(Eigen::Vector3d(
                    gEulers[nOptimizationIndex][0], 
                    gEulers[nOptimizationIndex][1], 
                    gEulers[nOptimizationIndex][2]));

                Eigen::Vector3d mTempTranslation_wi = Eigen::Vector3d(
                    gTranslations[nOptimizationIndex][0],
                    gTranslations[nOptimizationIndex][1],
                    gTranslations[nOptimizationIndex][2]);

                Eigen::Matrix3d mTempRotation_wi = mTempQ_wi.toRotationMatrix();


                Eigen::Matrix3d mOldRotation_wi;
                Eigen::Vector3d mOldTranslation_wi;
                pKeyFrame->GetPose(mOldTranslation_wi, mOldRotation_wi);
                Sophus::SE3 mRelativePose = Sophus::SE3(mOldRotation_wi, mOldTranslation_wi).inverse() * Sophus::SE3(mTempRotation_wi, mTempTranslation_wi);


                pKeyFrame->UpdatePose(mTempTranslation_wi, mTempRotation_wi);

                this->InsertUpdatedKeyFrame(pKeyFrame);
            }



            //Calculate the drift and update the latter poses.

            map<int, int>::iterator pIter;
            for(pIter = dLastSequenceIndices.begin(); pIter != dLastSequenceIndices.end(); pIter++)
            {
                int nClientID = pIter->first;
                int nLastIndex = pIter->second;

                ServerKeyFrame * pKeyFrame = this->GetKeyFrame(nLastIndex);

                Eigen::Vector3d mCurrentTranslation, mVIOTranslation;
                Eigen::Matrix3d mCurrentRotation, mVIORotation;
                pKeyFrame->GetPose(mCurrentTranslation, mCurrentRotation);
                pKeyFrame->GetVIOPose(mVIOTranslation, mVIORotation);

                //Modify the drift array.
                this->m_mDriftMutex.lock();
                double nYawDrift = ServerUtility::R2ypr(mCurrentRotation).x() - ServerUtility::R2ypr(mVIORotation).x();
                this->m_dRotationDrift[nClientID] = ServerUtility::ypr2R(Eigen::Vector3d(nYawDrift, 0 , 0));
                this->m_dTranslationDrift[nClientID] = mCurrentTranslation - this->m_dRotationDrift[nClientID] * mVIOTranslation;

                this->m_mDriftMutex.unlock();

                this->InsertUpdatedKeyFrame(pKeyFrame);
            }


            // update latter poses
            for (int k=nCurrentIndex+1; k < this->m_gAllKeyFrames.size(); k++)
            {
                ServerKeyFrame* pKeyFrame = this->m_gAllKeyFrames[k];
                int nClientID = pKeyFrame->m_nClientID;
                Eigen::Vector3d mTempTranslation_wi;
                Eigen::Matrix3d mTempRotation_wi;
                pKeyFrame->GetVIOPose(mTempTranslation_wi, mTempRotation_wi);
                mTempTranslation_wi =  this->m_dRotationDrift[nClientID]* mTempTranslation_wi + this->m_dTranslationDrift[nClientID];
                mTempRotation_wi = this->m_dRotationDrift[nClientID] * mTempRotation_wi;
                pKeyFrame->UpdatePose(mTempTranslation_wi, mTempRotation_wi);

                this->InsertUpdatedKeyFrame(pKeyFrame);
            }
            //TODO:Update path.
            // updatePath();


            m_mKeyFrameListMutex.unlock();

            // for (int ii=0;ii<30;ii++){
            //     cout << "Complete Optimization" << endl;
            // }
        }

        printf("Cost Ceres is: %fms\n", (cv::getTickCount() - nStartClock)/ cv::getTickFrequency() * 1000);

        //Have a rest.
        std::chrono::milliseconds nDura(5000);
        std::this_thread::sleep_for(nDura);
    }

}



void ServerPoseGraph::LoadVocabulary(string aVocPath){
    this->m_pVocabulary = new BriefVocabulary(aVocPath);
    this->m_iBriefDB.setVocabulary(*this->m_pVocabulary, false, 0);
}


void ServerPoseGraph::SetPlotter(ServerPlotter * pServerPlotter){
    this->m_pPlotter = pServerPlotter;
}


ServerPlotter * ServerPoseGraph::GetPlotter(){
    return this->m_pPlotter;
}





// void ServerPoseGraph::UpdatePath()
// {
//     map<int, bool>::iterator pIter;
//     for (pIter = this->m_dAligned.begin(); 
//          pIter != this->m_dAligned.end(); 
//          pIter++){
//         this->m_gPaths[pIter->first].poses.clear();
//     }
    
       
//     base_path.poses.clear();
//     posegraph_visualization->reset();

    
//     if (SAVE_LOOP_PATH)
//     {
//         std::string pose_graph_path = VINS_RESULT_PATH + "/pose_graph_path.csv";
//         ofstream loop_path_file_tmp(pose_graph_path, ios::out);
//         loop_path_file_tmp.close();
//         map<int, bool>::iterator iter;
//         for (iter = sequence_align_world.begin(); iter != sequence_align_world.end(); iter++)
//         {
//             std::string pose_graph_sequence_path = VINS_RESULT_PATH + "/pose_graph_path_" + to_string(iter->first) + ".csv";
//             ofstream loop_path_file_tmp(pose_graph_sequence_path, ios::out);
//             loop_path_file_tmp.close();
//         }
//     }
    

//     for (size_t k = 0; k < keyframe_vec.size(); k++)
//     {
//         KeyFrame* it = keyframe_vec[k];
//         Vector3d P;
//         Matrix3d R;
//         (it)->getPose(P, R);
//         Quaterniond Q;
//         Q = R;
// //        printf("path p: %f, %f, %f\n",  P.x(),  P.z(),  P.y() );

//         geometry_msgs::PoseStamped pose_stamped;
//         pose_stamped.header.stamp = ros::Time((it)->time_stamp);
//         pose_stamped.header.frame_id = "world";
//         pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
//         pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
//         pose_stamped.pose.position.z = P.z();
//         pose_stamped.pose.orientation.x = Q.x();
//         pose_stamped.pose.orientation.y = Q.y();
//         pose_stamped.pose.orientation.z = Q.z();
//         pose_stamped.pose.orientation.w = Q.w();
//         if((it)->sequence == 0)
//         {
//             base_path.poses.push_back(pose_stamped);
//             base_path.header = pose_stamped.header;
//         }
//         path[(it)->sequence].poses.push_back(pose_stamped);
//         path[(it)->sequence].header = pose_stamped.header;

        
//         if (SAVE_LOOP_PATH)
//         {
//             std::string pose_graph_path = VINS_RESULT_PATH + "/pose_graph_path.csv";
//             ofstream loop_path_file(pose_graph_path, ios::app);
//             loop_path_file.setf(ios::fixed, ios::floatfield);
//             loop_path_file.precision(0);
//             loop_path_file << (it)->time_stamp * 1e9 << ",";
//             loop_path_file.precision(5);
//             loop_path_file  << P.x() << ","
//                   << P.y() << ","
//                   << P.z() << ","
//                   << Q.w() << ","
//                   << Q.x() << ","
//                   << Q.y() << ","
//                   << Q.z() << ","
//                   << endl;
//             loop_path_file.close();

//             std::string pose_graph_sequence_path = VINS_RESULT_PATH + "/pose_graph_path_" + to_string((it)->sequence) + ".csv";
//             ofstream sequence_path_file(pose_graph_sequence_path, ios::app);
//             sequence_path_file.setf(ios::fixed, ios::floatfield);
//             sequence_path_file.precision(0);
//             sequence_path_file << (it)->time_stamp * 1e9 << ",";
//             sequence_path_file.precision(5);
//             sequence_path_file  << P.x() << ","
//                   << P.y() << ","
//                   << P.z() << ","
//                   << Q.w() << ","
//                   << Q.x() << ","
//                   << Q.y() << ","
//                   << Q.z() << ","
//                   << endl;
//             sequence_path_file.close();
//         }
        
//         //draw local connection
//         /*
//         if (SHOW_S_EDGE)
//         {
//             list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
//             list<KeyFrame*>::reverse_iterator lrit;
//             for (; rit != keyframelist.rend(); rit++)  
//             {  
//                 if ((*rit)->index == (*it)->index)
//                 {
//                     lrit = rit;
//                     lrit++;
//                     for (int i = 0; i < 4; i++)
//                     {
//                         if (lrit == keyframelist.rend())
//                             break;
//                         if((*lrit)->sequence == (*it)->sequence)
//                         {
//                             Vector3d conncected_P;
//                             Matrix3d connected_R;
//                             (*lrit)->getPose(conncected_P, connected_R);
//                             posegraph_visualization->add_edge(P, conncected_P);
//                         }
//                         lrit++;
//                     }
//                     break;
//                 }
//             } 
//         }
//         */
//         if (SHOW_L_EDGE)
//         {
//             if ((it)->has_loop && (it)->sequence != 0)
//             {
                
//                 KeyFrame* connected_KF = getKeyFrame((it)->loop_index);
//                 Vector3d connected_P;
//                 Matrix3d connected_R;
//                 connected_KF->getPose(connected_P, connected_R);
//                 //(*it)->getVioPose(P, R);
//                 (it)->getPose(P, R);
//                 if (it->sequence == connected_KF->sequence)
//                     posegraph_visualization->add_loopedge(P, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0), 0);
//                 else
//                     posegraph_visualization->add_loopedge(P, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0), 1);
//             }
//         }

//     }
//     publish();
// }



