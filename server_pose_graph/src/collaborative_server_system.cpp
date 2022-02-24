#include "../include/collaborative_server_system.h"
#include <fstream>

using namespace std;

CollaborativeServer::CollaborativeServer(string aVocFile, string aConfigFile, ros::NodeHandle iNodeHandler){
	this->m_aConfigFile = aConfigFile;

	//Construct the pose graph.
	this->m_pPoseGraph = new ServerPoseGraph();
	this->m_pPoseGraph->LoadVocabulary(aVocFile);

	//Initialize the plotter.
	this->m_pPlotter = new ServerPlotter();
	this->m_pPoseGraph->SetPlotter(this->m_pPlotter);


	this->m_iAgentSubscriber = iNodeHandler.subscribe("/agent_frame", 2000, &CollaborativeServer::AgentCallback, this);
	this->m_iImageSubscriber = iNodeHandler.subscribe("/pose_graph/image", 1000, &CollaborativeServer::ImageCallback, this);

    this->m_iDepthMapPublisher = iNodeHandler.advertise<sensor_msgs::Image>("depth/image_raw",1000);
    this->m_iColorImagePublisher = iNodeHandler.advertise<sensor_msgs::Image>("rgb/image_raw",1000);
    this->m_iDepthCameraInfoPublisher = iNodeHandler.advertise<sensor_msgs::CameraInfo>("depth/image_info",1000);
    this->m_iColorCameraInfoPublisher = iNodeHandler.advertise<sensor_msgs::CameraInfo>("rgb/image_info",1000);
    this->m_iPointCloudPublisher = iNodeHandler.advertise<sensor_msgs::PointCloud2>("point_cloud2",1000);


    //Save the mesh

    this->m_iSaveMeshClient = iNodeHandler.serviceClient<chisel_ros::SaveMeshService>("/Chisel/SaveMesh");
  
 

}


bool CollaborativeServer::SaveMesh(){
    
    // 创建learning_communication::AddTwoInts类型的service消息
    chisel_ros::SaveMeshService srv;
    srv.request.file_name = "/home/kyrie/Documents/DataSet/Mesh/mesh.ply";
      
    if (this->m_iSaveMeshClient.call(srv))
    {
        ROS_INFO("Save mesh");
        return true;
    }  
    ROS_WARN("Failed to save mesh");
    return false;
}



CollaborativeServer::CollaborativeServer(string aVocFile, vector<string> gAllConfigFiles, ros::NodeHandle iNodeHandler){
    
    this->m_aConfigFile = gAllConfigFiles[0];
    for (auto item : gAllConfigFiles){
        this->m_gAllConfigFiles.push_back(item);
    }

    //Construct the pose graph.
    this->m_pPoseGraph = new ServerPoseGraph();
    this->m_pPoseGraph->LoadVocabulary(aVocFile);

    //Initialize the plotter.
    this->m_pPlotter = new ServerPlotter();
    this->m_pPoseGraph->SetPlotter(this->m_pPlotter);


    this->m_iAgentSubscriber = iNodeHandler.subscribe("/agent_frame", 2000, &CollaborativeServer::AgentCallback, this);
    this->m_iImageSubscriber = iNodeHandler.subscribe("/pose_graph/image", 1000, &CollaborativeServer::ImageCallback, this);

    this->m_iDepthMapPublisher = iNodeHandler.advertise<sensor_msgs::Image>("depth/image_raw",1000);
    this->m_iColorImagePublisher = iNodeHandler.advertise<sensor_msgs::Image>("rgb/image_raw",1000);
    this->m_iDepthCameraInfoPublisher = iNodeHandler.advertise<sensor_msgs::CameraInfo>("depth/image_info",1000);
    this->m_iColorCameraInfoPublisher = iNodeHandler.advertise<sensor_msgs::CameraInfo>("rgb/image_info",1000);
    this->m_iPointCloudPublisher = iNodeHandler.advertise<sensor_msgs::PointCloud2>("point_cloud2",1000);

    //Save the mesh

    this->m_iSaveMeshClient = iNodeHandler.serviceClient<chisel_ros::SaveMeshService>("/Chisel/SaveMesh");
  
 
}    



void CollaborativeServer::ImageCallback(const sensor_msgs::ImageConstPtr &pImageMsg)
{
    this->m_mImageBufMutex.lock();
    this->m_gImageBuf.push(pImageMsg);
    this->m_mImageBufMutex.unlock();
}



void CollaborativeServer::AgentCallback(const agent_msg::AgentMsgConstPtr &pAgentMsg)
{
    if(this->m_bStartFlag)
    {
        this->m_mAgentBufMutex.lock();
        this->m_gAgentMsgBuf.push(pAgentMsg);
        this->m_mAgentBufMutex.unlock();
    }
}




void CollaborativeServer::InitializeCamera(agent_msg::AgentMsgConstPtr pAgentMsg){
    if (pAgentMsg == NULL){
        return;
    }

    int nClientID = pAgentMsg->seq;

    if (this->m_dCameras.find(nClientID) == this->m_dCameras.end()){
        //Create a new camera;
        ServerCamera * pCamera = new ServerCamera();

        string aConfigFile = "";
        if (this->m_gAllConfigFiles.size() == 0){
            aConfigFile = this->m_aConfigFile;
        }else{
            aConfigFile = this->m_gAllConfigFiles[nClientID-1];
        }

        cv::FileStorage fsSettings(aConfigFile, cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            std::cerr << "ERROR: Wrong path to settings" << std::endl;
            return;
        }

       
        if (!fsSettings["model_type"].isNone())
        {
            std::string aModelType;
            fsSettings["model_type"] >> aModelType;

            if (aModelType.compare("PINHOLE") != 0)
            {
                std::cerr << "Only Pinhole is supported now" << endl;
                return;
            }
        }

        Eigen::Matrix3d mK;
        Eigen::Matrix<double , 4, 1> mD;

        cv::FileNode iFileNode = fsSettings["distortion_parameters"];

        double nFx, nFy, nCx, nCy;
        double nK1, nK2, nP1, nP2;

        nK1 = static_cast<double>(iFileNode["k1"]);
        nK2 = static_cast<double>(iFileNode["k2"]);
        nP1 = static_cast<double>(iFileNode["p1"]);
        nP2 = static_cast<double>(iFileNode["p2"]);

        iFileNode = fsSettings["projection_parameters"];
        nFx = static_cast<double>(iFileNode["fx"]);
        nFy = static_cast<double>(iFileNode["fy"]);
        nCx = static_cast<double>(iFileNode["cx"]);
        nCy = static_cast<double>(iFileNode["cy"]);


        mK *=0;


        mK(0 , 0) = nFx;
        mK(1 , 1) = nFy;
        mK(0 , 2) = nCx;
        mK(1 , 2) = nCy;
        mK(2 , 2) = 1.0;

        mD(0 , 0) = nK1;
        mD(1 , 0) = nK2;
        mD(2 , 0) = nP1;
        mD(3 , 0) = nP2;

        pCamera->SetK(mK);
        pCamera->SetD(mD);

        this->m_dCameras[nClientID] = pCamera;

    }

}


void CollaborativeServer::PublishDenseInfo(  Sophus::SE3 mRefPose_wc,
                        std_msgs::Header iHeader,
                        cv::Mat & mDepthMap,
                        cv::Mat & mColorMap,
                        cv::Mat & mK,
                        int nWidth, int nHeight){

    int nPublishWidth = 640;
    int nPublishHeight = 480;

    if (mDepthMap.rows != nHeight || mDepthMap.cols != nWidth || 
        mColorMap.rows != nHeight || mColorMap.cols != nWidth){
        return ;
    }

    if (mDepthMap.rows ==0 || mDepthMap.cols ==0 || 
        mColorMap.rows ==0 || mColorMap.cols ==0){
        return ;
    }


    cv::resize(mDepthMap, mDepthMap, cv::Size(nPublishWidth, nPublishHeight));
    cv::resize(mColorMap, mColorMap, cv::Size(nPublishWidth, nPublishHeight));

    mK.at<double>(0 , 0) = mK.at<double>(0 , 0)/(double)nWidth * (double)nPublishWidth;
    mK.at<double>(0 , 2) = mK.at<double>(0 , 2)/(double)nWidth * (double)nPublishWidth;
    mK.at<double>(1 , 1) = mK.at<double>(1 , 1)/(double)nHeight * (double)nPublishHeight;
    mK.at<double>(1 , 2) = mK.at<double>(1 , 2)/(double)nHeight * (double)nPublishHeight;
    nWidth = nPublishWidth;
    nHeight = nPublishHeight;


    iHeader.stamp = ros::Time(0);
    
    //Send the ref pose
    // camera frame
    Eigen::Vector3d mTranslation = mRefPose_wc.translation();
    Eigen::Matrix3d mRotation = mRefPose_wc.rotation_matrix();

    static tf::TransformBroadcaster iBroadCaster;
    tf::Transform iTransform;
    tf::Quaternion iQ;
    iTransform.setOrigin(tf::Vector3(   mTranslation.x(),
                                        mTranslation.y(),
                                        mTranslation.z()));
    iQ.setW(Eigen::Quaterniond(mRotation).w());
    iQ.setX(Eigen::Quaterniond(mRotation).x());
    iQ.setY(Eigen::Quaterniond(mRotation).y());
    iQ.setZ(Eigen::Quaterniond(mRotation).z());
    iTransform.setRotation(iQ);
    iBroadCaster.sendTransform(
        tf::StampedTransform(
            iTransform, iHeader.stamp, "base", "ref_frame"));
    
    //TODO::Send the point cloud.
    this->SendPointCloud(iHeader, mDepthMap, mColorMap);

    //Send the image and the depth image.
    //Depth map
    {
        int nEdge = 0;

        cv::Mat mDepthMapFloat;
        mDepthMap.convertTo(mDepthMapFloat, CV_32FC1);

        float nBadPoint = std::numeric_limits<float>::quiet_NaN();
        int i = 0;
        for (int32_t u = nEdge; u < mDepthMapFloat.rows - nEdge; ++u)
        {
            for (int32_t v = nEdge; v < mDepthMapFloat.cols - nEdge; ++v, ++i)
            {
                if (mDepthMapFloat.at<float>(u , v) < 0.1f || 
                    mDepthMapFloat.at<float>(u , v) > 20.0f){
                    mDepthMapFloat.at<float>(u , v) = nBadPoint;
                }
            }
        }

        cv_bridge::CvImage out_msg;
        out_msg.header = iHeader;
        out_msg.header.frame_id = "camera";
        out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        out_msg.image = mDepthMapFloat;
        this->m_iDepthMapPublisher.publish(out_msg.toImageMsg());
    }
    //Color map
    {
        cv_bridge::CvImage out_msg;
        out_msg.header = iHeader;
        out_msg.header.frame_id = "camera";

        //Copy the image.
        if (mColorMap.type() == CV_8UC1)    
        {

            out_msg.encoding = sensor_msgs::image_encodings::MONO8;
        } 
        else if (mColorMap.type() == CV_8UC3)
        {
            out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        }

        out_msg.image = mColorMap.clone();
        this->m_iColorImagePublisher.publish(out_msg.toImageMsg());
    }

    {
        sensor_msgs::CameraInfo camera_info;
        camera_info.header = iHeader;

        camera_info.P[0] = mK.at<double>(0, 0);
        camera_info.P[5] = mK.at<double>(1, 1);
        camera_info.P[2] = mK.at<double>(0, 2);
        camera_info.P[6] = mK.at<double>(1, 2);

        camera_info.width = nWidth;
        camera_info.height = nHeight;
        this->m_iDepthCameraInfoPublisher.publish(camera_info);
        this->m_iColorCameraInfoPublisher.publish(camera_info);
        
    }

    mDepthMap.release();
    mColorMap.release();

}




void CollaborativeServer::SendPointCloud(const std_msgs::Header & iHeader,
                        cv::Mat & mDepthMap,
                        cv::Mat & mColorImage){
        sensor_msgs::PointCloud2Ptr points(new sensor_msgs::PointCloud2);
        points->header = iHeader;
        points->header.frame_id = "ref_frame";

        points->height = mDepthMap.rows;
        points->width = mDepthMap.cols;
        points->fields.resize(4);
        points->fields[0].name = "x";
        points->fields[0].offset = 0;
        points->fields[0].count = 1;
        points->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        points->fields[1].name = "y";
        points->fields[1].offset = 4;
        points->fields[1].count = 1;
        points->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        points->fields[2].name = "z";
        points->fields[2].offset = 8;
        points->fields[2].count = 1;
        points->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        points->fields[3].name = "rgb";
        points->fields[3].offset = 12;
        points->fields[3].count = 1;
        points->fields[3].datatype = sensor_msgs::PointField::FLOAT32;
        //points.is_bigendian = false; ???
        points->point_step = 16;
        points->row_step = points->point_step * points->width;
        points->data.resize(points->row_step * points->height);
        points->is_dense = false; // there may be invalid points

        float bad_point = std::numeric_limits<float>::quiet_NaN();
        int i = 0;
        for (int32_t u = 0; u < mDepthMap.rows; ++u)
        {
            for (int32_t v = 0; v < mDepthMap.cols; ++v, ++i)
            {
                float dep = mDepthMap.at<double>(u, v);
                float x = v;
                float y = u;
                if (dep < 10.0f && dep > 0.1f)
                {
                    uint8_t g = mColorImage.at<uint8_t>(u, v);
                    int32_t rgb = (g << 16) | (g << 8) | g;
                    memcpy(&points->data[i * points->point_step + 0], &x, sizeof(float));
                    memcpy(&points->data[i * points->point_step + 4], &y, sizeof(float));
                    memcpy(&points->data[i * points->point_step + 8], &dep, sizeof(float));
                    memcpy(&points->data[i * points->point_step + 12], &rgb, sizeof(int32_t));
                }
                else
                {
                    memcpy(&points->data[i * points->point_step + 0], &bad_point, sizeof(float));
                    memcpy(&points->data[i * points->point_step + 4], &bad_point, sizeof(float));
                    memcpy(&points->data[i * points->point_step + 8], &bad_point, sizeof(float));
                    memcpy(&points->data[i * points->point_step + 12], &bad_point, sizeof(float));
                }
            }
        }
        this->m_iPointCloudPublisher.publish(points);
    }



void CollaborativeServer::PublishProcess(){
     //TODO::Publish the dense info.
    int nMaxIndex = 100;
    bool bStart = false;

    int nIndex = 0;
    while (1){
        Sophus::SE3 mRefPose_wc;
        std_msgs::Header iHeader;
        cv::Mat mDepthMap;
        cv::Mat mColorMap;
        cv::Mat mK;
        int nWidth, nHeight;
                // Publish the dense info.
        ServerKeyFrame * pDenseMappingFrame = this->m_pPoseGraph->PopDepthFrames();
        while (pDenseMappingFrame != NULL){
            if (pDenseMappingFrame->m_bFinalizeDepthMap){
                pDenseMappingFrame->LoadRefInfo(
                            mRefPose_wc, 
                            iHeader, 
                            mDepthMap, 
                            mColorMap, 
                            mK, 
                            nWidth, 
                            nHeight);
                this->PublishDenseInfo(   
                    mRefPose_wc,
                    iHeader,
                    mDepthMap,
                    mColorMap,
                    mK,
                    nWidth, 
                    nHeight);
                bStart = true;
                nIndex = 0;
                pDenseMappingFrame->FreeSpace();
                for (ServerKeyFrame * pKeyFrame : this->m_pPoseGraph->m_gAllKeyFrames){
                    if (pKeyFrame->m_nGlobalIndex < pDenseMappingFrame->m_nGlobalIndex-5 && !pKeyFrame->m_bFreeSpace){
                        pKeyFrame->FreeSpace();
                    }
                }

                // //Have a rest.
                // std::chrono::milliseconds nDura(10);
                // std::this_thread::sleep_for(nDura);


            }
            pDenseMappingFrame = this->m_pPoseGraph->PopDepthFrames();

        }

        nIndex++;
        if (nIndex > nMaxIndex){
            if (bStart  && this->m_pPlotter->GetSaveMesh()){
                this->SaveMesh();
                this->m_pPlotter->SetSaveMesh(false);
            }
            nIndex = 0;
        }

        std::chrono::milliseconds nDura(30);
        std::this_thread::sleep_for(nDura);

    }
}

void CollaborativeServer::AgentProcess()
{
    list<sensor_msgs::ImageConstPtr> gTempImages;

    while (true)
    {
        std::chrono::milliseconds nDura(5);
        std::this_thread::sleep_for(nDura);

        //Synchronize the agent message and image message.
        agent_msg::AgentMsgConstPtr pAgentMsg = NULL;
        sensor_msgs::ImageConstPtr pImageMsg = NULL;


        this->m_mAgentBufMutex.lock();
        if (!this->m_gAgentMsgBuf.empty())
        {
            pAgentMsg = this->m_gAgentMsgBuf.front();
        }
        this->m_mAgentBufMutex.unlock();

        if (pAgentMsg == NULL){
            continue;
        }

        list<sensor_msgs::ImageConstPtr>::iterator pIter = gTempImages.begin();
        for (;pIter!=gTempImages.end(); pIter++){
            if ((*pIter)->header.stamp.toSec() == pAgentMsg->header.stamp.toSec()){
                pImageMsg = *pIter;
                gTempImages.erase(pIter);
                break;
            }
        }



        this->m_mImageBufMutex.lock();
        while (!this->m_gImageBuf.empty() && pImageMsg == NULL){
            pImageMsg = this->m_gImageBuf.front();
            if (pImageMsg->header.stamp.toSec() == pAgentMsg->header.stamp.toSec()){
                this->m_gImageBuf.pop();
                break;
            }else{
                gTempImages.push_back(pImageMsg);
                this->m_gImageBuf.pop();
                pImageMsg = NULL;
            }
        }
        this->m_mImageBufMutex.unlock();

        if (pImageMsg != NULL){
            this->m_mAgentBufMutex.lock();
            this->m_gAgentMsgBuf.pop();   
            this->m_mAgentBufMutex.unlock(); 
        }



        if(pAgentMsg != NULL && pImageMsg != NULL)
        {
            //Convert the pointer to image.
            cv_bridge::CvImageConstPtr pPointer;
            if (pImageMsg->encoding == "8UC1")
            {
                sensor_msgs::Image iImage;
                iImage.header = pImageMsg->header;
                iImage.height = pImageMsg->height;
                iImage.width = pImageMsg->width;
                iImage.is_bigendian = pImageMsg->is_bigendian;
                iImage.step = pImageMsg->step;
                iImage.data = pImageMsg->data;
                iImage.encoding = "mono8";
                pPointer = cv_bridge::toCvCopy(iImage, sensor_msgs::image_encodings::MONO8);
            }else if (pImageMsg->encoding == "rgb8"){
                sensor_msgs::Image iImage;
                iImage.header = pImageMsg->header;
                iImage.height = pImageMsg->height;
                iImage.width = pImageMsg->width;
                iImage.is_bigendian = pImageMsg->is_bigendian;
                iImage.step = pImageMsg->step;
                iImage.data = pImageMsg->data;
                iImage.encoding = "rgb8";
                pPointer = cv_bridge::toCvCopy(iImage, sensor_msgs::image_encodings::BGR8);
            }else{
                pPointer = cv_bridge::toCvCopy(pImageMsg, sensor_msgs::image_encodings::MONO8);
            }
            
            cv::Mat mImage = pPointer->image;



            this->InitializeCamera(pAgentMsg);


            double nTimeStamp = pAgentMsg->header.stamp.toSec();
           
            //sequence is the nClientID
            int nClientID = pAgentMsg->seq;
            //T
            Eigen::Vector3d mTranslation_wi = Eigen::Vector3d(
                                pAgentMsg->position_imu.x,
                                pAgentMsg->position_imu.y,
                                pAgentMsg->position_imu.z);
            //R
            Eigen::Matrix3d mRotation_wi = Eigen::Quaterniond(
                                pAgentMsg->orientation_imu.w,
                                pAgentMsg->orientation_imu.x,
                                pAgentMsg->orientation_imu.y,
                                pAgentMsg->orientation_imu.z).toRotationMatrix();

            Sophus::SE3 mVIOPose = Sophus::SE3(mRotation_wi, mTranslation_wi).inverse();

            //tic extrinsics
            Eigen::Vector3d mTranslation_ic = Eigen::Vector3d(
                                pAgentMsg->tic.x,
                                pAgentMsg->tic.y,
                                pAgentMsg->tic.z);

            //ric extrinsics
            Eigen::Matrix3d mRotation_ic = Eigen::Quaterniond(
                                pAgentMsg->ric.w,
                                pAgentMsg->ric.x,
                                pAgentMsg->ric.y,
                                pAgentMsg->ric.z).toRotationMatrix();

            vector<cv::Point3f> gWindowPoints3D;    //vector<cv::Point3f> point_3d;
            vector<cv::Point2f> gWindowPoints2D;
            vector<int> gWindowPointsID;


            vector<cv::Point2f> gProjectedPoints2D; //vector<cv::Point2f> feature_2d;
            //vector<BRIEF::bitset> feature_descriptors, point_descriptors;
            vector<DVision::BRIEF::bitset> gWindowDescriptors, gProjectedDescriptors;

            //Load 3D positions of mappoints.
            for (unsigned int i=0;i<pAgentMsg->point_3d.size();i++){
                cv::Point3f iPoint3D;
                iPoint3D.x = pAgentMsg->point_3d[i].x;
                iPoint3D.y = pAgentMsg->point_3d[i].y;
                iPoint3D.z = pAgentMsg->point_3d[i].z;
                gWindowPoints3D.push_back(iPoint3D);
            }  

            //Load 2D positions of mappoints.
            for (unsigned int i=0;i<pAgentMsg->point_uv.size();i++){
                cv::Point2f iPoint2D;
                iPoint2D.x = pAgentMsg->point_uv[i].x;
                iPoint2D.y = pAgentMsg->point_uv[i].y;
                gWindowPoints2D.push_back(iPoint2D);
            }  

            //Load the ID of mappoints.
            for (unsigned int i=0;i<pAgentMsg->point_id.size();i++){
                gWindowPointsID.push_back(pAgentMsg->point_id[i]);
            }

            //Load 2D positions of features.
            for (unsigned int i=0;i<pAgentMsg->feature_2d.size();i++){
                cv::Point2f iPoint2D;
                iPoint2D.x = pAgentMsg->feature_2d[i].x;
                iPoint2D.y = pAgentMsg->feature_2d[i].y;
                gProjectedPoints2D.push_back(iPoint2D);
            }

            //Load descriptors of mappoints.            
            for (unsigned int i = 0; i < pAgentMsg->point_des.size(); i += 4)
            {
                boost::dynamic_bitset<> tmp_brief(256);
                for (int k = 0; k < 4; k++)
                {
                    unsigned long long int tmp_int = pAgentMsg->point_des[i + k];
                    for (int j = 0; j < 64; ++j, tmp_int >>= 1)
                    {
                        tmp_brief[256 - 64 * (k + 1) + j] = (tmp_int & 1);
                    }
                } 
                gWindowDescriptors.push_back(tmp_brief);
            } 


            //Load descriptors of 2D features.
            for (unsigned int i = 0; i < pAgentMsg->feature_des.size(); i = i + 4)
            {
                boost::dynamic_bitset<> tmp_brief(256);
                for (int k = 0; k < 4; k++)
                {
                    unsigned long long int tmp_int = pAgentMsg->feature_des[i + k];
                    for (int j = 0; j < 64; ++j, tmp_int >>= 1)
                    {
                        tmp_brief[256 - 64 * (k + 1) + j] = (tmp_int & 1);
                    }
                } 
                gProjectedDescriptors.push_back(tmp_brief);
            } 


            bool bFromVisionSLAM = true;
            ServerCamera * pServerCamera = this->m_dCameras[nClientID];



            // cout << "Begin to create keyframe" << endl;
            ServerKeyFrame * pKeyFrame = new ServerKeyFrame(nTimeStamp, nClientID, 0, 
                    mVIOPose, 
                    mRotation_ic,
                    mTranslation_ic, 
                    gWindowPoints3D, 
                    gWindowPoints2D, 
                    gWindowPointsID,
                    gWindowDescriptors,
                    gProjectedPoints2D, 
                    gProjectedDescriptors,
                    bFromVisionSLAM,
                    pServerCamera);
                

            //Copy the image.
            if (mImage.type() == CV_8UC1)    
            {

                pKeyFrame->m_mImage = mImage.clone();
                pKeyFrame->m_mColoredImage = mImage.clone();
            } 
            else if (mImage.type() == CV_8UC3)
            {
                pKeyFrame->m_mColoredImage = mImage.clone();
                cv::cvtColor(pKeyFrame->m_mColoredImage, pKeyFrame->m_mImage, CV_BGR2GRAY);

            }
            


            // pKeyFrame->m_mImage = mImage;
            pKeyFrame->m_iHeader = pAgentMsg->header;

            pKeyFrame->UndistortImage();



            // cout << "Finish create keyframe" << endl;


            this->m_mProcessMutex.lock();
            this->m_pPoseGraph->AddKeyFrame(pKeyFrame);
            this->m_nFrameCount++;
            this->m_mProcessMutex.unlock();


            // //TODO::Publish the dense info.
            // Sophus::SE3 mRefPose_wc;
            // std_msgs::Header iHeader;
            // cv::Mat mDepthMap;
            // cv::Mat mColorMap;
            // cv::Mat mK;
            // int nWidth, nHeight;
            // // Publish the dense info.
            // int nCount =0;
            // ServerKeyFrame * pDenseMappingFrame = this->m_pPoseGraph->PopDepthFrames();
            // while (pDenseMappingFrame != NULL && nCount < 3){
            //     if (pDenseMappingFrame->m_bFinalizeDepthMap){
            //         pDenseMappingFrame->LoadRefInfo(
            //             mRefPose_wc, 
            //             iHeader, 
            //             mDepthMap, 
            //             mColorMap, 
            //             mK, 
            //             nWidth, 
            //             nHeight);
            //         this->PublishDenseInfo(   
            //             mRefPose_wc,
            //             iHeader,
            //             mDepthMap,
            //             mColorMap,
            //             mK,
            //             nWidth, 
            //             nHeight);
            //         pDenseMappingFrame->FreeSpace();

            //     }
            //     pDenseMappingFrame = this->m_pPoseGraph->PopDepthFrames();
            //     nCount++;
            // }



            
            
        }

    }
}


void CollaborativeServer::Run(){
    this->m_tServerPlotter = std::thread(&ServerPlotter::Run, this->m_pPlotter);
    this->m_tAgentProcess = std::thread(&CollaborativeServer::AgentProcess, this);
    this->m_tPublishProcess = std::thread(&CollaborativeServer::PublishProcess, this);
}

