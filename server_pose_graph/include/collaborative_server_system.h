#ifndef COLLABORATIVE_SERVER_SYSTEM_H_
#define COLLABORATIVE_SERVER_SYSTEM_H_

//Headers.
//ROS
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <agent_msg/AgentMsg.h>
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>

#include <cstdio>
#include <cstring>
#include <map>


#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <stereo_msgs/DisparityImage.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <chisel_ros/SaveMeshService.h>


//STD
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <string>
#include <vector>
#include <map>
#include <iterator>
#include <stdio.h>


//Eigen
#include <eigen3/Eigen/Dense>


//Sophus
#include "sophus/se3.h"

//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>


//Project
#include "../include/server_keyframe.h"
#include "../include/server_pose_graph.h"
#include "../include/parameters.h"
#include "../include/server_plotter.h"

#include "../ThirdParty/DVision/DVision.h"



using namespace std;



class CollaborativeServer
{
public:
	CollaborativeServer(string aVocFile, string aConfigFile, ros::NodeHandle iNodeHandler);
	
	CollaborativeServer(string aVocFile, vector<string> gAllConfigFiles, ros::NodeHandle iNodeHandler);
	


	void ImageCallback(const sensor_msgs::ImageConstPtr &pImageMsg);
	
	void AgentCallback(const agent_msg::AgentMsgConstPtr &pAgentMsg);

	void InitializeCamera(agent_msg::AgentMsgConstPtr pAgentMsg);

	void PublishDenseInfo(	Sophus::SE3 mRefPose_wc,
							std_msgs::Header iHeader,
							cv::Mat & mDepthMap,
							cv::Mat & mColorMap,
							cv::Mat & mK,
							int nWidth, int nHeight);


	void AgentProcess();

	void PublishProcess();

	void Run();

	bool SaveMesh();

private:
	void SendPointCloud(const std_msgs::Header & iHeader,
						cv::Mat & mDepthMap,
						cv::Mat & mColorImage);




	map<int, ServerCamera *> m_dCameras;

	//Mutices used when parallel different threads.
	std::mutex m_mBufMutex;           //std::mutex m_buf;
	std::mutex m_mProcessMutex;       //std::mutex m_process;
	std::mutex m_mAgentBufMutex;      //std::mutex m_agent_msg_buf;
	std::mutex m_mImageBufMutex;


	queue<agent_msg::AgentMsgConstPtr> m_gAgentMsgBuf;    
	queue<sensor_msgs::ImageConstPtr> m_gImageBuf;


	ServerPoseGraph * m_pPoseGraph;

	bool m_bStartFlag = true;

	//Count the number of frames.
	int m_nFrameCount = 0;            //int frame_cnt = 0;


	//Pubilsher
	ros::Publisher m_iUpdatedPosePublisher;
	ros::Publisher m_iUpdatedImagePublisher;

	//Publish the image, pose, depth map to openChisel.
	ros::Publisher m_iDepthMapPublisher;
	ros::Publisher m_iColorImagePublisher;
	ros::Publisher m_iDepthCameraInfoPublisher;
	ros::Publisher m_iColorCameraInfoPublisher;
	ros::Publisher m_iPointCloudPublisher;

	//Subscriber
	ros::Subscriber m_iAgentSubscriber;
	ros::Subscriber m_iImageSubscriber;


	//Save the mesh
	ros::ServiceClient m_iSaveMeshClient;


	string m_aVocFile;
	string m_aConfigFile;
	vector<string> m_gAllConfigFiles;

	//Plotting module.
	ServerPlotter * m_pPlotter;

	std::thread m_tServerPlotter;
	std::thread m_tAgentProcess;
	std::thread m_tPublishProcess;


};



#endif