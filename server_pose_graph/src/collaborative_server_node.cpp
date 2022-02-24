
//Eigen
#include <eigen3/Eigen/Dense>
//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

//ROS
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <agent_msg/AgentMsg.h>
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>

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




//Sophus
#include "sophus/se3.h"



//Project
#include "../include/server_keyframe.h"
#include "../include/server_pose_graph.h"
#include "../include/parameters.h"
#include "../include/server_plotter.h"
#include "../include/collaborative_server_system.h"

#include "../ThirdParty/DVision/DVision.h"


using namespace std;



int main(int argc, char **argv)
{

    ros::init(argc, argv, "server_pose_graph");
    
    ros::NodeHandle n("~");
 
    int nAgentNum = 1;
  
    n.getParam("agent_num", nAgentNum);


    vector<string> gConfigPaths;
    for (int i=0;i<nAgentNum;i++){
        stringstream ss;
        string aLocalConfigFile;
        ss << (i+1);
        string aIndex;
        ss >> aIndex;
        n.getParam("config_file_"+aIndex, aLocalConfigFile);
        gConfigPaths.push_back(aLocalConfigFile);

    }
    cout << "Config Path Size: " << gConfigPaths.size() << endl;
    cout << "Agent Num: " << nAgentNum << endl;
 	string aConfigFile;
    n.getParam("config_file", aConfigFile);
    cv::FileStorage fsSettings(aConfigFile, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    string aProjectPath;
    fsSettings["project_path"] >> aProjectPath;

    string aVocFile = aProjectPath + "/support_files/brief_k10L6.bin";
    cout << "Vocabulary_file: " << aVocFile << endl;
    

    SERVER_BRIEF_PATTERN_FILE = aProjectPath + "/support_files/brief_pattern.yml";
    cout << "BRIEF_PATTERN_FILE" << SERVER_BRIEF_PATTERN_FILE << endl;

    CollaborativeServer * pServer = new CollaborativeServer(aVocFile, gConfigPaths, n);

    pServer->Run();


 




    ros::spin();
    return 0;
}
