#pragma once

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <string>
using namespace std;

// extern camodocal::CameraPtr m_camera;
// extern Eigen::Vector3d tic;
// extern Eigen::Matrix3d qic;
// extern ros::Publisher pub_match_img;
// extern ros::Publisher pub_match_points;
extern int SERVER_VISUALIZATION_SHIFT_X;
extern int SERVER_VISUALIZATION_SHIFT_Y;

// //This path can be loaded from the node file.
// extern std::string BRIEF_PATTERN_FILE;

// extern std::string POSE_GRAPH_SAVE_PATH;
// extern int ROW;
// extern int COL;
// extern std::string VINS_RESULT_PATH;
// extern int DEBUG_IMAGE;
// extern int FAST_RELOCALIZATION;


extern string SERVER_BRIEF_PATTERN_FILE;


//Extrinsics of the VIO system.
// extern map<int, Eigen::Matrix3d> dRotation_ic;
// extern map<int, Eigen::Vector3d> dTranslation_ic;


