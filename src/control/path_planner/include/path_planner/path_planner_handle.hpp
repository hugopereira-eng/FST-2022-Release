#ifndef PATH_PLANNER_HANDLE_HPP
#define PATH_PLANNER_HANDLE_HPP

#include <ros/ros.h>
#include "common_msgs/ConeDetections.h"
#include "common_msgs/Cost.h"
#include <sensor_msgs/Image.h>
#include "path_planner/path_planner_pipeline.hpp"
#include <dynamic_reconfigure/server.h>
#include <path_planner/PathPlannerConfig.h>


class PathPlannerHandle {

public: 
  // Constructor
  PathPlannerHandle(ros::NodeHandle &nodeHandle);

  //Methods
  void advertiseToTopics(); 
  void subscribeToTopics();
  void subscribeToParametersServer(); 
  void publishToTopics();
  void run();

private: 
  ros::NodeHandle _nodeHandle;

  ros::Publisher _pubPath;
  ros::Publisher _pubMarker;
  ros::Publisher _pubDelaunayVisualization;
  ros::Publisher _pubAllPathsVisualization;
  ros::Publisher _pubCenterlineVisualization;
  ros::Publisher _pubCost;
  ros::Publisher _pubBestCost;
  ros::Publisher _pubSecondBestCost;
  ros::Publisher _pubThirdBestCost;

  ros::Subscriber _subConeDetectionsSlam;
  ros::Subscriber _subConeDectionsSensorFusion;
  ros::Subscriber _subCameraImage;
  ros::Subscriber _subPathPlannerActive;

  PathPlanner _pathPlanner;
  

  //server for Dynamic Ros Parameters
  dynamic_reconfigure::Server<path_planner::PathPlannerConfig> _server;

  void coneDetectionsSlamCallback(const common_msgs::ConeDetections &conesDetected);
  void coneDetectionsSensorFusionCallback(const common_msgs::ConeDetections &conesDetected);
  void pathPlannerActiveCallback(const std_msgs::Bool &pathPlannerActive);
  void cameraImageCallback(const sensor_msgs::Image &cameraImage);
  void pathPlannerParametersCallback(path_planner::PathPlannerConfig &config);
  void pathPlannerSlamActiveCallback(const std_msgs::Bool &slamActive);
};

#endif 