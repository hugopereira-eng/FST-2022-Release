#include "path_planner/path_planner_handle.hpp"

PathPlannerHandle::PathPlannerHandle(ros::NodeHandle &nodeHandle) : _nodeHandle(nodeHandle) {
  ROS_INFO("Constructing Path Planner Handle");
  advertiseToTopics();
  subscribeToTopics();
  subscribeToParametersServer();
}

void PathPlannerHandle::advertiseToTopics() {
  ROS_INFO("Path Planner will advertise to topics");
  _pubMarker =_nodeHandle.advertise<visualization_msgs::MarkerArray>("/control/path_planner/vis/mid_points_chosen", 1);
  _pubDelaunayVisualization = _nodeHandle.advertise<visualization_msgs::Marker>("/control/path_planner/vis/delaunay", 1);
  _pubAllPathsVisualization = _nodeHandle.advertise<visualization_msgs::Marker>("/control/path_planner/vis/all_paths", 1);
  _pubPath = _nodeHandle.advertise<nav_msgs::Path>("/control/path_planner/centerline", 1);
  _pubCenterlineVisualization = _nodeHandle.advertise<sensor_msgs::Image>("/control/path_planner/centerlineOnCamera", 1);

  //rqt_plot
  _pubBestCost = _nodeHandle.advertise<common_msgs::Cost>("control/path_planner/best_cost", 1);
  _pubSecondBestCost = _nodeHandle.advertise<common_msgs::Cost>("control/path_planner/second_best_cost", 1);
  _pubThirdBestCost = _nodeHandle.advertise<common_msgs::Cost>("control/path_planner/third_best_cost", 1);
}

void PathPlannerHandle::subscribeToTopics() {
  ROS_INFO("Path Planner will subscribe to topics");
  _subConeDetectionsSlam = _nodeHandle.subscribe("/estimation/slam/coneFrontDetections", 1, &PathPlannerHandle::coneDetectionsSlamCallback, this);
  _subConeDectionsSensorFusion = _nodeHandle.subscribe("/perception/cone_detections", 1, &PathPlannerHandle::coneDetectionsSensorFusionCallback, this);
  _subCameraImage = _nodeHandle.subscribe("/perception/darknet_ros/detection_image", 1, &PathPlannerHandle::cameraImageCallback, this);
  _subPathPlannerActive = _nodeHandle.subscribe("/common/mission_tracker/path_planner_active", 1, &PathPlannerHandle::pathPlannerActiveCallback, this);
}

void PathPlannerHandle::subscribeToParametersServer() {
  dynamic_reconfigure::Server<path_planner::PathPlannerConfig>::CallbackType f;

  f = boost::bind(&PathPlannerHandle::pathPlannerParametersCallback, this, _1);
  _server.setCallback(f);
}

void PathPlannerHandle::run() {
  _pathPlanner.runAlgorithm();
  publishToTopics();
}

void PathPlannerHandle::publishToTopics() {

  _pubPath.publish(_pathPlanner.getPath());

  _pubMarker.publish(_pathPlanner.getBestTrajectory());
  _pubDelaunayVisualization.publish(_pathPlanner.getDelaunayMarkerLines());
  _pubAllPathsVisualization.publish(_pathPlanner.getMidpointConnectionsLines());

  _pubBestCost.publish(_pathPlanner.getFirstCost());
  _pubSecondBestCost.publish(_pathPlanner.getSecondCost());
  _pubThirdBestCost.publish(_pathPlanner.getThirdCost());
}

void PathPlannerHandle::coneDetectionsSlamCallback(const common_msgs::ConeDetections &conesDetected) {
  _pathPlanner.setConeDetectionsSlam(conesDetected);
}

void PathPlannerHandle::coneDetectionsSensorFusionCallback(const common_msgs::ConeDetections &conesDetected) {
  _pathPlanner.setConeDetectionsSensorFusion(conesDetected);
}

void PathPlannerHandle::pathPlannerParametersCallback(path_planner::PathPlannerConfig &config) {
  _pathPlanner.updatePipelineParameters(config);
}

void PathPlannerHandle::pathPlannerActiveCallback(const std_msgs::Bool &pathPlannerActive) {
  _pathPlanner.setPathPlannerActive(pathPlannerActive);
}

void PathPlannerHandle::cameraImageCallback(const sensor_msgs::Image &cameraImage) {
  _pathPlanner.updateCameraImage(cameraImage);
}
