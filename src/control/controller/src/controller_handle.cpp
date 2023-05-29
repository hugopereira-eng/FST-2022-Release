#include "controller/controller_handle.hpp"

/**
*	Name: ControllerHandle.
*	Description: Controller Handle construction. It advertises and subscribes all the necessary topics.
*	Inputs: ROS NodeHandle
*	Output: void
*/
ControllerHandle::ControllerHandle(ros::NodeHandle &nodeHandle) : _nodeHandle(nodeHandle) {
  ROS_INFO("Constructing Controller Handle");
  advertiseToTopics();
  subscribeToTopics();

  _timerControlState = nodeHandle.createTimer(ros::Duration(0.5), &ControllerHandle::printControlState, this, false);
}

/**
*	Name: advertiseToTopics.
*	Description: Creates all the necessary ROS publishers. It's also where the name of the topics is defined.
*	Inputs: none
*	Output: void
*/
void ControllerHandle::advertiseToTopics() {
  ROS_INFO("Controller will advertise to topics");
  _pubControlCmd = _nodeHandle.advertise<common_msgs::ControlCmd>("/control/controller/control_cmd", 1); 
  _pubPointToFollow = _nodeHandle.advertise<common_msgs::PointToFollow>("/control/controller/point_to_follow", 1);
  _pubPointAhead = _nodeHandle.advertise<common_msgs::PointToFollow>("/control/controller/point_ahead", 1);
  _pubMarkerPointToFollow = _nodeHandle.advertise<visualization_msgs::Marker>("/control/controller/vis/point_to_follow", 1);
  _pubMarkerPointAhead = _nodeHandle.advertise<visualization_msgs::Marker>("/control/controller/vis/point_ahead", 1);
  _pubSpeedTarget = _nodeHandle.advertise<std_msgs::Float64>("/control/controller/speed_target", 1);
}

/**
*	Name: subscribeToTopics.
*	Description: Subscribes to the necessary topics.
*	Inputs: none.
*	Output: void.
*/
void ControllerHandle::subscribeToTopics() {
  ROS_INFO("Controller will subscribe to topics");

  // path (path planner + slam)
  _subPathPlanner = _nodeHandle.subscribe("/control/path_planner/centerline", 1, &ControllerHandle::pathPlannerCallback, this);
  _subSlamPath = _nodeHandle.subscribe("/estimation/slam/centerline", 1, &ControllerHandle::slamPathCallback, this);

  // monitoring topics
  _subControlMonitoring = _nodeHandle.subscribe("/common/mission_tracker/control_monitoring", 1, &ControllerHandle::controlMonitoringCallback, this);
  
  // estimation topics
  _subCurrentVelocity = _nodeHandle.subscribe("/estimation/state_estimation/velocity", 1, &ControllerHandle::currentVelocityCallback, this);
  _subSlamPose = _nodeHandle.subscribe("/estimation/slam/odometry", 1, &ControllerHandle::slamPoseCallback, this);
  
  // inspection
  _subInspectionSteering = _nodeHandle.subscribe("/common/mission_tracker/inspection_steering", 1, &ControllerHandle::inspectionSteeringCallback, this);
}

/**
*	Name: run.
*	Description: Runs the Controller pipeline algorithms and publishes topics when needed.
*	Inputs: none.
*	Output: void.
*/
void ControllerHandle::run() {
  _controller.runAlgorithm();
  if (_controller.shouldPublish()) publishToTopics();
}

/**
*	Name: printControlState.
*	Description: Prints the Control State, according to a timer set in the constructor
*	Inputs: Timer.
*	Output: void.
*/
void ControllerHandle::printControlState(const ros::TimerEvent &event) {
  _controller.printControlState();
}


/**
*	Name: publishToTopics.
*	Description: Publishes the results to the correspondent topics.
*	Inputs: none.
*	Output: void.
*/
void ControllerHandle::publishToTopics() {
  _pubControlCmd.publish(_controller.getControlCommand());
  _pubPointToFollow.publish(_controller.getPointToFollow());
  _pubPointAhead.publish(_controller.getPointAhead());
  _pubMarkerPointToFollow.publish(_controller.getMarkerPointToFollow());
  _pubMarkerPointAhead.publish(_controller.getMarkerPointAhead());
  _pubSpeedTarget.publish(_controller.getSpeedTarget());
}

/**
*	Name: pathPlannerCallback.
*	Description: Callback to update the path comming from the path planner algorithm.
*	Inputs: Path of the type nav_msgs::Path.
*	Output: void.
*/
void ControllerHandle::pathPlannerCallback(const nav_msgs::Path &path) {
  //ROS_INFO("Received path planner");
  _controller.setPathToFollow(path);
}

/**
*	Name: slamPathCallback.
*	Description: Callback to update the centerline coming from SLAM.
*	Inputs: slamPath of the type nav_msgs::Path.
*	Output: void.
*/
void ControllerHandle::slamPathCallback(const nav_msgs::Path &slamPath) {
  //ROS_INFO("Received slam path");
  _controller.setSlamPath(slamPath);
}

/**
*	Name: currentVelocityCallback.
*	Description: Callback to update the cars current velocity obtained from the state estimator.
*	Inputs: Velocity of the type common_msgs::CarVelocity.
*	Output: void.
*/
void ControllerHandle::currentVelocityCallback(const common_msgs::CarVelocity &vel){
  //ROS_INFO("Received ahrs velocity");
  _controller.setCurrentVelocity(vel);
}

/**
*	Name: controlMonitoringCallback.
*	Description: Callback to update the cars control state.
*	Inputs: Control Monitoring message.
*	Output: void.
*/
void ControllerHandle::controlMonitoringCallback(const common_msgs::ControlMonitoring &msg) {
  //ROS_INFO("Received control monitoring");
  _controller.changeControllerState(msg);
}

/**
*	Name: slamPoseCallback.
*	Description: Callback to update the cars pose, obtained from the SLAM algorithm.
*	Inputs: Cars pose of type nav_msgs::Odometry.
*	Output: void.
*/
void ControllerHandle::slamPoseCallback(const nav_msgs::Odometry &slamPose){
  //ROS_INFO("Received slam pose");
  _controller.setSlamPose(slamPose);
}

/**
*	Name: inspectionSteeringCallback.
*	Description: Callback to set the steering references for the inspection mission.
*	Inputs: Steering of type std_msgs::Float64.
*	Output: void.
*/
void ControllerHandle::inspectionSteeringCallback(const std_msgs::Float64 &inspectionSteering) {
  //ROS_INFO("Received inspection steering");
  _controller.setInspectionSteering(inspectionSteering);
}