#ifndef CONTROLLER_HANDLE_HPP
#define CONTROLLER_HANDLE_HPP

#include "controller/controller_pipeline.hpp"

class ControllerHandle {

public:
  // Constructor
  ControllerHandle(ros::NodeHandle &nodeHandle);

  //Methods
  void initParameters();
  void advertiseToTopics();
  void subscribeToTopics();
  void publishToTopics();
  void run();

private:
  ros::NodeHandle _nodeHandle;

  ros::Publisher _pubControlCmd;
  ros::Publisher _pubPointToFollow;
  ros::Publisher _pubPointAhead;
  ros::Publisher _pubMarkerPointToFollow;
  ros::Publisher _pubMarkerPointAhead;
  ros::Publisher _pubSpeedTarget;

  ros::Subscriber _subPathPlanner;
  ros::Subscriber _subSlamPath;
  ros::Subscriber _subCurrentVelocity;
  ros::Subscriber _subControlMonitoring;
  ros::Subscriber _subSlamPose;
  ros::Subscriber _subInspectionSteering;

  ros::Timer _timerControlState;

  Controller _controller;

  void printControlState(const ros::TimerEvent &);

  void pathPlannerCallback(const nav_msgs::Path &path);
  void slamPathCallback(const nav_msgs::Path &slamPath);
  void currentVelocityCallback(const common_msgs::CarVelocity &vel);
  void controlMonitoringCallback(const common_msgs::ControlMonitoring &msg);  
  void slamPoseCallback(const nav_msgs::Odometry &slamPose);
  void inspectionActiveCallback(const std_msgs::Bool &inpectionActive);
  void inspectionSteeringCallback(const std_msgs::Float64 &inspectionSteering);

  std::string _mission;
};

#endif //CONTROLLER_HANDLE_HPP
