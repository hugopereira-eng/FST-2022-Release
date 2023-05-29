#ifndef CONTROLLER_PIPELINE_HPP
#define CONTROLLER_PIPELINE_HPP

#include "common_msgs/ControlCmd.h"
#include "common_msgs/ControlMonitoring.h"
#include "common_msgs/CarVelocity.h"
#include "common_msgs/CarVelocityArray.h"
#include "common_msgs/PointToFollow.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"
#include "controller/controller_pid.hpp"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include <as_lib/common.h>
#include <ros/ros.h>
#include <numeric>
#include <deque>

enum class ControlState {
  CONTROL_OFF,
  CONTROL_ON,
  BRAKING_MODE,
  INSPECTION
};

class Controller {
  
public:

  // Constructor
  Controller();

  // Getters
  common_msgs::ControlCmd const & getControlCommand() const;
  common_msgs::PointToFollow const &getPointToFollow() const;
  common_msgs::PointToFollow const &getPointAhead() const;
  visualization_msgs::Marker const & getMarkerPointToFollow() const;
  visualization_msgs::Marker const & getMarkerPointAhead() const;
  std_msgs::Float64 const & getSpeedTarget() const;
  bool const shouldPublish() const;

  // Setters
  void setMission(const std::string &mission);
  void setPathToFollow(const nav_msgs::Path &path);
  void setSlamPath(const nav_msgs::Path &slamPath);
  void setCurrentVelocity(const common_msgs::CarVelocity &vel);
  void changeControllerState(const common_msgs::ControlMonitoring &msg);
  void setSlamPose(const nav_msgs::Odometry &slamPose);
  void setInspectionSteering(const std_msgs::Float64 &inpectionSteering);
  
  //Methods
  void runAlgorithm();
  void printControlState();

private:

  ControlState _controlState;

  nav_msgs::Path _pathToFollow;
  nav_msgs::Path _slamPath;
  nav_msgs::Path _pathPlannerPath;

  common_msgs::CarVelocity _currentVelocity;
  nav_msgs::Odometry _carPose;
  float _steeringInspection;
  float _filteredSteering = -1;
  std_msgs::Float64 _speedTarget;

  PID _pid;

  common_msgs::ControlCmd _controlCmd;
  common_msgs::ControlCmd _previousControlCmd;
  common_msgs::PointToFollow _pointToFollow;
  common_msgs::PointToFollow _pointAhead;
  visualization_msgs::Marker _markerPointToFollow;
  visualization_msgs::Marker _markerPointAhead;

  bool _publish = false;
  int _index = 0;
  
  double _controllerfrequency;
  bool _UseAdaptiveControl;
  double _accelK;
  double _accelKi;
  double _brakeK;
  double _steeringK;
  double _steeringFilterK;
  double _speedRef;
  double _minSpeed;
  double _speedAngle;
  double _maxSpeed;
  double _lookAheadTime;
  double _furtherLookAheadTime;
  double _minDistanceSetPoint;
  double _minStaSteering;
  double _maxStaSteering;
  double _maxSteeringChange;
  int _mission;

  void LoadParameters();
  void purePursuitControlCmd();
  void brakingControlCmd();
  void inspectionControlCmd();
  void followPath();
  void followSlamTrajectory();
  void computeCommands(geometry_msgs::PoseStamped, std_msgs::Header);
  int point2Follow(int, int, float, nav_msgs::Path);
  void getNearestIndex();
  float getDistance(double);
  double getYawFromQuaternion(nav_msgs::Odometry);
  float saturateThrottle(float _throttle);
  void computeAdaptiveControl();

  //visualization
  void setMarkerPointToFollow(double, double, std_msgs::Header, std::string);
  void setMarkerPointAhead(double, double, std_msgs::Header, std::string);
};

#endif //CONTROLLER_PIPELINE_HPP
