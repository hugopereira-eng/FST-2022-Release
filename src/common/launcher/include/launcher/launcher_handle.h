#ifndef LAUNCHER_HANDLE_HPP
#define LAUNCHER_HANDLE_HPP

#include <ros/ros.h>
#include "common_msgs/Mission.h"
using namespace std;
#include<string.h>
#include "shared/as_lib/common.h"

class LauncherHandle {

public: 
  // Constructor
  LauncherHandle(ros::NodeHandle &nodeHandle);

  //Methods
  void subscribeToTopics();
  void launcher();
  void missionCallback(const common_msgs::Mission &mission);

  //Getter
  //common_msgs::Mission getMission() const;

private: 
  ros::NodeHandle _nodeHandle;
  ros::Subscriber _missionSubscriber;

  as_missions _mission;
  as_missions _oldMission;

};

#endif