#include "ros/ros.h"
#include "mission_tracker/mission_tracker_handle.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "mission_tracker");
  ros::NodeHandle nh("~");

  MissionTrackerHandle missonTrackerHandle(nh);

  ros::Rate rate(200);

  while (ros::ok())
  {

    missonTrackerHandle.run();

    ros::spinOnce();
    rate.sleep();
  }
  

  return 0;
}