#include "ros/ros.h"
#include "path_planner/path_planner_handle.hpp"
#include <chrono>

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_planner");
  ros::NodeHandle nh;

  PathPlannerHandle pathPlannerHandle(nh);

  ros::Rate loop_rate(45);
  while (ros::ok()) {

    pathPlannerHandle.run(); 
    ros::spinOnce();                // Keeps node alive basically
    loop_rate.sleep();              // Sleep for loop_rate
  }
  return 0;
}

