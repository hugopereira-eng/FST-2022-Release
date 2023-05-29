#include "ros/ros.h"
#include "state_estimation/state_estimation_handle.hpp"

int main (int argc, char** argv) {
  ros::init(argc, argv, "state_estimation");
  ros::NodeHandle nh("~");

  StateEstimationHandle StateEstimationHandle(nh);
  ros::Rate loop_rate(100); //Running at 100hz

  while (ros::ok()) {
    StateEstimationHandle.runAlgorithm(); 
    ros::spinOnce();                // Keeps node alive basically
    loop_rate.sleep();              // Sleep for loop_rate
  }
  return 0;
}

