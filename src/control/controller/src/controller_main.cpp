#include "ros/ros.h"
#include "controller/controller_handle.hpp"
#include <chrono>

int main(int argc, char** argv) {
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;

  ControllerHandle controllerHandle(nh);

  ros::Rate loop_rate(100);
  while (ros::ok()) {

    controllerHandle.run(); 

    ros::spinOnce();                // Keeps node alive basically
    loop_rate.sleep();              // Sleep for loop_rate
  }
  return 0;
}

