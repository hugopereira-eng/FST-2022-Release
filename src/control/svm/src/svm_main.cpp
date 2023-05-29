#include "ros/ros.h"
#include "svm/svm_handle.hpp"
#include <chrono>

int main(int argc, char** argv) {
  ros::init(argc, argv, "svm");
  ros::NodeHandle nh;

  SVMHandle SVMHandle(nh);

  ros::Rate loop_rate(45);
  while (ros::ok()) {
    SVMHandle.run(); 
    ros::spinOnce();                // Keeps node alive basically
    loop_rate.sleep();              // Sleep for loop_rate
    
  }
  return 0;
}

