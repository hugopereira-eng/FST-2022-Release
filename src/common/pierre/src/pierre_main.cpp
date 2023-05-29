#include "ros/ros.h"
#include "pierre/pierre_handle.hpp"
 
int main(int argc, char** argv) {
  ros::init(argc, argv, "pierre");
  ros::NodeHandle nh;
  PierreHandle pierreHandle(nh);
    
  ros::spin();                // Keeps node alive basically
  
  return 0;
}
