#include "ros/ros.h"
#include "fastslam/fastslam_handle.hpp"

int main (int argc, char** argv) {
  ros::init(argc, argv, "fastslam");
  ros::NodeHandle nh("~");

  FastSlamHandle fastSlamHandle(nh);
 
  ros::spin();

  return 0;
}