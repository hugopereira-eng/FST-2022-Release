#include "ros/ros.h"
#include "fst_interface/fst_interface_handle.hpp"

int main (int argc, char** argv){
  ros::init(argc, argv, "fst_interface");
  ros::NodeHandle nh;
  FstInterfaceHandle fstInterfaceHandle(nh);
    
  ros::spin();            
              
  return 0;
}
