#include <ros/ros.h>
#include <graphslam/graphslam_handle.hpp>


int main(int argc, char **argv) {
    
    ros::init(argc, argv, "graphslam");
    ros::NodeHandle nh("~");
    GraphSlamHandle GraphSlamHandle(nh);
    
    ros::spin();
    
    
    return 0;
}