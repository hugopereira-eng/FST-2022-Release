#include <ros/ros.h>
#include <fastslam2_0/fastslam2_0_handle.hpp>


int main(int argc, char **argv) {
    
    ros::init(argc, argv, "fastslam2_0");
    ros::NodeHandle nh("~");
    FastSlamHandle fastSlamHandle(nh);
    
    ros::spin();
    
    
    return 0;
}