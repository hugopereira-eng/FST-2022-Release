#include "ros/ros.h"
#include "new_planner/new_planner_handle.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "new_planner");
    ros::NodeHandle nh;

    NewPlannerHandle NewPlannerHandle(nh);
    
    ros::spin();            

    return 0;
}

