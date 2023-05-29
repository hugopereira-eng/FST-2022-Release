#include "ros/ros.h"
#include "launcher/launcher_handle.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "launcher");
  ros::NodeHandle nh;

  LauncherHandle launcherHandle(nh);

 // ros::Rate loop_rate(10);

 // while (ros::ok()) {

 //   common_msgs::Mission mission = launcherHandle.getMission();
 //   launcherHandle.launcher(mission);

    ros::spin();
 //   ros::spinOnce();                // Keeps node alive basically
 //   loop_rate.sleep();              // Sleep for loop_rate
 // }
  return 0;
}