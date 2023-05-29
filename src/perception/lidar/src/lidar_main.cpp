#include "ros/ros.h"
#include "lidar/lidar_handle.h"

int main(int argc, char** argv) {

	ros::init(argc, argv, "lidar_cone_detector");
	ros::NodeHandle nh("~");
	ConeDetectorHandle coneDetectorHandle(nh);

	ros::spin();

	return 0;
}