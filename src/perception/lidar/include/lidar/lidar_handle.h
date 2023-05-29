#ifndef LIDAR_HANDLE_HPP
#define LIDAR_HANDLE_HPP

#include "lidar/lidar_pipeline.h"
#include <dynamic_reconfigure/server.h>
class ConeDetectorHandle {

public: 
	// Constructor
	ConeDetectorHandle(ros::NodeHandle &nodeHandle);
	// Methods
	void advertiseToTopics(); 
	void subscribeToTopics();
	void subscribeToParametersServer(); 
	void publishToTopics();
	void run();

private: 
	ros::NodeHandle _nodeHandle;

	// Publishers
	ros::Publisher _pubClusteredPoints;
	ros::Publisher _pubClusterCentroid;
	ros::Publisher _pubClusterMarker;
	ros::Publisher _pubConeMarker;
	ros::Publisher _pubFilteredPointcloud;
	ros::Publisher _pubRansacPointcloud;  

	// Subscribers
	ros::Subscriber _subLidarPoints;

	ConeDetector _coneDetector;

	int _pipeline;

	// Server for Dynamic Ros Parameters
	dynamic_reconfigure::Server<lidar::LidarConfig> _server;

	void lidarPointsCallback(const sensor_msgs::PointCloud2 &pointCloud);
	void lidarParametersCallback(lidar::LidarConfig &config, uint32_t level);
};

#endif