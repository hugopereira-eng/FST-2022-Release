#include "lidar/lidar_handle.h"

/**
*	Name: ConeDetectorHandle.
*	Description: ConeDetector Handle construction. It advertises and subscribes all the necessary topics.
*	Inputs: ROS NodeHandle
*	Output: void
*/
ConeDetectorHandle::ConeDetectorHandle(ros::NodeHandle &nodeHandle) : _nodeHandle(nodeHandle) {
	ROS_INFO("Constructing ConeDetector Handle");
	advertiseToTopics();
	subscribeToTopics();
	subscribeToParametersServer();
}

/**
*	Name: advertiseToTopics.
*	Description: Creates all the necessary ROS publishers. It's also where the name of the topics is defined.
*	Inputs: none
*	Output: void
*/
void ConeDetectorHandle::advertiseToTopics() {
    _pubClusteredPoints = _nodeHandle.advertise<sensor_msgs::PointCloud2>("/perception/lidar/clustered_points", 1); 
    _pubClusterCentroid = _nodeHandle.advertise<common_msgs::ConeDetections>("/perception/cone_detections", 1); 
    _pubClusterMarker = _nodeHandle.advertise<visualization_msgs::MarkerArray>("/perception/lidar/vis/cluster_markers", 1);
    _pubConeMarker = _nodeHandle.advertise<visualization_msgs::MarkerArray>("/perception/lidar/vis/cone_markers", 1);
    _pubFilteredPointcloud = _nodeHandle.advertise<sensor_msgs::PointCloud2>("/perception/lidar/vis/filtered_pointcloud", 1);
    _pubRansacPointcloud = _nodeHandle.advertise<sensor_msgs::PointCloud2>("/perception/lidar/vis/ransac_pointcloud", 1);
}

/**
*	Name: subscribeToTopics.
*	Description: Subscribes to the correct topic, depending on the LiDAR that is being used.
*	Inputs: none
*	Output: void
*/
void ConeDetectorHandle::subscribeToTopics() {
    _subLidarPoints = _nodeHandle.subscribe("/velodyne_points", 1, &ConeDetectorHandle::lidarPointsCallback, this);
}

/**
*	Name: subscribeToParametersServer.
*	Description: Subscribes to the dynamic parameters server.
*	Inputs: none
*	Output: void
*/
void ConeDetectorHandle::subscribeToParametersServer() {
    dynamic_reconfigure::Server<lidar::LidarConfig>::CallbackType f;
    f = boost::bind(&ConeDetectorHandle::lidarParametersCallback, this, _1, _2);
    _server.setCallback(f);  
}

/**
*	Name: run.
*	Description: Runs the LiDAR pipeline algorithms and publishes the results to the correspondent topics.
*	Inputs: none
*	Output: void
*/
void ConeDetectorHandle::run() {
    _coneDetector.runAlgorithm();
    publishToTopics();
}

/**
*	Name: publishToTopics.
*	Description: Publishes the results to the correspondent topics.
*	Inputs: none
*	Output: void
*/
void ConeDetectorHandle::publishToTopics() {
    _pubClusteredPoints.publish(_coneDetector.getClusteredPoints());
    _pubClusterCentroid.publish(_coneDetector.getClusterCentroids());
    _pubClusterMarker.publish(_coneDetector.getClusterMarkers());
    _pubConeMarker.publish(_coneDetector.getConeMarkers());
    _pubFilteredPointcloud.publish(_coneDetector.getFilteredPointcloud());
    _pubRansacPointcloud.publish(_coneDetector.getRansacPointcloud());
}

/**
*	Name: lidarPointsCallback.
*	Description: Callback to update the pointcloud obtained from the LiDAR.
*	Inputs: Pointcloud of the type sensor_msgs::PointCloud2
*	Output: void
*/
void ConeDetectorHandle::lidarPointsCallback(const sensor_msgs::PointCloud2 &pointCloud) {
    _coneDetector.updatePointCloud(pointCloud);
    run();
}

/**
*	Name: lidarParametersCallback.
*	Description: Callback to update the dynamic parameters set on rqt_reconfigure.
*	Inputs: Config file with the parameters to be changed.
*	Output: void
*/
void ConeDetectorHandle::lidarParametersCallback(lidar::LidarConfig &config, uint32_t level) {
    _coneDetector.updatePipelineParameters(config, level);
    run();   
}
