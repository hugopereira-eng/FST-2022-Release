#ifndef LIDAR_PIPELINE_HPP
#define LIDAR_PIPELINE_HPP

#include <ros/ros.h>
#include <as_lib/common.h>
#include "lidar/lidar_passthrough_filter.h"
#include "lidar/lidar_ransac.h"
#include "lidar/lidar_euclidian_clustering.h"
#include "lidar/lidar_cone_reconstruction.h"
#include "lidar/lidar_cone_identification.h"
#include <lidar/LidarConfig.h>

#include <sensor_msgs/PointCloud2.h>
#include <common_msgs/ConeDetections.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_msgs/PointIndices.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

typedef pcl::PointXYZI PclPoint;
typedef pcl::PointCloud<PclPoint> PclPointCloud;
typedef enum Pipeline {
    THESIS, DARKNET
} Pipeline;
class ConeDetector {

public:
	// Constructor:
	ConeDetector();
	// Getters:
	sensor_msgs::PointCloud2 const & getClusteredPoints() const;
	common_msgs::ConeDetections const & getClusterCentroids() const;
	visualization_msgs::MarkerArray const & getClusterMarkers() const;
	visualization_msgs::MarkerArray const & getConeMarkers() const;
	sensor_msgs::PointCloud2 const & getFilteredPointcloud() const;
	sensor_msgs::PointCloud2 const & getRansacPointcloud() const;
	
	void updatePipelineParameters(const lidar::LidarConfig &config, uint32_t level);
	void updatePointCloud(const sensor_msgs::PointCloud2 &newPointCloud);
	void runAlgorithm();

protected:
	// Methods
	void convertFromRosToPcl();
	void passThroughFiltering();
	void removeGround();
	void detectClusters();
	void calculateClusterCentroids();
	void visualizeClusterCentroid();
	void convertFromPclToRos();
	void reconstructCones();
	void validateCluster();
	void visualizeConeCentroid();
	void transformClusters();

	sensor_msgs::PointCloud2 _pointCloud;
	sensor_msgs::PointCloud2 _filteredPointCloud;
	sensor_msgs::PointCloud2 _ransacPointCloud;
	sensor_msgs::PointCloud2 _clusteredPoints;
	PclPointCloud::Ptr _pclPointCloud;
	PclPointCloud::Ptr _pclPointCloudGround;
	common_msgs::ConeDetections _clusterDetections;
	std::vector<pcl::PointIndices> _clusterIndices;
	std::vector<pcl::PointIndices> _validatedClustersIndex;
	visualization_msgs::MarkerArray _clusterCentroidMarkers;
	visualization_msgs::MarkerArray _coneCentroidMarkers;
	geometry_msgs::TransformStamped _transform;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tfListener;

	// Objects needed for the detection algorithm sequence;
	PassthroughFilter _passthroughFilter;
	Ransac _ransacAlgorithm;
	EuclidianClustering _clusteringMethod;
	ConeReconstruction _coneReconstruction;
	ConeIdentification _coneIdentification;
};

#endif