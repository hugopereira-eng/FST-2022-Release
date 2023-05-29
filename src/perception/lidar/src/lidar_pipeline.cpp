#include "lidar/lidar_pipeline.h"

/****************************************
*             General Tasks             *
****************************************/

// Constructor
ConeDetector::ConeDetector(): _pclPointCloud(new PclPointCloud), tfListener(tf_buffer_) {
	ROS_INFO("Constructing ConeDetector");
};

// Getters
sensor_msgs::PointCloud2 const & ConeDetector::getClusteredPoints() const { return _clusteredPoints;}
common_msgs::ConeDetections const & ConeDetector::getClusterCentroids() const { return _clusterDetections; }
visualization_msgs::MarkerArray const & ConeDetector::getClusterMarkers() const { return _clusterCentroidMarkers; }
visualization_msgs::MarkerArray const & ConeDetector::getConeMarkers() const { return _coneCentroidMarkers; }
sensor_msgs::PointCloud2 const & ConeDetector::getFilteredPointcloud() const {return _filteredPointCloud; }
sensor_msgs::PointCloud2 const & ConeDetector::getRansacPointcloud() const {return _ransacPointCloud; }

/**
*	Name: updatePipelineParameters.
*	Description: Executed by callback function to update the parameters of the various stages of the detection algorithm.
*	Inputs: Config file with the parameters to be changed.
*	Output: void
*/
void ConeDetector::updatePipelineParameters(const lidar::LidarConfig &config, uint32_t level) {
	_passthroughFilter.setMinX(config.passthrough_filter_min_x);
	_passthroughFilter.setMaxX(config.passthrough_filter_max_x);
	_passthroughFilter.setMinY(config.passthrough_filter_min_y);
	_passthroughFilter.setMaxY(config.passthrough_filter_max_y);
	_passthroughFilter.setMinZ(config.passthrough_filter_min_z);
	_passthroughFilter.setMaxZ(config.passthrough_filter_max_z);
	_passthroughFilter.setDistanceFilter(config.passthrough_filter_distance_filter);
	_ransacAlgorithm.setMaxIteration(config.ransac_max_iterations);
	_ransacAlgorithm.setDistanceThreshold(config.ransac_distance_threshold);
	_clusteringMethod.setTolerance(config.cluster_tolerance);
	_clusteringMethod.setClusterMinSize(config.cluster_min_size);
	_clusteringMethod.setCusterMaxSize(config.cluster_max_size);
	_coneReconstruction.setCylinderRadius(config.reconstruction_cylinder_radius);
	_coneReconstruction.setCylinderHeight(config.reconstruction_cylinder_height);
	_coneIdentification.setVerticalResolution(config.identification_vertical_resolution);
	_coneIdentification.setCylinderRadius(config.identification_cylinder_radius);
	_coneIdentification.setCylinderHeight(config.identification_cylinder_height);
}

/**
*	Name: updatePointCloud.
*	Description: Executed by a callback to update the _pointCloud with the one obtained from the LiDAR.
*	Inputs: Pointcloud of the type sensor_msgs::PointCloud2
*	Output: void
*/
void ConeDetector::updatePointCloud(const sensor_msgs::PointCloud2 &newPointCloud) {
	_pointCloud = newPointCloud;
}

/**
*	Name: convertFromRosToPcl.
*	Description: Converts the raws from pointcloud2 to pcl.
*	Inputs: none
*	Output: void
*/
void ConeDetector::convertFromRosToPcl() {    
	pcl::fromROSMsg(_pointCloud, *_pclPointCloud);
}

/**
*	Name: convertFromPclToRos.
*	Description: Converts the filtered results from pcl back to pointcloud2
*	Inputs: _pclPointCloud with the filtered results.
*	Output: _clusteredPoints sensor_msgs::PointCloud2 with the filtered results.
*	Temporary. Might be deleted from here in a final verbose version.
*/
void ConeDetector::convertFromPclToRos() {							
	pcl::toROSMsg(*_pclPointCloud, _clusteredPoints);
	_clusteredPoints.header = _pointCloud.header;
}

/**
*	Name: visualizeClusterCentroid.
*	Description: Used to visualize the cluster's centroid using markers on Rviz.
*	Inputs: none
*	Output: void
*	Temporary. Might be deleted from here in a final verbose version.
*/
void ConeDetector::visualizeClusterCentroid() {
	std::vector<visualization_msgs::Marker> markerArray(_clusterDetections.cone_detections.size()+1);
	markerArray[0].action = visualization_msgs::Marker::DELETEALL;
	for (size_t i = 1; i < _clusterDetections.cone_detections.size() + 1; ++i){
		markerArray[i].header.frame_id = _pointCloud.header.frame_id;
		markerArray[i].header.stamp = _pointCloud.header.stamp;
		markerArray[i].ns = "Cluster Centroids";
		markerArray[i].id = i+1;
		markerArray[i].action = visualization_msgs::Marker::ADD;
		markerArray[i].type = visualization_msgs::Marker::SPHERE;
		markerArray[i].pose.position = _clusterDetections.cone_detections[i-1].position;
		markerArray[i].scale.x = 0.1;
		markerArray[i].scale.y = 0.1;
		markerArray[i].scale.z = 0.1;
		markerArray[i].color.a = 1.0;
		markerArray[i].color.r = 0.55;
		markerArray[i].color.g = 0.1;
		markerArray[i].color.b = 0.7;
	}
	_clusterCentroidMarkers.markers = markerArray;
}

/**
*	Name: visualizeConeCentroid.
*	Description: Used to visualize the validated cone's centroid using markers on Rviz.
*	Inputs: none
*	Output: void
*	Temporary. Might be deleted from here in a final verbose version.
*/
void ConeDetector::visualizeConeCentroid() {
	std::vector<visualization_msgs::Marker> markerArray(_clusterDetections.cone_detections.size()+1);
	markerArray[0].action = visualization_msgs::Marker::DELETEALL;
	for (size_t i = 1; i < _clusterDetections.cone_detections.size() + 1; ++i){
		markerArray[i].header.frame_id = _pointCloud.header.frame_id;
		markerArray[i].header.stamp = _pointCloud.header.stamp;
		markerArray[i].ns = "cone centroids";
		markerArray[i].id = i+1;
		markerArray[i].action = visualization_msgs::Marker::ADD;
		markerArray[i].type = visualization_msgs::Marker::SPHERE;
		markerArray[i].pose.position = _clusterDetections.cone_detections[i-1].position;
		markerArray[i].scale.x = 0.5;
		markerArray[i].scale.y = 0.5;
		markerArray[i].scale.z = 0.5;
		markerArray[i].color.a = 1.0;
		markerArray[i].color.r = 0.0;
		markerArray[i].color.g = 1.0;
		markerArray[i].color.b = 0.0;
	}
	_coneCentroidMarkers.markers = markerArray;
}

/********  End of General Tasks  ********/

/*************************************************
*             Point Cloud Processing             *
*************************************************/

/**
*	Name: runAlgorithm.
*	Description: It runs the LiDAR pipeline in order.
*	Inputs: string with the lidar being used (ouster/velodyne)
*	Output: void
*/
void ConeDetector::runAlgorithm() {
	convertFromRosToPcl();
	if(_pclPointCloud->size() > 0) {
		passThroughFiltering();
		removeGround();
		detectClusters();
		calculateClusterCentroids();
		visualizeClusterCentroid();
		reconstructCones();
		calculateClusterCentroids();
		validateCluster();
		visualizeConeCentroid();
	}
	convertFromPclToRos();
	transformClusters();
}

/**
*	Name: passTroughFiltering.
*	Description: Applies a passTrough filter on all axis. Used to remove points from behind the car, for example.
*	Inputs: _pclPointCloud - original pointcloud from the LiDAR.
*	Output: Filtered _pclPointCloud.
*/
void ConeDetector::passThroughFiltering() {
	_pclPointCloud = _passthroughFilter.filter(_pclPointCloud);

	//Convert PCL pointcloud to ROS pointcloud for visualization
	pcl::toROSMsg(*_pclPointCloud, _filteredPointCloud);
	_filteredPointCloud.header = _pointCloud.header;
}

/**
*	Name: removeGround.
*	Description: Uses RANSAC to detect and remove the ground.
*	Inputs: PassTrough filtered _pclPointCloud.
*	Output: _pclPointCloud without the ground and _pclPointCloudGround with only the ground.
*/
void ConeDetector::removeGround() {
	std::tie(_pclPointCloud, _pclPointCloudGround) = _ransacAlgorithm.removePlane(_pclPointCloud);

	//Convert PCL pointcloud to ROS pointcloud for visualization
	pcl::toROSMsg(*_pclPointCloud, _ransacPointCloud);
	_ransacPointCloud.header = _pointCloud.header;
}

/**
*	Name: detectClusters.
*	Description: Uses Euclidean Clustering method to separate the pointcloud into clusters.
*	Inputs: PassTrough filtered _pclPointCloud without the ground.
*	Output: _clusterIndices with the indices of the identified clusters.
*/
void ConeDetector::detectClusters() {
	_clusterIndices = _clusteringMethod.getClusterIndices(_pclPointCloud);
}

/**
*	Name: calculateClusterCentroids.
*	Description: Uses compute3DCentroid function from PCL to calculate the centroid of the detected clusters.
*	Inputs: _pclPointCloud and _clusterIndices.
*	Output: _clusterDetections with the centroid of each of the detected clusters.
*/
void ConeDetector::calculateClusterCentroids() {
	std::vector<Eigen::Vector4d> centroids(_clusterIndices.size());
	std::vector<common_msgs::Cone> clusterArray(_clusterIndices.size());

	for (size_t i = 0; i < _clusterIndices.size(); ++i) {
		pcl::compute3DCentroid(*_pclPointCloud, _clusterIndices[i], centroids[i]);
		clusterArray[i].position.x = centroids[i][0];
		clusterArray[i].position.y = centroids[i][1];
		clusterArray[i].position.z = centroids[i][2];
		clusterArray[i].color = UNKNOWN_CONE;
		clusterArray[i].header = _pointCloud.header;
  	}
	_clusterDetections.cone_detections = clusterArray;
	_clusterDetections.header = _pointCloud.header;
}

/**
*	Name: reconstructCones.
*	Description: Performs cone reconstruction using the pointcloud with the removed ground.
*	Inputs: _pclPointCloud, _pclPointCloudGround and _clusterDetections.
*	Output: _pclPointCloud with points that were removed by RANSAC and belong to cones.
*/
void ConeDetector::reconstructCones() {
	std::tie(_pclPointCloud, _clusterIndices) = _coneReconstruction.reconstruction(_pclPointCloud, _pclPointCloudGround, _clusterDetections, _clusterIndices);
}

/**
*	Name: validateCluster.
*	Description: Performs cluster validation in order to validate a cluster as a cone.
*	Inputs: Pointcloud with the cluster detections and its indices.
*	Output: _coneDetections with the validated clusters and it's indices.
*/
void ConeDetector::validateCluster() {
	_clusterDetections.cone_detections = _coneIdentification.validateClusters(_clusterIndices, _pclPointCloud, _clusterDetections);
}

void ConeDetector::transformClusters() {
	_transform = tf_buffer_.lookupTransform("cog",  _pointCloud.header.frame_id , ros::Time(0));

	for (size_t i = 0; i < _clusterDetections.cone_detections.size(); i++){
		_clusterDetections.cone_detections[i].position.x += _transform.transform.translation.x;
	}
	_clusterDetections.header.frame_id = "cog";
}

/********  End of Point Cloud Processing  ********/