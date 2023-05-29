#include "lidar/lidar_euclidian_clustering.h"

// Constructor
EuclidianClustering::EuclidianClustering() {}

// Setters
void EuclidianClustering::setTolerance(double tolerance) { _tolerance = tolerance; }
void EuclidianClustering::setClusterMinSize(int clusterMinSize) { _clusterMinSize = clusterMinSize; }
void EuclidianClustering::setCusterMaxSize(int clusterMaxSize) {  _clusterMaxSize = clusterMaxSize; }

/**
*	Name: getClusterIndices.
*	Description: Uses Euclidean Clustering method to separate the pointcloud into clusters.
*	Inputs: PassTrough filtered pointCloud without the ground.
*	Output: _clusterIndices with the indices of the identified clusters.
*/
std::vector<pcl::PointIndices> const & EuclidianClustering::getClusterIndices(PclPointCloud::Ptr pointCloud) {
	_clusterIndices.clear();
	pcl::search::KdTree<PclPoint>::Ptr tree(new pcl::search::KdTree<PclPoint>);
	tree->setInputCloud(pointCloud);

	pcl::EuclideanClusterExtraction<PclPoint> ec;
	ec.setClusterTolerance(_tolerance);
	ec.setMinClusterSize(_clusterMinSize);
	ec.setMaxClusterSize(_clusterMaxSize);
	ec.setSearchMethod(tree);
	ec.setInputCloud(pointCloud);
	ec.extract(_clusterIndices);

	return _clusterIndices;
}
