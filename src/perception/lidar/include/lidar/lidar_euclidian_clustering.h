#ifndef LIDAR_EUCLIDIAN_CLUSTERING_HPP
#define LIDAR_EUCLIDIAN_CLUSTERING_HPP

#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointXYZI PclPoint;
typedef pcl::PointCloud<PclPoint> PclPointCloud;
class EuclidianClustering {

public:
	// Constructor
	EuclidianClustering();
	// Setters
	void setTolerance(double tolerance);
	void setClusterMinSize(int _clusterMinSize);
	void setCusterMaxSize(int _clusterMaxSize);
	// Methods
	std::vector<pcl::PointIndices> const & getClusterIndices(PclPointCloud::Ptr pointCloud);

private:
	double _tolerance;
	int _clusterMinSize;
	int _clusterMaxSize;
	std::vector<pcl::PointIndices> _clusterIndices; 
};

#endif 
