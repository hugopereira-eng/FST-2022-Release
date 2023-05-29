#ifndef LIDAR_RANSAC_HPP
#define LIDAR_RANSAC_HPP

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZI PclPoint;
typedef pcl::PointCloud<PclPoint> PclPointCloud;
class Ransac {

public:
	// Constructor
	Ransac();
	// Setters
	void setDistanceThreshold(double distanceThreshold);
	void setMaxIteration(int maxIterations);
	// Methods
	std::tuple<PclPointCloud::Ptr, PclPointCloud::Ptr> removePlane(PclPointCloud::Ptr pointCloud);

private:
	double _distanceThreshold;
	int _maxIterations;
};

#endif 
