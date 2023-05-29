#ifndef LIDAR_CONE_IDENTIFICATION_HPP
#define LIDAR_CONE_IDENTIFICATION_HPP

#include <as_lib/common.h>
#include "common_msgs/ConeDetections.h"
#include <pcl/common/common.h>
#include <math.h>

typedef pcl::PointXYZI PclPoint;
typedef pcl::PointCloud<PclPoint> PclPointCloud;
typedef Eigen::Vector3d Point;

class ConeIdentification {
public:
  	// Constructor
	ConeIdentification();
	// Setters
	void setVerticalResolution(double verticalResolution);
	void setCylinderRadius(double cylinderRadius);
	void setCylinderHeight(double cylinderHeight);
	
	std::vector<common_msgs::Cone> validateClusters(std::vector<pcl::PointIndices> clusterIndices, PclPointCloud::Ptr pclPointCloud, common_msgs::ConeDetections clusterDetections);

private:
	double _verticalResolution;
	double _cylinderRadius;
	double _cylinderHeight;
};
#endif