#ifndef LIDAR_CONE_RECONSTRUCTION_HPP
#define LIDAR_CONE_RECONSTRUCTION_HPP

#include <pcl_ros/point_cloud.h>
#include "common_msgs/ConeDetections.h"

typedef pcl::PointXYZI PclPoint;
typedef pcl::PointCloud<PclPoint> PclPointCloud;
typedef Eigen::Vector3d Point;

class ConeReconstruction {
public:
	// Constructor
	ConeReconstruction();
	// Setters
  	void setCylinderRadius(double cylinderRadius);
  	void setCylinderHeight(double cylinderHeight);
	// Methods
	std::tuple<PclPointCloud::Ptr, std::vector<pcl::PointIndices>> reconstruction(PclPointCloud::Ptr pclPointCloud, PclPointCloud::Ptr pclPointCloudGround, common_msgs::ConeDetections clusterDetections, std::vector<pcl::PointIndices> clusterIndices);

private:
	double _cylinderRadius;
	double _cylinderHeight;
};

#endif 