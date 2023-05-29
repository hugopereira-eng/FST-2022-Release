#ifndef LIDAR_PASSTHROUGH_FILTER_HPP
#define LIDAR_PASSTHROUGH_FILTER_HPP

#include "pcl/filters/passthrough.h"
#include <pcl_ros/point_cloud.h>

typedef pcl::PointXYZI PclPoint;
typedef pcl::PointCloud<PclPoint> PclPointCloud;
class PassthroughFilter {
	
public:
	// Constructor
	PassthroughFilter();
	// Methods
	PclPointCloud::Ptr filter(PclPointCloud::Ptr pointCloud);
	// Setters
	void setMinX(double minX);
	void setMaxX(double maxX);
	void setMinY(double minY);
	void setMaxY(double maxY);
	void setMinZ(double minZ);
	void setMaxZ(double maxZ);
	void setDistanceFilter(double distanceFilter);


private:
	PclPointCloud::Ptr filterX(PclPointCloud::Ptr pointCloud);
	PclPointCloud::Ptr filterY(PclPointCloud::Ptr pointCloud);
	PclPointCloud::Ptr filterZ(PclPointCloud::Ptr pointCloud);

	pcl::PassThrough<PclPoint> _pass;
	double _minX;
	double _maxX;
	double _minY;
	double _maxY;
	double _minZ;
	double _maxZ;
	double _distanceFilter;
};

#endif 
