#include "lidar/lidar_passthrough_filter.h"

// Constructor
PassthroughFilter::PassthroughFilter() {}

// Setters
void PassthroughFilter::setMinX(double minX) { _minX = minX; }
void PassthroughFilter::setMaxX(double maxX) { _maxX = maxX; }
void PassthroughFilter::setMinY(double minY) { _minY = minY; }
void PassthroughFilter::setMaxY(double maxY) { _maxY = maxY; }
void PassthroughFilter::setMinZ(double minZ) { _minZ = minZ; }
void PassthroughFilter::setMaxZ(double maxZ) { _maxZ = maxZ; }
void PassthroughFilter::setDistanceFilter(double distanceFilter) { _distanceFilter = distanceFilter; }

/**
*	Name: filterX.
*	Description: Applies a passTrough filter on the X-axis. Used to remove points from behind the car or point that are too far away.
*	Inputs: pointCloud - original pointcloud from the LiDAR.
*	Output: Filtered pointCloud on the X-axis.
*/
PclPointCloud::Ptr PassthroughFilter::filterX(PclPointCloud::Ptr pointCloud) {
	_pass.setInputCloud(pointCloud);
	_pass.setFilterFieldName("x");
	_pass.setFilterLimits(_minX, _maxX);
	_pass.filter(*pointCloud);
	return pointCloud;
}

/**
*	Name: filterY.
*	Description: Applies a passTrough filter on the Y-axis. Used to remove points on the car sides
*	Inputs: pointCloud filtered on the X-axis.
*	Output: Filtered pointCloud on the XY-axis.
*/
PclPointCloud::Ptr PassthroughFilter::filterY(PclPointCloud::Ptr pointCloud) {
	_pass.setInputCloud(pointCloud);
	_pass.setFilterFieldName("y");
	_pass.setFilterLimits(_minY, _maxY);
	_pass.filter(*pointCloud);
	return pointCloud;
}

/**
*	Name: filterZ.
*	Description: Applies a passTrough filter on the Z-axis. Used to remove points that are too high.
*	Inputs: pointCloud filtered on the XY-axis.
*	Output: Filtered pointCloud on the XYZ-axis.
*/
PclPointCloud::Ptr PassthroughFilter::filterZ(PclPointCloud::Ptr pointCloud) {
	_pass.setInputCloud(pointCloud);
	_pass.setFilterFieldName("z");
	_pass.setFilterLimits(_minZ, _maxZ);
	_pass.filter(*pointCloud);
	return pointCloud;
}

/**
*	Name: filter.
*	Description: Applies a passTrough filter on all axis.
*	Inputs: pointCloud - original pointcloud from the LiDAR.
*	Output: Filtered pointCloud on the XYZ-axis.
*/
PclPointCloud::Ptr PassthroughFilter::filter(PclPointCloud::Ptr pointCloud) {
	PclPointCloud::Ptr filteredPointCloud;
	filteredPointCloud =  filterZ(filterY(filterX(pointCloud)));
	for (PclPointCloud::iterator it = filteredPointCloud->begin(); it != filteredPointCloud->end(); ) {
		if(std::hypot(it->x,it->y) < _distanceFilter){
			it = filteredPointCloud->erase(it);
		} else {
			++it;
		}
	}
	return filteredPointCloud;
}
