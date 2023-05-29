#include "lidar/lidar_ransac.h"

//Constructor
Ransac::Ransac() {}

//Setters
void Ransac::setDistanceThreshold(double distanceThreshold) { _distanceThreshold = distanceThreshold; }
void Ransac::setMaxIteration(int maxIterations) { _maxIterations = maxIterations; }

/**
*	Name: removePlane.
*	Description: Uses RANSAC with the plane model to detect and remove the ground.
*	Inputs: PassTrough filtered _pclPointCloud.
*	Output: pointCloud without the ground and pointCloudGround with only the ground.
*/
std::tuple<PclPointCloud::Ptr, PclPointCloud::Ptr> Ransac::removePlane(PclPointCloud::Ptr pointCloud) {
  
    PclPointCloud::Ptr pointCloudGround(new PclPointCloud);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PclPoint> seg;
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE); // Model to be used by the RANSAC
    seg.setMethodType(pcl::SAC_RANSAC); // Choosing RANSAC as the method
    seg.setDistanceThreshold(_distanceThreshold);
    seg.setMaxIterations(_maxIterations);
    seg.setInputCloud(pointCloud);
    seg.segment(*inliers, *coefficients);

	if (inliers->indices.size () == 0) {
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return std::make_tuple(pointCloud,pointCloudGround); //PointCloud unchanged.
    }
	
    pcl::ExtractIndices<PclPoint> extract;
    
    extract.setInputCloud(pointCloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*pointCloudGround); // saving the ground points in pointCloudGround
    extract.setNegative(true);
    extract.filter(*pointCloud); // saving the pointcloud without ground in pointCloud

    return std::make_tuple(pointCloud,pointCloudGround);
}