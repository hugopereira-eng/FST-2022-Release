#include "lidar/lidar_cone_reconstruction.h"

ConeReconstruction::ConeReconstruction() {}

//Setters
void ConeReconstruction::setCylinderRadius(double cylinderRadius) { _cylinderRadius = cylinderRadius; }
void ConeReconstruction::setCylinderHeight(double cylinderHeight) { _cylinderHeight = cylinderHeight; }

/**
*	Name: reconstruction.
*	Description: Performs cone reconstruction by fitting an imaginary cylinder around each cluster and evaluating which points from the ground pointcloud fall inside.
*	Inputs: _pclPointCloud, _pclPointCloudGround and _clusterDetections.
*	Output: _pclPointCloud with points that were removed by RANSAC and belong to cones.
*/
std::tuple<PclPointCloud::Ptr, std::vector<pcl::PointIndices>> ConeReconstruction::reconstruction(PclPointCloud::Ptr pclPointCloud, PclPointCloud::Ptr pclPointCloudGround, common_msgs::ConeDetections clusterDetections, std::vector<pcl::PointIndices> clusterIndices) {
	
	std::vector<int> reconstructedPoints (clusterDetections.cone_detections.size());

	// For each point of the ground point cloud check if it's inside a cylinder around the centroid of each cluster
	for(PclPointCloud::iterator it = pclPointCloudGround->begin(); it!= pclPointCloudGround->end(); it++) {

		for (size_t i = 0; i < clusterDetections.cone_detections.size(); ++i) {

			Point center(clusterDetections.cone_detections[i].position.x,clusterDetections.cone_detections[i].position.y,clusterDetections.cone_detections[i].position.z); // cluster centroid
			Point bottomCenter(center(0),center(1),center(2)-_cylinderHeight); // center of the botom face of the cylinder
			Point topCenter(center(0),center(1),center(2)+_cylinderHeight); // center of the top face of the cylinder
			Point groundPoint(it->x,it->y,it->z); // ground point to be evaluated
			Point vec((topCenter-bottomCenter));
			double cte = _cylinderRadius * (topCenter-bottomCenter).norm();
			PclPoint point;
			point.x = it->x;
			point.y = it->y;
			point.z = it->z;
			point.intensity = it->intensity;

			// Checking if the point is inside a cylinder around the cluster by evaluating if the point is higher than the bottom cylinder face, lower than the top cylinder face and inside of cte "circumference"
			if( ( (groundPoint-bottomCenter).dot(vec) >= 0 ) && ( (groundPoint-topCenter).dot(vec) <= 0 ) && ( ((groundPoint-bottomCenter).cross(vec)).norm() <= cte ) ){
				pclPointCloud->points.push_back(point);
				pclPointCloud->width = (std::uint32_t) pclPointCloud->points.size();
				pclPointCloud->height = 1;
				clusterIndices[i].indices.push_back(pclPointCloud->points.size()-1);
				reconstructedPoints[i]++;
			}
		}
	}
	return std::make_tuple(pclPointCloud, clusterIndices);
}