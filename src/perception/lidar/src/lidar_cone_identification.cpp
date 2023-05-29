#include "lidar/lidar_cone_identification.h"

ConeIdentification::ConeIdentification() {}

//Setters
void ConeIdentification::setVerticalResolution(double verticalResolution) { _verticalResolution = verticalResolution; }
void ConeIdentification::setCylinderRadius(double cylinderRadius) { _cylinderRadius = cylinderRadius; }
void ConeIdentification::setCylinderHeight(double cylinderHeight) { _cylinderHeight = cylinderHeight; }

/**
*	Name: validateClusters.
*	Description: Performs cluster validation by fitting a cylinder (with the dimensions of a cone) around each centroid and evaluating if all of points from the cluster fall inside this cylinder. 
* 				 For these clusters, check if its centroid is lower than the middle of the cluster (as it should have more points at bottom because a cone is larger at bottom) and check if the distance
*				 between the top and bottom points of the cluster is larger than the distance between two LiDAR layers.
*	Inputs: Pointcloud with the cluster detections and its indices.
*	Output: coneArray with the validated clusters and validatedClusters with the corresponding indices.
*/
std::vector<common_msgs::Cone> ConeIdentification::validateClusters(std::vector<pcl::PointIndices> clusterIndices, PclPointCloud::Ptr pclPointCloud, common_msgs::ConeDetections clusterDetections) {

  int i = 0;
  double d;
  bool out;
  PclPoint minPt, maxPt;
  std::vector<PclPoint> minPtVec;
  std::vector<PclPoint> maxPtVec;
  std::vector<common_msgs::Cone> coneArray;
  std::vector<pcl::PointIndices> validatedClusters;
  
	//loop over all clusters
	for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
		
        PclPointCloud::Ptr cloud_cluster (new PclPointCloud);
        geometry_msgs::Point clusterPos = clusterDetections.cone_detections[i].position;
        Point centroid(clusterPos.x,clusterPos.y,clusterPos.z);
        Point p1(centroid(0),centroid(1),centroid(2)-_cylinderHeight); //lower bound
        Point p2(centroid(0),centroid(1),centroid(2)+_cylinderHeight); //upper bound
        Point vec((p2-p1));
        double cte = _cylinderRadius * (p2-p1).norm();

		//loop over all points in the cluster
		// Fitting a cylinder (with the dimensions of a cone) around each centroid and evaluating if all of points from the cluster fall inside this cylinder.
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
			PclPoint &point = pclPointCloud->points[*pit];
			Point q(point.x,point.y,point.z);
			 	
            if( ((q-p1).dot(vec) >= 0) && ((q-p2).dot(vec) <= 0) && (((q-p1).cross(vec)).norm() <= cte )) {
                cloud_cluster->points.push_back (pclPointCloud->points[*pit]);
            } else {
                out = 1;
                break;
            }
        }
      
        if (out == 0) {
            common_msgs::Cone cone;
            cone.position.x = clusterDetections.cone_detections[i].position.x;
            cone.position.y = clusterDetections.cone_detections[i].position.y;
            cone.position.z = clusterDetections.cone_detections[i].position.z;
            cone.color = UNKNOWN_CONE;
            pcl_msgs::PointIndices clusterIndices;
        	clusterIndices.indices = it->indices;
            cone.point_indices = clusterIndices;
            coneArray.push_back(cone);
        }
        out = 0;
        i++;
    }
	return coneArray; 	
}