#include <ros/ros.h>
#include "path_planner/path_planner_pipeline.hpp"
#include "geometry_msgs/PolygonStamped.h"

// Constructor
PathPlanner::PathPlanner() {
	ROS_INFO("Constructing PathPlanner");
	if (!ros::param::get("/common/mission_selected", _mission)) {

		ROS_WARN_STREAM("Could not load _mission. Default mission is trackdrive");
		_mission = TRACKDRIVE;
	}
	
	initParameters();
};

void PathPlanner::initParameters() {

	if(!ros::param::has("/common/camera/intrinsic_matrix") &&
		  !ros::param::has("/common/camera/distortion_coefficients") &&
		  !ros::param::has("/common/lidar/ext_matrix_rot") &&
		  !ros::param::has("/common/lidar/ext_matrix_tra")) {
		ROS_ERROR("This node needs the sensor matrices for visualization. Please load them.");
		exit(EXIT_FAILURE);	
	}

    /* load matrices values from rosparam server */
    std::vector<double> intMatVector;
    std::vector<double> extRotMatVector;
    std::vector<double> extTraMatVector;
    ros::param::get("/common/camera/intrinsic_matrix", intMatVector);
    ros::param::get("/common/lidar/ext_matrix_rot", extRotMatVector);
    ros::param::get("/common/lidar/ext_matrix_tra", extTraMatVector);

    /* load intrinsic matrix */
    Eigen::Matrix<double,3,3> intMat(intMatVector.data());
    intMat.transposeInPlace();

    /* load extrinsic matrix (lidar to camera transform) */
    Eigen::Matrix<double,3,3> extRotMat(extRotMatVector.data());
    Eigen::Matrix<double,3,1> extTraMat(extTraMatVector.data());
    Eigen::Matrix<double,3,4> extMat;
    extRotMat.transposeInPlace();
    extMat << extRotMat, extTraMat;
    
    _pMat = intMat * extMat;
}


// Getters
visualization_msgs::MarkerArray const & PathPlanner::getBestTrajectory() const { return _bestTrajectoryMarkers; }
visualization_msgs::Marker const & PathPlanner::getDelaunayMarkerLines() const { return _delaunayMarkerLines; }
visualization_msgs::Marker const & PathPlanner::getMidpointConnectionsLines() const { return _midpointConnectionsLines; }
nav_msgs::Path const & PathPlanner::getPath() const { return _pathToFollow; }

common_msgs::Cost const & PathPlanner::getFirstCost() const { return _pathTree.getBestCost(); }
common_msgs::Cost const & PathPlanner::getSecondCost() const { return _pathTree.getSecondBestCost(); }
common_msgs::Cost const & PathPlanner::getThirdCost() const { return _pathTree.getThirdBestCost(); }


void PathPlanner::updatePipelineParameters(const path_planner::PathPlannerConfig &config) {
	_visualize = config.visualization;

	_pathTree.setKwidth(config.cost_function_Kwidth);
	_pathTree.setKdistance(config.cost_function_Kdistance);
	_pathTree.setKangle(config.cost_function_Kangle);
	_pathTree.setKdistance_total(config.cost_function_Kdistance_total);
	_pathTree.setconeColor(config.cost_function_coneColor);
	_pathTree.setconeNoColor(config.cost_function_coneNoColor);
	_pathTree.setwrongSide(config.cost_function_wrongSide);

	_pathTree.setWidthAdmMax(config.cost_adm_max_width);
	_pathTree.setWidthAdmMin(config.cost_adm_min_width);
	_pathTree.setDistanceAdmMax(config.cost_adm_max_distance);
	_pathTree.setDistanceAdmMin(config.cost_adm_min_distance);
	_pathTree.setAngleAdm(config.cost_adm_angle);
	_pathTree.setDistanceTotalAdmMax(config.cost_adm_max_distanceTotal);
	_pathTree.setDistanceTotalAdmMin(config.cost_adm_min_distanceTotal);

}

/* Update to be executed on the callback of the sensor_fusion node handle. Updates the detections in order to
 * compute the new path to follow.
 */
void PathPlanner::setConeDetectionsSlam(const common_msgs::ConeDetections &newConeDetections) {
	// _coneDetections = newConeDetections;
}

void PathPlanner::setConeDetectionsSensorFusion(const common_msgs::ConeDetections &newConeDetections) {
	_coneDetections = newConeDetections;
}

void PathPlanner::updateCameraImage(const sensor_msgs::Image &newCameraImage) {
	_cameraImage = newCameraImage;
}

void PathPlanner::setPathPlannerActive(const std_msgs::Bool &pathPlannerActive) {
	_pathPlannerActive = pathPlannerActive;
}

void PathPlanner::runAlgorithm() {
	findCenterLine();
	if (_visualize) visualizeCenterlineOnCamera();
}

void PathPlanner::findCenterLine() {

	common_msgs::ConeDetections yellowCones;
	common_msgs::ConeDetections blueCones;
	nav_msgs::Path center_line;

	_delaunayMarkerLines.points.clear();
	_midpointConnectionsLines.points.clear();
	_bestTrajectoryMarkers.markers.clear();
	
	// upon speedPlanner inclusion the commented condition should be included
	if (_coneDetections.cone_detections.size() >= 3 /*&& _pathPlannerActive.data == true*/) {
		_pathToFollow.poses.clear();
		delaunay();
	}
	else _pathToFollow = _pathTree.getBestPath();

}

void PathPlanner::delaunay() {

	std::vector<std::pair<Point, ColorData>> conesInfo;
	ColorData infodata;
	infodata.color = -1;

	for (const auto &cone: _coneDetections.cone_detections) {
		if (cone.color != ORANGE_CONE && cone.color != BIG_ORANGE_CONE) {
			if (cone.color == BLUE_CONE) {
				infodata.color = 1;
			}
			else if (cone.color == YELLOW_CONE) {
				infodata.color = 2;
			}
			else {
				// if (hypot(cone.position.x, cone.position.y) < 2.5)
				// 	continue;
				infodata.color = 0;
			}
			conesInfo.push_back(std::make_pair(Point(cone.position.x, cone.position.y), infodata));
		}
	}

	Delaunay dt(conesInfo.begin(),conesInfo.end());

	if (conesInfo.size() < 3 ){
		_pathToFollow=_pathTree.getBestPath();
		return;
	}
	else _pathToFollow.poses.clear();

	_pathTree.generateTree(dt);
	_pathTree.findTrajectory();
	_pathTree.vizualization();

	_delaunayMarkerLines = _pathTree.getDelaunayMarkerLines();
	_midpointConnectionsLines = _pathTree.getMidpointConnectionsLines();
	_bestTrajectoryMarkers = _pathTree.getBestTrajectoryMarkers();
	_delaunayMarkerLines.header = _coneDetections.header;
	_midpointConnectionsLines.header = _coneDetections.header;
	for (auto & marker : _bestTrajectoryMarkers.markers)
		marker.header = _coneDetections.header;

	_pathToFollow = _pathTree.getBestPath();
	_pathToFollow.header = _pathTree.getBestPath().header;
	_pathTree.deleteAllTree();
	_pathToFollow.header = _coneDetections.header;
	
}

void PathPlanner::visualizeCenterlineOnCamera() {

	// convert path poses
	std::vector<Eigen::Vector2d> convertedPoints = PathPlanner::coordinatesConversion();

	// get cv image from image msg
	cv_bridge::CvImagePtr cv_ptr;
    try {
    	cv_ptr = cv_bridge::toCvCopy(_cameraImage, sensor_msgs::image_encodings::BGR8);
    }
	catch (cv_bridge::Exception& e) {
    	ROS_ERROR("cv_bridge exception: %s", e.what());
    	return;
    }

	// draw centerline
	for (size_t i = 1; i < convertedPoints.size(); ++i) {
		cv::circle(cv_ptr->image, cv::Point(convertedPoints[i][0], convertedPoints[i][1]), 10, CV_RGB(0,255,0), cv::FILLED);
		if (i == convertedPoints.size()-1) break;
		cv::line(cv_ptr->image, cv::Point(convertedPoints[i][0], convertedPoints[i][1]), cv::Point(convertedPoints[i+1][0], convertedPoints[i+1][1]), CV_RGB(0,0,255), 3);
	}

	// convert cv image to image msg
	cv_ptr->header.stamp = _pathToFollow.header.stamp;
	cv_ptr->header.frame_id = "centerline_visualization";
	cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
	_centerlineVisualization = *cv_ptr->toImageMsg();
}

std::vector<Eigen::Vector2d> PathPlanner::coordinatesConversion() {

	Eigen::Matrix<double,3,1> fusionMat;
	Eigen::Matrix<double,4,1> lidarPointsMat;
	std::vector<Eigen::Vector2d> convertedPoints(_pathToFollow.poses.size());
	
	for (size_t i = 0; i < _pathToFollow.poses.size(); ++i){
		float xLidar = _pathToFollow.poses[i].pose.position.x;
		float yLidar = _pathToFollow.poses[i].pose.position.y;
		float zLidar = _pathToFollow.poses[i].pose.position.z;
		lidarPointsMat << xLidar, yLidar, zLidar, 1;
		fusionMat = _pMat * lidarPointsMat;
		float u = fusionMat[0]/fusionMat[2];
		float v = fusionMat[1]/fusionMat[2];
		convertedPoints[i][0] = u;
		convertedPoints[i][1] = v;
	}
	return convertedPoints;
}