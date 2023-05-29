#ifndef PATH_PLANNER_PIPELINE_HPP
#define PATH_PLANNER_PIPELINE_HPP

#include "shared/as_lib/common.h"
#include "common_msgs/ConeDetections.h"
#include "nav_msgs/Path.h"
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_reconfigure/server.h>
#include <path_planner/PathPlannerConfig.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include "path_planner/path_planner_path_tree.hpp"
#include <fstream>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

typedef Delaunay::Finite_edges_iterator Edge_iterator;
typedef Delaunay::Vertex_handle Vertex_handle;
typedef Delaunay::Edge_circulator Edge_circulator;
typedef Delaunay::Edge Edge;
typedef K::Point_2 Point;

class PathPlanner {
public:
  // Constructor:
  PathPlanner();

  // Getters:
  nav_msgs::Path const & getPath() const;
  visualization_msgs::MarkerArray const & getBestTrajectory() const;
  visualization_msgs::Marker const & getDelaunayMarkerLines() const;
  visualization_msgs::Marker const & getMidpointConnectionsLines() const;
  common_msgs::Cost const & getCost() const;
  common_msgs::Cost const & getFirstCost() const;
  common_msgs::Cost const & getSecondCost() const;
  common_msgs::Cost const & getThirdCost() const;

  void updatePipelineParameters(const path_planner::PathPlannerConfig &config);

  // Changes path information to the new value received:
  void setConeDetectionsSlam(const common_msgs::ConeDetections &newConeDetections);
  void setConeDetectionsSensorFusion(const common_msgs::ConeDetections &newConeDetections);
  void setPathPlannerActive(const std_msgs::Bool &pathPlannerActive);
  void updateCameraImage(const sensor_msgs::Image &newCameraImage);
  void runAlgorithm();
  
private:
  common_msgs::ConeDetections _coneDetections;
  nav_msgs::Path _pathToFollow;
  std_msgs::Bool _pathPlannerActive;
  visualization_msgs::MarkerArray _bestTrajectoryMarkers;
  visualization_msgs::Marker _delaunayMarkerLines;
  visualization_msgs::Marker _midpointConnectionsLines;
  sensor_msgs::Image _cameraImage;
  sensor_msgs::Image _centerlineVisualization;
  
  int _mission;
  bool _visualize;

  // for dinamic matrixes input
  Eigen::Matrix<double,3,4> _pMat;    // extMat * intMat
  PathPlannerTree _pathTree;

  void initParameters();
  void findCenterLine();
  void delaunay();
  void visualizeCenterlineOnCamera();
  std::vector<Eigen::Vector2d> coordinatesConversion();
};

#endif 
