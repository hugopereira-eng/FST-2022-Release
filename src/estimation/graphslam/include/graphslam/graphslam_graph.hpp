#ifndef GRAPHSLAM_GRAPH_HPP
#define GRAPHSLAM_GRAPH_HPP


#include "graphslam/graphslam_types.hpp"
#include "graphslam/graphslam_motion_model.hpp"
#include <eigen3/Eigen/Eigen>
#include <fstream>
#include <random>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/slam2d/se2.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

class Graph {
    public:
    // Constructor
    Graph();

    //Getters
    visualization_msgs::MarkerArray const & getVertexesMarkers() const;
    visualization_msgs::Marker const & getEdgeMarkers() const;

    // Atributes
    g2o::SparseOptimizer _graph;
    g2o::HyperGraph::VertexSet _vertexSet;
    g2o::HyperGraph::EdgeSet _edgeSet;

    // Methods
    void addOdometryVertex(const Pose& pose, const size_t graphId);
    void addOdometryEdge(const int odomPrev, const int odomNext, Eigen::Matrix3d controlNoise);
    void addLandmarkVertex( const Landmark& lm, const size_t graphId);
    void addLandmarkEdge(const size_t vertexOdomId, const size_t vertexLmId, Observation& obs, Eigen::Matrix2d obsNoise);
    void updateLandmarkVertex(const Landmark& lm);
    void optimizeGraph(size_t numIter, size_t level);
    void deleteVertexfromGraph(size_t vertexId);
    void deleteEdgesfromGraph(size_t vertexId);
    Eigen::Vector2d getLandmarkLocationOptimized(const size_t lmId);
    Pose getOptimizedPose(const size_t odomId);
    void fixGraph();
    void unfixGraph();
    void saveGraph(std::string name);
    
    // Visualization
    void visualizeGraphVertexes();
    void visualizeGraphEdges();

      
    
    private: 
    
    // Atributes
    int _saveGraph = 0;
    
    // Visualization
    visualization_msgs::MarkerArray _vertexes;
    visualization_msgs::Marker _edges;  

    // Methods
    Eigen::Vector2d getLandmarkLocation(const Observation& obs, const std::vector<double>& pose);

};
#endif