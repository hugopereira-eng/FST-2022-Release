#include <graphslam/graphslam_graph.hpp>

using namespace Eigen;

//Constructor
Graph::Graph() {
    
    typedef g2o::BlockSolver< g2o::BlockSolverTraits< -1, -1> >  SlamBlockSolver;
    typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseLandmarkMatrixType> SlamLinearSolver;
    
    std::unique_ptr<SlamLinearSolver> linearSolver(new SlamLinearSolver());
    linearSolver->setBlockOrdering(false);
    std::unique_ptr<SlamBlockSolver> blockSolver(new SlamBlockSolver(std::move(linearSolver)));
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(std::move(blockSolver));
    _graph.setAlgorithm(solver);
}

// Getters
visualization_msgs::MarkerArray const & Graph::getVertexesMarkers() const {return _vertexes;}
visualization_msgs::Marker const & Graph::getEdgeMarkers() const {return _edges;}

void Graph::addOdometryVertex(const Pose& pose, const size_t graphId) {
    
    Vector3d pose_(pose.pos.x(), pose.pos.y(), pose.theta);
    g2o::VertexSE2* vertex = new g2o::VertexSE2;
    
    vertex->setId(graphId);
    vertex->setEstimate(pose_);
    _graph.addVertex(vertex);

    if (!graphId) vertex->setFixed(true);
    
    _vertexSet.insert(vertex);
}

void Graph::addOdometryEdge(const int odomPrev, const int odomNext, Matrix3d controlNoise) {

    std::vector<double> posePrev, poseNext;
    g2o::EdgeSE2* edge = new g2o::EdgeSE2;

    _graph.vertex(odomPrev)->getEstimateData(posePrev);
    _graph.vertex(odomNext)->getEstimateData(poseNext);

    g2o::SE2 parentVertex(posePrev[0], posePrev[1], posePrev[2]);
    g2o::SE2 childVertex(poseNext[0], poseNext[1], poseNext[2]);

    edge->vertices()[0] = _graph.vertex(odomPrev);
    edge->vertices()[1] = _graph.vertex(odomNext);
    edge->setMeasurement(parentVertex.inverse() * childVertex);
    edge->setInformation(controlNoise.inverse());
    // edge->setLevel(1);
    _graph.addEdge(edge);
    _edgeSet.insert(edge);
}

void Graph::addLandmarkVertex(const Landmark& landmark, const size_t graphIndex) {

    g2o::VertexPointXY* vertex = new g2o::VertexPointXY;
    Vector2d lmPose = Vector2d(landmark.mean.x(), landmark.mean.y());

    vertex->setId(graphIndex);   
    vertex->setEstimate(lmPose);

    vertex->setFixed(true);

    _graph.addVertex(vertex);
    _vertexSet.insert(vertex);
}

void Graph::addLandmarkEdge(const size_t vertexOdomId, const size_t vertexLmId, Observation& obs, Eigen::Matrix2d obsNoise) {

    g2o::EdgeSE2PointXY* edge = new g2o::EdgeSE2PointXY;
    edge->vertices()[0] = _graph.vertex(vertexOdomId);
    edge->vertices()[1] = _graph.vertex(vertexLmId);
    
    std::vector<double> odomPose, lmPose;
    _graph.vertex(vertexOdomId)->getEstimateData(odomPose);
    _graph.vertex(vertexLmId)->getEstimateData(lmPose);
    g2o::SE2 pose (odomPose[0], odomPose[1], odomPose[2]);
    Vector2d lm = getLandmarkLocation(obs, odomPose);
    
    edge->setMeasurement(pose.inverse() * lm);
    Matrix2d noise = Matrix2d::Identity() * 0.5;
    edge->setInformation(noise.inverse());
    // edge->setLevel(0);
    _graph.addEdge(edge);
    _edgeSet.insert(edge);
}

void Graph::updateLandmarkVertex(const Landmark& lm) {
    
    std::vector<double> lmPose(2);
    lmPose[0] = lm.mean.x();
    lmPose[1] = lm.mean.y();
    _graph.vertex(lm.index)->setEstimateData(lmPose);
}

void Graph::optimizeGraph(size_t numIter, size_t level) {
    _graph.initializeOptimization();
    _graph.optimize(numIter);

    // _firstOptimization = false;
    // _vertexSet.clear();
    // _edgeSet.clear();    
    // _saveGraph++;
}

void Graph::saveGraph(std::string name){
    _graph.save(name.c_str());
}

inline Vector2d Graph::getLandmarkLocation(const Observation& obs, const std::vector<double>& pose) {
    return Vector2d(pose[0] + obs.measure.d * cosf(obs.measure.theta + pose[2]),
                    pose[1] + obs.measure.d * sinf(obs.measure.theta + pose[2]));
}

Vector2d Graph::getLandmarkLocationOptimized(const size_t lmId) {
    std::vector<double> lm;

    _graph.vertex(lmId)->getEstimateData(lm);     
    
    return Vector2d(lm[0], lm[1]);;
}

Pose Graph::getOptimizedPose(const size_t odomId) {
    std::vector<double> pose;
    
    _graph.vertex(odomId)->getEstimateData(pose);
     
    return Pose(pose[0], pose[1], pose[2]);
}

void Graph::deleteVertexfromGraph(size_t vertexIndex) {
    
    g2o::OptimizableGraph::Vertex* vertex = _graph.vertex(vertexIndex);
    _vertexSet.erase(_vertexSet.find(vertex));    
    _graph.removeVertex(vertex);    
}

void Graph::deleteEdgesfromGraph(size_t vertexIndex) {
    g2o::HyperGraph::EdgeSet edges = _graph.vertex(vertexIndex)->edges();
    
    for (g2o::HyperGraph::Edge* edge : edges) {
        _edgeSet.erase(_edgeSet.find(edge));                    
        _graph.removeEdge(edge);
    } 
}

void Graph::fixGraph() {
    for (auto it = _vertexSet.begin(); it != _vertexSet.end(); ++it) {
        g2o::HyperGraph::Vertex* vertex = *it; 
        _graph.vertex(vertex->id())->setFixed(true);
    }
}

void Graph::unfixGraph(){
    for (auto it = _vertexSet.begin(); it != _vertexSet.end(); ++it) {
        g2o::HyperGraph::Vertex* vertex = *it; 
        _graph.vertex(vertex->id())->setFixed(false);
    }

    g2o::HyperGraph::Vertex* vertex = *(_vertexSet.begin());
    _graph.vertex(vertex->id())->setFixed(true);
}

/************************************/
/******** Graph Visualization********/
/************************************/

void Graph::visualizeGraphVertexes() {
    
    _vertexes.markers.clear();

    visualization_msgs::Marker vertexPoint;
    vertexPoint.header.frame_id ="map";
    vertexPoint.action = visualization_msgs::Marker::ADD;
    vertexPoint.type = visualization_msgs::Marker::CUBE;
    vertexPoint.scale.x = 0.2;
    vertexPoint.scale.y = 0.2;
    vertexPoint.scale.z = 0.2;
    vertexPoint.color.a = 1.0;
    vertexPoint.lifetime = ros::Duration(0);
    vertexPoint.header.stamp = ros::Time::now();

    for (auto it = _vertexSet.begin(); it != _vertexSet.end(); ++it) {
       
       g2o::HyperGraph::Vertex* vertex = *it;
       vertexPoint.id = vertex->id();
       std::vector<double> pose;

        _graph.vertex(vertex->id())->getEstimateData(pose);

        vertexPoint.pose.position.x = pose[0];
        vertexPoint.pose.position.y = pose[1];
        vertexPoint.pose.position.z = 0;
        vertexPoint.pose.orientation = tf::createQuaternionMsgFromYaw(pose[2]);

        _vertexes.markers.push_back(vertexPoint);
    }
}

void Graph::visualizeGraphEdges() {
    
    _edges.points.clear();

    geometry_msgs::Point graphEdgePoint1;
    geometry_msgs::Point graphEdgePoint2;
    _edges.header.frame_id = "map";
    _edges.header.stamp = ros::Time::now();
    _edges.lifetime = ros::Duration(0);
    
    _edges.ns = "Optimized edges";
    _edges.action = visualization_msgs::Marker::ADD;
    _edges.type = visualization_msgs::Marker::LINE_LIST;
    _edges.scale.x = 0.05;
    _edges.color.a = 1.0;
    _edges.color.r = 0.0;
    _edges.color.g = 1.0;
    _edges.color.b = 1.0;

    for (auto it = _edgeSet.begin(); it != _edgeSet.end(); ++it) {

        g2o::HyperGraph::Edge* edge = *it;
        g2o::HyperGraph::Vertex* parentVertex = edge->vertices()[0];
        g2o::HyperGraph::Vertex* childVertex = edge->vertices()[1];
        std::vector<double> pose, landmarkPose;

        _graph.vertex(parentVertex->id())->getEstimateData(pose);
        _graph.vertex(childVertex->id())->getEstimateData(landmarkPose);

        graphEdgePoint1.x = pose[0];
        graphEdgePoint1.y = pose[1];
        graphEdgePoint1.z = 0;

        graphEdgePoint2.x = landmarkPose[0];
        graphEdgePoint2.y = landmarkPose[1];
        graphEdgePoint2.z = 0;

        _edges.points.push_back(graphEdgePoint1);
        _edges.points.push_back(graphEdgePoint2);
    }
}
