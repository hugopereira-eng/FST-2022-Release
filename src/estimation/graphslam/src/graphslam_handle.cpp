#include <graphslam/graphslam_handle.hpp>

GraphSlamHandle::GraphSlamHandle(ros::NodeHandle &nodeHandle) : _nodeHandle(nodeHandle), _graphSlam(nodeHandle) {
    ROS_WARN_STREAM("Constructing GraphSlam");
    subscribeToTopics();
    advertiseToTopics();
}

void GraphSlamHandle::advertiseToTopics() {
    ROS_WARN_STREAM("GraphSlam will advertise to topics");
    _pubLandmarks = _nodeHandle.advertise<visualization_msgs::MarkerArray>("/estimation/slam/landmarks", 1);
    _pubGraphVertexes = _nodeHandle.advertise<visualization_msgs::MarkerArray>("/estimation/slam/graph_vertexes", 1);         
    _pubGraphEdges = _nodeHandle.advertise<visualization_msgs::Marker>("/estimation/slam/graph_edges", 1);
    _pubOdometry = _nodeHandle.advertise<nav_msgs::Odometry>("/estimation/slam/odometry", 1);
    _pubCenterline = _nodeHandle.advertise<nav_msgs::Path>("/estimation/slam/centerline", 1);
    _pubCurrentObservations = _nodeHandle.advertise<visualization_msgs::MarkerArray>("/estimation/slam/current_observations", 1);
    _pubCurrentDetections = _nodeHandle.advertise<common_msgs::ConeDetections>("/estimation/slam/cone_detections", 1);
    _pubCurrentDetectionsMarkers = _nodeHandle.advertise<visualization_msgs::MarkerArray>("/estimation/slam/cone_detections_markers", 1); 
    _pubDataAssociationMarkers = _nodeHandle.advertise<visualization_msgs::MarkerArray>("/estimation/slam/data_association_markers", 1);
    _pubMapCones = _nodeHandle.advertise<common_msgs::ConeDetections>("/estimation/slam/map", 1);
    _pubLoopClosure = _nodeHandle.advertise<std_msgs::Bool>("/estimation/slam/loop_closure", 1);
    _pubIcpAlignedPointCloud = _nodeHandle.advertise<sensor_msgs::PointCloud2>("/estimation/slam/icp/aligned", 1); 
    _pubIcpSourcePointCloud = _nodeHandle.advertise<sensor_msgs::PointCloud2>("/estimation/slam/icp/source", 1); 
    _pubIcpTargetPointCloud = _nodeHandle.advertise<sensor_msgs::PointCloud2>("/estimation/slam/icp/target", 1);
    _pubIcpOdometry = _nodeHandle.advertise<nav_msgs::Odometry>("/estimation/slam/icp/odometry", 1);
    _pubConeInfoMarkers = _nodeHandle.advertise<visualization_msgs::MarkerArray>("/estimation/slam/landmarks_id", 1);
}

void GraphSlamHandle::subscribeToTopics() {
    ROS_WARN_STREAM("Subscribed to topics");
    _subVelocity = _nodeHandle.subscribe("/estimation/state_estimation/velocity", 1, &GraphSlamHandle::velocityCallback, this);   
    _subConeDetections = _nodeHandle.subscribe("/perception/cone_detections", 1, &GraphSlamHandle::coneDetectionsCallback, this);
}

void GraphSlamHandle::velocityCallback(const common_msgs::CarVelocity &vel) {
    _graphSlam.updateVelocity(vel);
}

void GraphSlamHandle::coneDetectionsCallback(const common_msgs::ConeDetections &coneDetections) {
    // ROS_INFO_STREAM("CONE DETECTIONS CALLBACK");
    _graphSlam.updateConeDetections(coneDetections);
    run();
}

void GraphSlamHandle::run() {
    publishToTopics();
}

void GraphSlamHandle::publishToTopics() {
    _pubLandmarks.publish(_graphSlam.getLandmarkMarkers());
    _pubGraphVertexes.publish(_graphSlam.getVertexesMarkers());    
    _pubGraphEdges.publish(_graphSlam.getEdgeMarkers());
    _pubOdometry.publish(_graphSlam.getOdometry());
    //_pubCenterline.publish(_graphSlam.getCenterline());
    _pubCurrentObservations.publish(_graphSlam.getCurrentObservationsMarkers());
    _pubCurrentDetections.publish(_graphSlam.getCurrentConeDetections());
    _pubCurrentDetectionsMarkers.publish(_graphSlam.getCurrentConeDetectionsMarkers());
    _pubDataAssociationMarkers.publish(_graphSlam.getDataAssociationMarkers());
    _pubMapCones.publish(_graphSlam.getMapCones());
    _pubLoopClosure.publish(_graphSlam.getLoopClosure());
    _pubIcpAlignedPointCloud.publish(_graphSlam.getAlignedPoincloud());
    _pubIcpSourcePointCloud.publish(_graphSlam.getSourcePointcloud());
    _pubIcpTargetPointCloud.publish(_graphSlam.getTargetPointcloud());
    _pubIcpOdometry.publish(_graphSlam.getIcpOdometry());
    _pubConeInfoMarkers.publish(_graphSlam.getConeInfoMarkers());
}

