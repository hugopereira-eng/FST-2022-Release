#include <fastslam2_0/fastslam2_0_handle.hpp>
#include <chrono>

FastSlamHandle::FastSlamHandle(ros::NodeHandle &nodeHandle) : _nodeHandle(nodeHandle), _fastSlam(nodeHandle) {
    ROS_WARN_STREAM("Constructing FastSlam2.0");
    subscribeToTopics();
    advertiseToTopics();
}

void FastSlamHandle::advertiseToTopics() {
    ROS_WARN_STREAM("FastSlam2.0 will advertise to topics");
    _pubSlamCones = _nodeHandle.advertise<visualization_msgs::MarkerArray>("/estimation/slam/landmarks", 1);
    _pubSlamParticles = _nodeHandle.advertise<geometry_msgs::PoseArray>("/estimation/slam/particles", 1);
    _pubCenterLine = _nodeHandle.advertise<nav_msgs::Path>("/estimation/slam/centerline", 1);
    _pubOdometry = _nodeHandle.advertise<nav_msgs::Odometry>("/estimation/slam/odometry", 1);
    _pubCurrentConeDetections = _nodeHandle.advertise<common_msgs::ConeDetections>("/estimation/slam/cone_detections", 1);
    _pubCurrentConeMarkers = _nodeHandle.advertise<visualization_msgs::MarkerArray>("/estimation/slam/cone_detections_markers", 1);  
    _pubCurrentObservations = _nodeHandle.advertise<visualization_msgs::MarkerArray>("/estimation/slam/current_observations", 1); 
    _pubMapCones = _nodeHandle.advertise<common_msgs::ConeDetections>("/estimation/slam/map", 1);
}

void FastSlamHandle::subscribeToTopics() {
    ROS_WARN_STREAM("FastSlam2.0 Subscribed to topics");
    _subOdometry = _nodeHandle.subscribe("/estimation/state_estimation/position", 1, &FastSlamHandle::odometryCallback, this);
    _subVelocity = _nodeHandle.subscribe("/estimation/state_estimation/velocity", 1, &FastSlamHandle::velocityCallback, this);
    _subConeDetections = _nodeHandle.subscribe("/perception/cone_detections", 1, &FastSlamHandle::coneDetectionsCallback, this);
    _subSVMCenterline = _nodeHandle.subscribe("/estimation/svm/centerline", 1, &FastSlamHandle::svmCenterlineCallback, this);
}

void FastSlamHandle::velocityCallback(const common_msgs::CarVelocity &vel) {
    _fastSlam.updateVelocity(vel);
}

void FastSlamHandle::odometryCallback(const common_msgs::CarPose &odom) {
    // ROS_INFO_STREAM("ODOMETRY CALLBACK");
    _fastSlam.updateOdometry(odom);

}   

void FastSlamHandle::coneDetectionsCallback(const common_msgs::ConeDetections &coneDetections) {
    // ROS_INFO_STREAM("CONE DETECTIONS CALLBACK");

    auto start = std::chrono::steady_clock::now();
    _fastSlam.updateConeDetections(coneDetections);
    run();
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    sum += elapsed_seconds.count();
    auto average = sum/n;
    std::cout << "Average Time: " << average << "s\n";
    n++;
}

void FastSlamHandle::svmCenterlineCallback(const nav_msgs::Path &centerline) {
    _fastSlam.updateCenterline(centerline);
}

void FastSlamHandle::run() {
    publishToTopics();
}

void FastSlamHandle::publishToTopics() {
    _pubSlamCones.publish(_fastSlam.getLandmarks());
    _pubSlamParticles.publish(_fastSlam.getParticlePoses());
    _pubCenterLine.publish(_fastSlam.getCenterLine());
    _pubOdometry.publish(_fastSlam.getOdometry());
    _pubCurrentConeDetections.publish(_fastSlam.getCurrentConeDetections());
    _pubCurrentConeMarkers.publish(_fastSlam.getCurrentConeDetectionsVisualization());
    _pubCurrentObservations.publish(_fastSlam.getCurrentObservationsVisualization());
    _pubMapCones.publish(_fastSlam.getMapCones());
}
