#include "svm/svm_handle.hpp"

SVMHandle::SVMHandle(ros::NodeHandle &nodeHandle) : _nodeHandle(nodeHandle) {
    getConeDetectionsMethod();
    advertiseToTopics();
    subscribeToTopics();
}

void SVMHandle::getConeDetectionsMethod() {
    if(!ros::param::has("/svm/coneDetections")) {
        ROS_WARN_STREAM("Could not load parameter coneDetections. Default will be assumed");
        _coneDetections = SENSORFUSION;
    }

    int coneDetections;
    ros::param::get("/svm/coneDetections", coneDetections);
    _coneDetections = ConeDetections(coneDetections);
}

void SVMHandle::advertiseToTopics() {
    _pubPath = _nodeHandle.advertise<nav_msgs::Path>("/control/svm/centerline", 1);
    _pubSvmImage = _nodeHandle.advertise<sensor_msgs::Image>("/control/svm/vis/svm_image", 1);
}

void SVMHandle::subscribeToTopics() {
    switch (_coneDetections){
    case SLAM:
        _subConeDetectionsSlam = _nodeHandle.subscribe("/estimation/slam/cone_detections", 1, &SVMHandle::coneDetectionsSlamCallback, this);
        break;
    case SENSORFUSION:
        _subConeDectionsSensorFusion = _nodeHandle.subscribe("/perception/cone_detections", 1, &SVMHandle::coneDetectionsSensorFusionCallback, this);
        break;
    default:
        break;
    }
}


void SVMHandle::run() {
    _svm.runAlgorithm();
    publishToTopics();
}

void SVMHandle::publishToTopics() {
    _pubPath.publish(_svm.getPath());
    _pubSvmImage.publish(_svm.getSvmImage());
}

void SVMHandle::coneDetectionsSlamCallback(const common_msgs::ConeDetections &conesDetected) {
    _svm.setConeDetectionsSlam(conesDetected);
}

void SVMHandle::coneDetectionsSensorFusionCallback(const common_msgs::ConeDetections &conesDetected) {
    _svm.setConeDetectionsSensorFusion(conesDetected);
}