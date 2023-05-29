#include <pierre/pierre_handle.hpp>

PierreHandle::PierreHandle(ros::NodeHandle &nodeHandle) : _nodeHandle(nodeHandle) {
    advertiseToTopics();
    std::tie(_pipeline, _numberCameras) = _pierre.initParameters();
    subscribeToTopics();
    subscribeToParametersServer();
}

void PierreHandle::advertiseToTopics() {
    _pubVisualization = _nodeHandle.advertise<sensor_msgs::Image>("/common/pierre/visualization", 1);
    _pubLeftImageVisualization = _nodeHandle.advertise<sensor_msgs::Image>("/common/pierre/left_image_visualization", 1);
    _pubRightImageVisualization = _nodeHandle.advertise<sensor_msgs::Image>("/common/pierre/right_image_visualization", 1);
}

void PierreHandle::subscribeToTopics() {
    ROS_INFO("subscribe to topics");
    if(_pipeline == DARKNET)
        _subCameraImage.subscribe(_nodeHandle, "/perception/darknet_ros/detection_image", 1);
    
    _subConeDetections.subscribe(_nodeHandle, "/perception/data_association/cone_detections", 1);
    _subCoNetProposals.subscribe(_nodeHandle, "/perception/sensor_fusion/coNet_proposals", 1);
    _subLidarClusteredPoints.subscribe(_nodeHandle, "/perception/lidar/clustered_points", 1);
    _subCenterline.subscribe(_nodeHandle, "/control/svm/centerline", 1);
    _subPointToFollow.subscribe(_nodeHandle, "/control/controller/vis/point_to_follow", 1);
   
    if(_numberCameras == 2) {
        _subCameraImageLeft.subscribe(_nodeHandle, "/arena_camera_node_2/image_raw", 1);
        _subCameraImageRight.subscribe(_nodeHandle, "/arena_camera_node_1/image_raw", 1);
        // _syncPtr0.reset(new Sync0(SyncPolicy0(20), _subCameraImage, _subCameraImage2));
        // _syncPtr0->registerCallback(boost::bind(&PierreHandle::jointCallback0, this, _1, _2));
        _syncPtr1.reset(new Sync1(SyncPolicy1(20), _subCameraImageLeft, _subCameraImageRight, _subLidarClusteredPoints));
        _syncPtr1->registerCallback(boost::bind(&PierreHandle::jointCallback1, this, _1, _2, _3));
        _syncPtr2.reset(new Sync2(SyncPolicy2(20), _subCameraImageLeft, _subCameraImageRight, _subLidarClusteredPoints, _subConeDetections, _subCoNetProposals));
        _syncPtr2->registerCallback(boost::bind(&PierreHandle::jointCallback2, this, _1, _2, _3, _4, _5));
        _syncPtr3.reset(new Sync3(SyncPolicy3(20), _subCameraImageLeft, _subCameraImageRight, _subLidarClusteredPoints, _subConeDetections, _subCoNetProposals, _subCenterline));
        _syncPtr3->registerCallback(boost::bind(&PierreHandle::jointCallback3, this, _1, _2, _3, _4, _5, _6));
        _syncPtr4.reset(new Sync4(SyncPolicy4(20), _subCameraImageLeft, _subCameraImageRight, _subLidarClusteredPoints, _subConeDetections, _subCoNetProposals, _subCenterline, _subPointToFollow));
        _syncPtr4->registerCallback(boost::bind(&PierreHandle::jointCallback4, this, _1, _2, _3, _4, _5, _6, _7));
    } else {
        _subCameraImageOnly = _nodeHandle.subscribe("/arena_camera_node/image_raw", 1, &PierreHandle::cameraImageCallback, this);
        _subCameraImage.subscribe(_nodeHandle, "/arena_camera_node/image_raw", 1);
        _syncPtr10.reset(new Sync10(SyncPolicy10(20), _subCameraImage, _subLidarClusteredPoints));
        _syncPtr10->registerCallback(boost::bind(&PierreHandle::jointCallback10, this, _1, _2));
        _syncPtr20.reset(new Sync20(SyncPolicy20(20), _subCameraImage, _subLidarClusteredPoints, _subConeDetections, _subCoNetProposals));
        _syncPtr20->registerCallback(boost::bind(&PierreHandle::jointCallback20, this, _1, _2, _3, _4));
        _syncPtr30.reset(new Sync30(SyncPolicy30(20), _subCameraImage, _subLidarClusteredPoints, _subConeDetections, _subCoNetProposals, _subCenterline));
        _syncPtr30->registerCallback(boost::bind(&PierreHandle::jointCallback30, this, _1, _2, _3, _4, _5));
        _syncPtr40.reset(new Sync40(SyncPolicy40(20), _subCameraImage, _subLidarClusteredPoints, _subConeDetections, _subCoNetProposals, _subCenterline, _subPointToFollow));
        _syncPtr40->registerCallback(boost::bind(&PierreHandle::jointCallback40, this, _1, _2, _3, _4, _5, _6));
    }

}

void PierreHandle::subscribeToParametersServer() {
    dynamic_reconfigure::Server<pierre::PierreConfig>::CallbackType f;
    f = boost::bind(&PierreHandle::pierreParametersCallback, this, _1);

    _server.setCallback(f);
}

void PierreHandle::run() {
    _pierre.runAlgorithm();
    publishToTopics();
}

void PierreHandle::publishToTopics() {
    _pubVisualization.publish(_pierre.getVisualization());
    _pubLeftImageVisualization.publish(_pierre.getLeftCameraVisualization());
    _pubRightImageVisualization.publish(_pierre.getRightCameraVisualization());
}

void PierreHandle::pierreParametersCallback(pierre::PierreConfig &config) {
    _pierre.updatePipelineParameters(config);
    run();
}

void PierreHandle::jointCallback0(const sensor_msgs::ImageConstPtr &cameraImageLeft,
                                const sensor_msgs::ImageConstPtr &cameraImageRight) {

    ROS_INFO("Pierre: JOINT CALLBACK 0 CALLED!");
    _pierre.updateCameraImage(*cameraImageLeft);
    _pierre.updateCameraImage(*cameraImageRight);
    _pierre.updateConfiguration(0);
    run();
}

void PierreHandle::jointCallback1(const sensor_msgs::ImageConstPtr &cameraImageLeft,
                                const sensor_msgs::ImageConstPtr &cameraImageRight,
                                const sensor_msgs::PointCloud2ConstPtr &lidarClusteredPoints) {

    ROS_INFO("Pierre: JOINT CALLBACK 1 CALLED!");
    _pierre.updateCameraImage(*cameraImageLeft);
    _pierre.updateCameraImage(*cameraImageRight);
    _pierre.updateLidarClusteredPoints(*lidarClusteredPoints);
    _pierre.updateConfiguration(1);
    run();
}

void PierreHandle::jointCallback2(const sensor_msgs::ImageConstPtr &cameraImageLeft,
                                const sensor_msgs::ImageConstPtr &cameraImageRight,
                                const sensor_msgs::PointCloud2ConstPtr &lidarClusteredPoints,
                                const common_msgs::ConeDetectionsConstPtr &coneDetections,
                                const common_msgs::ConeDetectionsConstPtr &coNetProposals) {

    ROS_INFO("Pierre: JOINT CALLBACK 2 CALLED!");
    _syncPtr1.reset();
    _pierre.updateCameraImage(*cameraImageLeft);
    _pierre.updateCameraImage(*cameraImageRight);
    _pierre.updateLidarClusteredPoints(*lidarClusteredPoints);
    _pierre.updateConeDetections(*coneDetections);
    _pierre.updateCoNetProposals(*coNetProposals);
    _pierre.updateConfiguration(2);
    run();
}

void PierreHandle::jointCallback3(const sensor_msgs::ImageConstPtr &cameraImageLeft,
                                const sensor_msgs::ImageConstPtr &cameraImageRight,
                                const sensor_msgs::PointCloud2ConstPtr &lidarClusteredPoints,
                                const common_msgs::ConeDetectionsConstPtr &coneDetections,
                                const common_msgs::ConeDetectionsConstPtr &coNetProposals,
                                const nav_msgs::PathConstPtr &centerline) {

    ROS_INFO("Pierre: JOINT CALLBACK 3 CALLED!");
    // _syncPtr0.reset();
    _syncPtr1.reset();
    _syncPtr2.reset();
    _pierre.updateCameraImage(*cameraImageLeft);
    _pierre.updateCameraImage(*cameraImageRight);
    _pierre.updateLidarClusteredPoints(*lidarClusteredPoints);
    _pierre.updateConeDetections(*coneDetections);
    _pierre.updateCoNetProposals(*coNetProposals);
    _pierre.updateCenterline(*centerline);
    _pierre.updateConfiguration(3);
    run();
}

void PierreHandle::jointCallback4(const sensor_msgs::ImageConstPtr &cameraImageLeft,
                                const sensor_msgs::ImageConstPtr &cameraImageRight,
                                const sensor_msgs::PointCloud2ConstPtr &lidarClusteredPoints,
                                const common_msgs::ConeDetectionsConstPtr &coneDetections,
                                const common_msgs::ConeDetectionsConstPtr &coNetProposals,
                                const nav_msgs::PathConstPtr &centerline,
                                const visualization_msgs::MarkerConstPtr &pointToFollow) {

    ROS_INFO("Pierre: JOINT CALLBACK 4 CALLED!");
    _syncPtr0.reset();
    _syncPtr1.reset();
    _syncPtr2.reset();
    _syncPtr3.reset();
    _pierre.updateCameraImage(*cameraImageLeft);
    _pierre.updateCameraImage(*cameraImageRight);
    _pierre.updateLidarClusteredPoints(*lidarClusteredPoints);
    _pierre.updateConeDetections(*coneDetections);
    _pierre.updateCoNetProposals(*coNetProposals);
    _pierre.updateCenterline(*centerline);
    _pierre.updatePointToFollow(*pointToFollow);
    _pierre.updateConfiguration(4);
    run();
}

void PierreHandle::cameraImageCallback(const sensor_msgs::Image &cameraImage) {
    _pierre.updateCameraImage(cameraImage);
    _pierre.updateConfiguration(0);
    run();
}

void PierreHandle::jointCallback10(const sensor_msgs::ImageConstPtr &cameraImage,
                                const sensor_msgs::PointCloud2ConstPtr &lidarClusteredPoints) {

    ROS_INFO("Pierre: JOINT CALLBACK 10 CALLED!");
    _subCameraImageOnly.shutdown();
    _pierre.updateCameraImage(*cameraImage);
    _pierre.updateLidarClusteredPoints(*lidarClusteredPoints);
    _pierre.updateConfiguration(1);
    run();
}

void PierreHandle::jointCallback20(const sensor_msgs::ImageConstPtr &cameraImage,
                                const sensor_msgs::PointCloud2ConstPtr &lidarClusteredPoints,
                                const common_msgs::ConeDetectionsConstPtr &coneDetections,
                                const common_msgs::ConeDetectionsConstPtr &coNetProposals) {

    ROS_INFO("Pierre: JOINT CALLBACK 20 CALLED!");
    _subCameraImageOnly.shutdown();
    _syncPtr10.reset();
    _pierre.updateCameraImage(*cameraImage);
    _pierre.updateLidarClusteredPoints(*lidarClusteredPoints);
    _pierre.updateConeDetections(*coneDetections);
    _pierre.updateCoNetProposals(*coNetProposals);
    _pierre.updateConfiguration(2);
    run();
}

void PierreHandle::jointCallback30(const sensor_msgs::ImageConstPtr &cameraImage,
                                const sensor_msgs::PointCloud2ConstPtr &lidarClusteredPoints,
                                const common_msgs::ConeDetectionsConstPtr &coneDetections,
                                const common_msgs::ConeDetectionsConstPtr &coNetProposals,
                                const nav_msgs::PathConstPtr &centerline) {

    ROS_INFO("Pierre: JOINT CALLBACK 30 CALLED!");
    _subCameraImageOnly.shutdown();
    _syncPtr10.reset();
    _syncPtr20.reset();
    _pierre.updateCameraImage(*cameraImage);
    _pierre.updateLidarClusteredPoints(*lidarClusteredPoints);
    _pierre.updateConeDetections(*coneDetections);
    _pierre.updateCoNetProposals(*coNetProposals);
    _pierre.updateCenterline(*centerline);
    _pierre.updateConfiguration(3);
    run();
}

void PierreHandle::jointCallback40(const sensor_msgs::ImageConstPtr &cameraImage,
                                const sensor_msgs::PointCloud2ConstPtr &lidarClusteredPoints,
                                const common_msgs::ConeDetectionsConstPtr &coneDetections,
                                const common_msgs::ConeDetectionsConstPtr &coNetProposals,
                                const nav_msgs::PathConstPtr &centerline,
                                const visualization_msgs::MarkerConstPtr &pointToFollow) {

    ROS_INFO("Pierre: JOINT CALLBACK 40 CALLED!");
    _subCameraImageOnly.shutdown();
    _syncPtr10.reset();
    _syncPtr20.reset();
    _syncPtr30.reset();
    _pierre.updateCameraImage(*cameraImage);
    _pierre.updateLidarClusteredPoints(*lidarClusteredPoints);
    _pierre.updateConeDetections(*coneDetections);
    _pierre.updateCoNetProposals(*coNetProposals);
    _pierre.updateCenterline(*centerline);
    _pierre.updatePointToFollow(*pointToFollow);
    _pierre.updateConfiguration(4);
    run();
}
