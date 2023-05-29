#ifndef PIERRE_HANDLE_HPP
#define PIERRE_HANDLE_HPP

#include <ros/ros.h>
#include <pierre/pierre_pipeline.hpp>

#include <common_msgs/ConeDetections.h>
#include <sensor_msgs/Image.h>

#include <dynamic_reconfigure/server.h>
#include <pierre/PierreConfig.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy0;
typedef message_filters::Synchronizer<SyncPolicy0> Sync0;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2> SyncPolicy1;
typedef message_filters::Synchronizer<SyncPolicy1> Sync1;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2, common_msgs::ConeDetections, common_msgs::ConeDetections> SyncPolicy2;
typedef message_filters::Synchronizer<SyncPolicy2> Sync2;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2, common_msgs::ConeDetections, common_msgs::ConeDetections, nav_msgs::Path> SyncPolicy3;
typedef message_filters::Synchronizer<SyncPolicy3> Sync3;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2, common_msgs::ConeDetections, common_msgs::ConeDetections, nav_msgs::Path, visualization_msgs::Marker> SyncPolicy4;
typedef message_filters::Synchronizer<SyncPolicy4> Sync4;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> SyncPolicy10;
typedef message_filters::Synchronizer<SyncPolicy10> Sync10;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2, common_msgs::ConeDetections, common_msgs::ConeDetections> SyncPolicy20;
typedef message_filters::Synchronizer<SyncPolicy20> Sync20;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2, common_msgs::ConeDetections, common_msgs::ConeDetections, nav_msgs::Path> SyncPolicy30;
typedef message_filters::Synchronizer<SyncPolicy30> Sync30;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2, common_msgs::ConeDetections, common_msgs::ConeDetections, nav_msgs::Path, visualization_msgs::Marker> SyncPolicy40;
typedef message_filters::Synchronizer<SyncPolicy40> Sync40;

class PierreHandle {

public:
  // Constructor
  PierreHandle(ros::NodeHandle &nodeHandle);

  // Methods
  void advertiseToTopics();
  void subscribeToTopics();
  void publishToTopics();
  void subscribeToParametersServer();
  void run();

private:
  // Attributes
  ros::NodeHandle _nodeHandle;
  // Publishers
  ros::Publisher _pubVisualization;
  ros::Publisher _pubLeftImageVisualization;
  ros::Publisher _pubRightImageVisualization;
  // Subscribers
  ros::Subscriber _subCameraImageOnly;
  
  message_filters::Subscriber<sensor_msgs::Image> _subCameraImageLeft;
  message_filters::Subscriber<sensor_msgs::Image> _subCameraImageRight;
  message_filters::Subscriber<sensor_msgs::Image> _subCameraImage;
  message_filters::Subscriber<common_msgs::ConeDetections> _subConeDetections;
  message_filters::Subscriber<common_msgs::ConeDetections> _subCoNetProposals;
  message_filters::Subscriber<sensor_msgs::PointCloud2> _subLidarClusteredPoints;
  message_filters::Subscriber<nav_msgs::Path> _subCenterline;
  message_filters::Subscriber<visualization_msgs::Marker> _subPointToFollow;

  boost::shared_ptr<Sync0> _syncPtr0;
  boost::shared_ptr<Sync1> _syncPtr1;
  boost::shared_ptr<Sync2> _syncPtr2;
  boost::shared_ptr<Sync3> _syncPtr3;
  boost::shared_ptr<Sync4> _syncPtr4;

  boost::shared_ptr<Sync10> _syncPtr10;
  boost::shared_ptr<Sync20> _syncPtr20;
  boost::shared_ptr<Sync30> _syncPtr30;
  boost::shared_ptr<Sync40> _syncPtr40;

  Pierre _pierre;
  // server for Dynamic Ros Parameters
  dynamic_reconfigure::Server<pierre::PierreConfig> _server;

  int _pipeline;
  int _numberCameras;

  // DUAL SETUP
  void jointCallback0(const sensor_msgs::ImageConstPtr &cameraImageLeft,
                      const sensor_msgs::ImageConstPtr &cameraImageRight);

  void jointCallback1(const sensor_msgs::ImageConstPtr &cameraImageLeft,
                      const sensor_msgs::ImageConstPtr &cameraImageRight,
                      const sensor_msgs::PointCloud2ConstPtr &lidarClusteredPoints);

  void jointCallback2(const sensor_msgs::ImageConstPtr &cameraImageLeft,
                      const sensor_msgs::ImageConstPtr &cameraImageRight,
                      const sensor_msgs::PointCloud2ConstPtr &lidarClusteredPoints,
                      const common_msgs::ConeDetectionsConstPtr &coneDetections,
                      const common_msgs::ConeDetectionsConstPtr &coNetProposals);

  void jointCallback3(const sensor_msgs::ImageConstPtr &cameraImageLeft,
                      const sensor_msgs::ImageConstPtr &cameraImageRight,
                      const sensor_msgs::PointCloud2ConstPtr &lidarClusteredPoints,
                      const common_msgs::ConeDetectionsConstPtr &coneDetections,
                      const common_msgs::ConeDetectionsConstPtr &coNetProposals,
                      const nav_msgs::PathConstPtr &centerline);

  void jointCallback4(const sensor_msgs::ImageConstPtr &cameraImageLeft,
                      const sensor_msgs::ImageConstPtr &cameraImageRight,
                      const sensor_msgs::PointCloud2ConstPtr &lidarClusteredPoints,
                      const common_msgs::ConeDetectionsConstPtr &coneDetections,
                      const common_msgs::ConeDetectionsConstPtr &coNetProposals,
                      const nav_msgs::PathConstPtr &centerline,
                      const visualization_msgs::MarkerConstPtr &pointToFollow);

  // MONO SETUP
  void cameraImageCallback(const sensor_msgs::Image &cameraImage);

  void jointCallback10(const sensor_msgs::ImageConstPtr &cameraImage,
                      const sensor_msgs::PointCloud2ConstPtr &lidarClusteredPoints);

  void jointCallback20(const sensor_msgs::ImageConstPtr &cameraImage,
                      const sensor_msgs::PointCloud2ConstPtr &lidarClusteredPoints,
                      const common_msgs::ConeDetectionsConstPtr &coneDetections,
                      const common_msgs::ConeDetectionsConstPtr &coNetProposals);

  void jointCallback30(const sensor_msgs::ImageConstPtr &cameraImage,
                      const sensor_msgs::PointCloud2ConstPtr &lidarClusteredPoints,
                      const common_msgs::ConeDetectionsConstPtr &coneDetections,
                      const common_msgs::ConeDetectionsConstPtr &coNetProposals,
                      const nav_msgs::PathConstPtr &centerline);

  void jointCallback40(const sensor_msgs::ImageConstPtr &cameraImage,
                      const sensor_msgs::PointCloud2ConstPtr &lidarClusteredPoints,
                      const common_msgs::ConeDetectionsConstPtr &coneDetections,
                      const common_msgs::ConeDetectionsConstPtr &coNetProposals,
                      const nav_msgs::PathConstPtr &centerline,
                      const visualization_msgs::MarkerConstPtr &pointToFollow);
  
  void pierreParametersCallback(pierre::PierreConfig &config);

};

#endif