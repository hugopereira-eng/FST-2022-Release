#ifndef FASTSLAM_HANDLE_HPP
#define FASTSLAM_HANDLE_HPP

#include <fastslam2_0/fastslam2_0_types.hpp>
#include <fastslam2_0/fastslam2_0_pipeline.hpp>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseArray.h>
#include <common_msgs/Cone.h>
#include <common_msgs/ConeDetections.h>
#include <common_msgs/CarVelocity.h>
#include <common_msgs/CarPose.h>
#include <common_msgs/Track.h>
#include "fssim_common/State.h"
#include <visualization_msgs/MarkerArray.h>

class FastSlamHandle {
public:
    // Constructor
    FastSlamHandle(ros::NodeHandle &nodeHandle);

    // Methods
    void advertiseToTopics();
    void subscribeToTopics();
    void publishToTopics();
    void run();

private:
    // Attributes
    ros::NodeHandle _nodeHandle;
    
    ros::Subscriber _subOdometry;
    ros::Subscriber _subVelocity;
    ros::Subscriber _subConeDetections;
    ros::Subscriber _subSVMCenterline;

    ros::Publisher _pubSlamCones;
    ros::Publisher _pubSlamParticles;
    ros::Publisher _pubCenterLine;
    ros::Publisher _pubOdometry;
    ros::Publisher _pubCurrentConeDetections;
    ros::Publisher _pubCurrentConeMarkers;
    ros::Publisher _pubCurrentObservations;
    ros::Publisher _pubMapCones;

    FastSlam _fastSlam;
    int n = 0;
    double sum = 0;

    // Methods
    void odometryCallback(const common_msgs::CarPose &odom);
    void velocityCallback(const common_msgs::CarVelocity &vel);
    void coneDetectionsCallback(const common_msgs::ConeDetections &coneDetections);
    void svmCenterlineCallback(const nav_msgs::Path &centerline);
};

#endif