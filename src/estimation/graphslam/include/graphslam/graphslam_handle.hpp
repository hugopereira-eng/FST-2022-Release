#ifndef GRAPHSLAM_HANDLE_HPP
#define GRAPHSLAM_HANDLE_HPP

#include <graphslam/graphslam_types.hpp>
#include <graphslam/graphslam_pipeline.hpp>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseArray.h>
#include <common_msgs/Cone.h>
#include <common_msgs/ConeDetections.h>
#include <common_msgs/CarVelocity.h>
#include <common_msgs/Track.h>
#include <common_msgs/CarPose.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

class GraphSlamHandle {
public:
    // Constructor
    GraphSlamHandle(ros::NodeHandle &nodeHandle);

    // Methods
    void advertiseToTopics();
    void subscribeToTopics();
    void publishToTopics();
    void run();

private:
    // Attributes
    ros::NodeHandle _nodeHandle;
    
    ros::Subscriber _subVelocity;
    ros::Subscriber _subConeDetections;

    ros::Publisher _pubLandmarks;
    ros::Publisher _pubGraphVertexes;    
    ros::Publisher _pubGraphEdges;
    ros::Publisher _pubOdometry;
    ros::Publisher _pubCenterline;
    ros::Publisher _pubCurrentObservations;
    ros::Publisher _pubCurrentDetections;
    ros::Publisher _pubCurrentDetectionsMarkers;
    ros::Publisher _pubDataAssociationMarkers;
    ros::Publisher _pubMapCones;
    ros::Publisher _pubLoopClosure;
    ros::Publisher _pubIcpAlignedPointCloud;
	ros::Publisher _pubIcpSourcePointCloud;
	ros::Publisher _pubIcpTargetPointCloud;
    ros::Publisher _pubIcpOdometry;
    ros::Publisher _pubConeInfoMarkers;

    GraphSlam _graphSlam;

    // Methods
    void velocityCallback(const common_msgs::CarVelocity &vel);
    void coneDetectionsCallback(const common_msgs::ConeDetections &coneDetections);
};

#endif