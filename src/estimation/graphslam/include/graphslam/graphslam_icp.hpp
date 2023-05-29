#ifndef GRAPHSLAM_ICP_HPP
#define GRAPHSLAM_ICP_HPP

#include "graphslam/graphslam_types.hpp"

#include <ros/ros.h>
#include "graphslam/graphslam_types.hpp"
#include "graphslam/graphslam_motion_model.hpp"
#include "graphslam/graphslam_graph.hpp"
#include "graphslam/graphslam_data_association.hpp"
#include "graphslam/graphslam_icp.hpp"
#include <eigen3/Eigen/Eigen>
#include <fstream>
#include <random>
#include <math.h>
#include <stdlib.h>
#include <common_msgs/CarVelocity.h>
#include <common_msgs/CarPose.h>
#include <common_msgs/ConeDetections.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <common_msgs/ControlCmd.h>
#include <pcl_msgs/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/print.h>

typedef pcl::PointXYZI PclPoint;
typedef pcl::PointCloud<PclPoint> PclPointCloud;
typedef pcl::PointXYZINormal PclPointNormal;
typedef pcl::PointCloud<PclPointNormal> PclPointCloudNormal;

class ICP_odom {

    public:
        //Constructor
        ICP_odom();


        // getter
        common_msgs::ConeDetections const & getCurrentConeDetections() const;
        sensor_msgs::PointCloud2 const & getAlignedPoincloud() const;
        sensor_msgs::PointCloud2 const & getSourcePointcloud() const;
        sensor_msgs::PointCloud2 const & getTargetPointcloud() const;
        nav_msgs::Odometry const & getIcpOdometry() const;

        void run(const common_msgs::ConeDetections &coneDetections);
    
    private:
        void icpWihtoutOutliers(const common_msgs::ConeDetections &coneDetections);
        float distance_to_line(float x1,float y1, float x2, float y2, float x0, float y0);
        void computeIcpOdometry();
        sensor_msgs::PointCloud2 convertPcl2Ros(PclPointCloud::Ptr pclPointCloud);

        std_msgs::Header _detectionsHeader;
        
        //ICP Atts
        common_msgs::ConeDetections _previousConeDetections;
        common_msgs::ConeDetections _currentConeDetections;
        sensor_msgs::PointCloud2 _sourcePointCloud;
        sensor_msgs::PointCloud2 _targetPointCloud;
        sensor_msgs::PointCloud2 _alignedPointCloud;
        bool _icpInit = false;
        int _icpStepCounter = 5;
        nav_msgs::Odometry _icpOdometry;
        Eigen::Vector3f _icpPose;

        // Atributes
        std::vector<int> _odometryIndex;
        Pose _odomPose;
        Pose _lastPose;
        Pose _pose;
        Pose _prevPose;

};

#endif