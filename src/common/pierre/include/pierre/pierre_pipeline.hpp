#ifndef PIERRE_PIPELINE_HPP
#define PIERRE_PIPELINE_HPP

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <common_msgs/ConeDetections.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
#include <pierre/PierreConfig.h>
#include <cv_bridge/cv_bridge.h>
#include <as_lib/common.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_ros/point_cloud.h>
typedef enum Pipeline {
    THESIS, DARKNET
} Pipeline;

typedef enum Camera_side{
    LEFT_CAMERA, RIGHT_CAMERA
} Camera_side;

typedef struct Camera {
    Eigen::Matrix3d K;
    Eigen::Matrix3d R;
    Eigen::Vector3d T;
    Camera_side side;
}Camera;

typedef pcl::PointXYZI PclPoint;
typedef pcl::PointCloud<PclPoint> PclPointCloud;
class Pierre {
public:
    // Constructor:
    Pierre();

    // Getters:
    sensor_msgs::Image const & getVisualization() const;
    sensor_msgs::Image const & getLeftCameraVisualization() const;
    sensor_msgs::Image const & getRightCameraVisualization() const;

    // Update methods:
    void updateCameraImage(const sensor_msgs::Image &newCameraImage);
    void updateConeDetections(const common_msgs::ConeDetections &newConeDetections);
    void updateCoNetProposals(const common_msgs::ConeDetections &newCoNetProposals);
    void updateLidarClusteredPoints(const sensor_msgs::PointCloud2 &newLidarClusteredPoints);
    void updateCenterline(const nav_msgs::Path &newCenterline);
    void updatePointToFollow(const visualization_msgs::Marker &newPointToFollow);
    void updateConfiguration(int configuration);
    void updatePipelineParameters(const pierre::PierreConfig &config);
   
    void runAlgorithm();
    std::tuple<int,int> initParameters();

private:
    // Attributes:
    double _tx, _ty, _tz, _rx, _ry, _rz;
    
    sensor_msgs::Image _visualization;
    sensor_msgs::Image _leftCameraVisualization;
    sensor_msgs::Image _rightCameraVisualization;
    std::vector<sensor_msgs::Image> _cameraImageVector;
    common_msgs::ConeDetections _coNetProposals;
    common_msgs::ConeDetections _coneDetections;
    sensor_msgs::PointCloud2 _lidarClusteredPoints;
    PclPointCloud::Ptr _pclPointCloud;
    nav_msgs::Path _centerline;
    geometry_msgs::Point _pointToFollow;
    bool _manualCalibration; // toggles manual calibration
    int _numberCameras;
    int _pipeline;
    int _configuration;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tfListener;
    
    // Methods:
    void visualizeEverythingOnImage();
    std::vector<Eigen::Vector4d> pointsConversion(std::vector<Eigen::Vector4d> points);
    std::vector<std::vector<Eigen::Vector4d>> projectBoundingBox(std::vector<Eigen::Vector4d> points);
    std::vector<Eigen::Vector4d> extractPointsFromCloud();
    cv::Scalar getBoundingBoxColor(int coneColor);
    std::vector<Eigen::Vector3d> centerlineConversion();
    void transformConeDetections();
    //camera parameters methods
    void buildMatrices(Camera&);
    double toRad(double);
    
    Camera _left;
    Camera _right;
};

#endif
