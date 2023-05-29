#ifndef GRAPHSLAM_PIPELINE_HPP
#define GRAPHSLAM_PIPELINE_HPP

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
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

class GraphSlam {
public:
    //Constructor
    GraphSlam(ros::NodeHandle &nodeHandle);

    //setters
    void updateOdometry(const common_msgs::CarPose &odom);
    void updateVelocity(const common_msgs::CarVelocity &vel);
    void updateConeDetections(const common_msgs::ConeDetections &coneDetections);
    void updateCenterline(const nav_msgs::Path &centerline);
    
    //getters
    visualization_msgs::MarkerArray const & getLandmarkMarkers() const;
    visualization_msgs::MarkerArray const & getCurrentObservationsMarkers() const;
    visualization_msgs::MarkerArray const & getCurrentConeDetectionsMarkers() const;
    visualization_msgs::MarkerArray const & getVertexesMarkers() const;    
    visualization_msgs::Marker const & getEdgeMarkers() const;
    visualization_msgs::MarkerArray const & getDataAssociationMarkers() const;
    visualization_msgs::MarkerArray const & getConeInfoMarkers() const;
    nav_msgs::Odometry const & getOdometry() const;
    nav_msgs::Path const & getCenterline() const;
    common_msgs::ConeDetections const & getCurrentConeDetections() const;
    common_msgs::ConeDetections const & getMapCones() const;
    std_msgs::Bool const & getLoopClosure() const;
    //ICP
    sensor_msgs::PointCloud2 const & getAlignedPoincloud() const;
    sensor_msgs::PointCloud2 const & getSourcePointcloud() const;
    sensor_msgs::PointCloud2 const & getTargetPointcloud() const;
    nav_msgs::Odometry const & getIcpOdometry() const;
    

    // Atributes
    std::vector<int> _odometryIndex;
    Pose _odomPose;
    Pose _lastPose;
    Pose _pose;
    Pose _prevPose;

    
private:

    //Attributes
    static std::default_random_engine rng;

    Graph _graphTypes;
    DataAssociation _dataAssociation;
    
    /*************Config Parameters***************/
    Eigen::Matrix2d _observationNoise;
    Eigen::Matrix3d _controlNoise;
    int _observationIncrement;
    double _associationThreshold;
    double _associationRange;
    double _startingHeadingDeviation;
    double _loopClosureFactor;
    double _loopClosureRange;
    int _deleteLandmarkThreshold;
    bool _loadMap;
    std::string _mapPath;
    double _wheelSteering;
    /*********************************************/
    
    size_t _graphIndex = 0;    
        
    std::vector<Landmark> _landmarks;
    bool _firstOdometryUpdate = true; 
    bool _firstLandmarkUpdate = true; 
    ros::Time _lastUpdateTime = ros::Time::now();
    float _distanceIncrement = 0;
    int _partialLoopClosure = 0;
    // std_msgs::Bool _loopClosureDetected;
    std_msgs::Int16 _lapCount;

    // nav_msgs::Odometry _odometry;
    // nav_msgs::Path _centerLine;
    // tf::TransformBroadcaster tf_br_;
    // common_msgs::ConeDetections _currentConeDetections;
    // common_msgs::ConeDetections _mapConeDetections;

    //ICP Atts
    ICP_odom icp_c;
    common_msgs::ConeDetections _previousConeDetections;
    sensor_msgs::PointCloud2 _sourcePointCloud;
    sensor_msgs::PointCloud2 _targetPointCloud;
    sensor_msgs::PointCloud2 _alignedPointCloud;
    bool _icpInit = false;
    int _icpStepCounter = 5;
    nav_msgs::Odometry _icpOdometry;
    Eigen::Vector3f _icpPose;
    // nav_msgs::Odometry _odometry;
    // nav_msgs::Path _centerLine;
    // tf::TransformBroadcaster tf_br_;
    // common_msgs::ConeDetections _currentConeDetections;
    // common_msgs::ConeDetections _mapConeDetections;   

    //Visualization
    // visualization_msgs::MarkerArray _coneMarkers;
    // visualization_msgs::MarkerArray _coneObservationMarkers;
    // visualization_msgs::MarkerArray _currentConeMarkers;
    // visualization_msgs::MarkerArray _dataAssociationMarkers;
    // visualization_msgs::MarkerArray _daMarkers;
    // visualization_msgs::Marker _edges;
    
    //Methods
    Eigen::Vector2d getLandmarkLocation(Observation observation);    
    
    // void updateLandmarks(std::vector<Observation> observations);
    // std::tuple<float, int> computeDataAssociation(Pose pose, Observation observation, std::vector<bool> observedLandmarks, Eigen::Matrix2f &landmarkJacobian, Eigen::Matrix2f &innovationMatrix, Eigen::Vector2f &predictedObservation);
    // Landmark createLandmark(Observation observation, Pose pose);
    // void updateStatistics(std::vector<bool> observedLandmarks, Pose pose);
    // void computeOdometry();
    // void computeCenterLine();
    // void loadMap(ros::NodeHandle &nodeHandle);
    // void publishTf(const Pose &x);
    // void saveOptimizedLandmarks();
    // void saveMap();

    // Visualization
    // void visualizeLandmarks();
    void currentDetectionsVisualization(Pose pose);
    // void visualizeCurrentObservations(std::vector<Observation> observations, Pose pose);
    // void visualizeDataAssociation(std::vector<bool> observedLandmarks);
    void validateDataAssociation();

    //New attributes after re-writing
    common_msgs::ConeDetections _coneDetections;
    std::vector<Observation> _associatedObs;
    SlamPhase _slamMode = SlamPhase::Slam;
    bool _loopClosure = false;
    DaMethod _daMethod;
    std::string _mission;
    std::vector<Observation> _observations;
    tf::TransformBroadcaster tf_br_;
    std::vector<Landmark> _inView;
    geometry_msgs::TransformStamped _transform;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tfListener;

    // used by getters
    visualization_msgs::MarkerArray _coneMarkers;
    visualization_msgs::MarkerArray _daMarkers;
    visualization_msgs::MarkerArray _coneObservationMarkers;
    visualization_msgs::MarkerArray _currentConeMarkers;
    visualization_msgs::MarkerArray _dataAssociationMarkers;
    visualization_msgs::MarkerArray _coneInfoMarkers;
    visualization_msgs::Marker _edges;
    common_msgs::ConeDetections _currentConeDetections;
    common_msgs::ConeDetections _mapConeDetections; 
    nav_msgs::Odometry _odometry;
    nav_msgs::Path _centerLine;  
    std_msgs::Bool _loopClosureDetected;


    // new methods after re-writting
    
    // parameters
    void loadParameters();
    
    // main loop functions
    void run();
    void switchToLocalization();
    void localize();
    void handleAssociations();
    bool getInViewLm(Landmark& lm);
    void updateStatistics();
    void switchToLocal();
    
    // graph related
    Landmark createNewLandmark(Observation& obs);
    void graphAddLm(Observation& obs);
    void graphAddPose();
    void graphUpdateLm(Observation& obs);
    void graphDeleteLm(Landmark& lm);
    void saveOptimizedLandmarks();
    
    // file loading / saving
    void loadMap(ros::NodeHandle& nh);
    void saveMap();
    std::vector<Observation> tracking(std::vector<Observation> observations);

    // odometry / TFs
    void computeOdometry();
    void publishTf(const Pose &x);
    void transformConeDetections();

    // Visualization
    void visualization();
    void visualizeLandmarks();
    void currentDetectionsVisualization();
    //void currentDetectionsVisualization();
    //void visualizeCurrentObservations();
    //void visualizeDataAssociation(std::vector<bool> observedLandmarks);
    //void validateDataAssociation();

};
#endif