#ifndef FASTSLAM_PIPELINE_HPP
#define FASTSLAM_PIPELINE_HPP

#include <fastslam2_0/fastslam2_0_types.hpp>
#include <fastslam2_0/fastslam2_0_particle.hpp>
#include <common_msgs/CarVelocity.h>
#include <common_msgs/CarPose.h>
#include <common_msgs/ConeDetections.h>
#include <pcl_ros/point_cloud.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include "fssim_common/State.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class FastSlam {
public:
    //Constructor
    FastSlam(ros::NodeHandle &nodeHandle);

    //setters
    void updateOdometry(const common_msgs::CarPose &odom);
    void updateVelocity(const common_msgs::CarVelocity &vel);
    void updateConeDetections(const common_msgs::ConeDetections &coneDetections);   
    void updateCenterline(const nav_msgs::Path &centerline); 
    
    //getters
    visualization_msgs::MarkerArray const & getLandmarks() const;
    geometry_msgs::PoseArray const & getParticlePoses() const;
    nav_msgs::Path const & getCenterLine() const;
    nav_msgs::Odometry const & getOdometry() const;
    common_msgs::ConeDetections const & getCurrentConeDetections() const; 
    common_msgs::ConeDetections const & getMapCones() const;
    visualization_msgs::MarkerArray const & getCurrentConeDetectionsVisualization() const;
    visualization_msgs::MarkerArray const & getCurrentObservationsVisualization() const;

private:

    //Attributes
    static std::default_random_engine rng;
    SLAM_PHASE _currentSlamPhase = SLAM_PHASE_MAP_BUILDING;
    std::vector<Particle> _particles;
    nav_msgs::Odometry _odometry;
    std::vector<float> _weights;
    int _particleNumber ;
    int _observationIncrement;
    bool _firstUpdate = true;
    float _resampleFactor;
    float _loopClosureParticleFactor;
    float _loopClosureFactor;
    Eigen::Matrix2f _observationNoise;
    Eigen::Matrix3f _controlNoise;
    float _newLandmarkThreshold;
    float _startingHeadingDeviation;
    bool _loadMap;
    std::string _mapName;
    std::string _mapLocation;
    ros::Time _lastUpdateTime = ros::Time::now();
    std_msgs::Header _detectionsHeader;
    tf::TransformBroadcaster tf_br_;
    std::vector<Landmark> _landmarks;
    
    Particle _mostLikelyParticle;    

    //Visualization
    visualization_msgs::MarkerArray _coneMarkers;
    geometry_msgs::PoseArray _particlePoses;
    nav_msgs::Path _centerLine;
    common_msgs::ConeDetections _currentConeDetections;
    common_msgs::ConeDetections _mapConeDetections;
    visualization_msgs::MarkerArray _currentConeMarkers;
    visualization_msgs::MarkerArray _currentObservations;

    //Methods
    void initParticles(Pose initialPosition);
    void updateParticleKalman(std::vector<Observation> observations);
    std::vector<Particle> resampleParticles(); 
    void computeOdometry();
    void tempParticlePoseAndStdDev(Eigen::Vector3f &mean, Eigen::Vector3f &std);
    void particlePose(Eigen::ArrayXf &x, Eigen::ArrayXf &y, Eigen::ArrayXf &theta);
    void particlePoseAndStdDev(Eigen::Vector3f &mean, Eigen::Vector3f &std,Eigen::ArrayXf &x, Eigen::ArrayXf &y, Eigen::ArrayXf &theta);
    void publishTf(const double x, const double y, const double theta);
    void loadMap(ros::NodeHandle &nodeHandle);
    void computeCenterLine();
    void saveMap();
    
    //Visualization
    void landmarkVisualization();
    void particleVisualization();
    void currentDetectionsVisualization();
    void currentObservationsVisualization(std::vector<Observation> observations);
};

#endif