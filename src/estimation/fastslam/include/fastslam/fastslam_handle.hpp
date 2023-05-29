#ifndef FASTSLAM_HANDLE_HPP
#define FASTSLAM_HANDLE_HPP

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <ros/node_handle.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseArray.h>
#include "common_msgs/Cone.h"
#include "common_msgs/ConeDetections.h"
#include "common_msgs/CarVelocity.h"
#include "common_msgs/CarPose.h"
#include "common_msgs/Track.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
#include "fastslam/fastslam_particle.hpp"
#include "fastslam/fastslam_motion_model.hpp"
#include "fastslam/fastslam_util.hpp"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

struct OdoMeasurement {
    Eigen::Vector3f val;
    ros::Time timestamp;
};

class FastSlamHandle {

public: 
    // Constructor
    FastSlamHandle(ros::NodeHandle &nodeHandle);

    // Methods
    void advertiseToTopics(); 
    void subscribeToTopics();
  
    
private:
    // Methods
    void odometryCallback(const common_msgs::CarVelocity &odo);
    void coneDetectionsCallback(const common_msgs::ConeDetections &conesDetected);
    void poseCallback(const common_msgs::CarPose &position);

    void setupThreadPool(size_t count);

    void update(std::vector<Observation> observations, ros::Time observationTime);
    std::vector<Particle> resampleParticles();

    void init();
    void initParticles(Pose initialPosition);
    void loadMap(ros::NodeHandle &nodeHandle);

    void particlePoses(Eigen::ArrayXf &x, Eigen::ArrayXf &y, Eigen::ArrayXf &theta);
    void tempParticlePoses(Eigen::ArrayXf &x, Eigen::ArrayXf &y, Eigen::ArrayXf &theta);
    void particlesPoseMeanAndStdDev(Eigen::Vector3f &mean, Eigen::Vector3f &std, Eigen::ArrayXf &x, Eigen::ArrayXf &y, Eigen::ArrayXf &theta);
    void particlesPoseMeanAndStdDev(Eigen::Vector3f &mean, Eigen::Vector3f &std);
    void tempParticlesPoseMeanAndStdDev(Eigen::Vector3f &mean, Eigen::Vector3f &std);
    void particlesPoseStdDeviation(float &xStd, float &yStd, float &thetaStd);

    void publishCones();
    void publishFrontFacingCones();
    void publishParticles();
    void publishRelativeLandmarksAndMeasurements(std::vector<Observation> &observations);
    void publishOdometry(ros::Time time);
    void publishTf(const float x, const float y, const float yaw);

    void skidAccCenterLine();
    void centerline();

    //Attributes
    ros::NodeHandle _nodeHandle;
    ros::Publisher _pubSlamConesAutoTests;
    ros::Publisher _pubSlamLoopClosure;
    ros::Publisher _pubSlamCones;
    ros::Publisher _pubFrontFacingCones;
    ros::Publisher _pubSlamFrontConeDetections;
    ros::Publisher _pubRelativeCones;
    ros::Publisher _pubRelativeMeasurements;
    ros::Publisher _pubParticles;
    ros::Publisher _pubOdometry;
    ros::Publisher _pubCenterLine;
    ros::Subscriber _subOdom;
    ros::Subscriber _subCarPose;
    ros::Subscriber _subConeDetections;
    std::default_random_engine rng;
    boost::asio::io_service ioService;
    boost::thread_group workerThreads;
    boost::asio::io_service::work _ioServiceLock;

    ros::Time lastUpdateTime = ros::Time::now();

    std::vector<Particle> particles;
    std::vector<OdoMeasurement> odoMeasurements;

    Pose _pose;
    std::vector<Landmark> _landmarks;
    nav_msgs::Path _centerline;
    bool firstOdoUpdate = true;
    bool firstUpdate = true;

    Particle mostLikelyParticle;
    Control lastControl;

    SLAM_PHASE currentSlamPhase = SLAM_PHASE_MAP_BUILDING;

    /* SLAM config parameters */
    std::string _mapFrameId;
    size_t _particleCount;
    bool _deleteCones;
    bool _associateSameColorOnly;
    int _observationIncrement;
    float _particleResampleFactor;
    float _loopClosureParticleFactor;
    float _coneRadius;
    
    /* parameters for map loading */
    bool _loadTrack;
    std::string _mapLocation;
    std::string _loadTrackName;   

    nav_msgs::Path _newPath;

    std_msgs::Header _detectionsHeader;

    bool _odometryFirst = true;
    bool _detectionsFirst = true;
    std_msgs::Header _odometryFirstStamp; 
    std_msgs::Header _detectionsFirstStamp;
    const double _lidarToCgOffset = 1.5;
    tf::TransformBroadcaster tf_br_;
};

#endif