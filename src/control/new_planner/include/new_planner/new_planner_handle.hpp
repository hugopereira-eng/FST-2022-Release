#ifndef NEW_PLANNER_HANDLE_HPP
#define NEW_PLANNER_HANDLE_HPP

#include <ros/ros.h>
#include <vector>
#include "new_planner/new_planner_particle.hpp"

#include <tf/transform_datatypes.h>
#include "common_msgs/ConeDetections.h"
#include "common_msgs/Cone.h"
#include "nav_msgs/Path.h"

class NewPlannerHandle {

public: 
    // Constructor
    NewPlannerHandle(ros::NodeHandle &nodeHandle);

    //Methods
    void advertiseToTopics(); 
    void subscribeToTopics();
    void publishToTopics();
    void loadParameters();
    void run();

private: 
    ros::NodeHandle _nodeHandle;
  
    ros::Subscriber _subCones;

    ros::Publisher _pubPath;

    common_msgs::ConeDetections _cones;
    nav_msgs::Path _path;

    std::vector<Particle> _particles;

    int _particleCount;
    float _distributionAngle;
    float _distributionRadius;
    float _searchRegionRadius;
    int _stepCount;

    void coneDetections(const common_msgs::ConeDetections &conesDetected);

    void computePath();
    bool coneInRegion(geometry_msgs::Pose pose, common_msgs::Cone cone);
};

#endif 