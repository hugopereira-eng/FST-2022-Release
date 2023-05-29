#ifndef MISSION_TRACKER_HANDLE_HPP
#define MISSION_TRACKER_HANDLE_HPP

#include "shared/as_lib/common.h"
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include "common_msgs/CarVelocity.h"
#include "common_msgs/CarMotor.h"
#include "common_msgs/ConeDetections.h"
#include "common_msgs/ControlMonitoring.h"
#include "common_msgs/RES.h"
#include "common_msgs/Mission.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"


class MissionTrackerHandle {

public:
    //Constructor
    MissionTrackerHandle(ros::NodeHandle &nodeHandle);

    //Destructor
    ~MissionTrackerHandle();

    //Methods
    void run();

private:
    void printASState(const ros::TimerEvent &);
    //Methods
    /* ROS related methods */
    void advertiseToTopics(); 
    void subscribeToTopics();
    void loadParameters();
    void conesCallback(const common_msgs::ConeDetections &conesDetected);
    void RESCallback(const common_msgs::RES &resState);
    //void velocityCallback(const common_msgs::CarVelocity &vel);
    void velocityCallback(const common_msgs::CarMotor &vel);
    void slamPathCallback(const nav_msgs::Path &slamPath);
    void slamPoseCallback(const nav_msgs::Odometry &slamPose);
    void loopClosureCallback(const std_msgs::Bool &loopClosure);

    /* Methods related to checking if misson is done */
    void checkMissonState();
    bool checkMissionConditions();
    bool checkAllOrange();
    int checkOrangeDistance();
    bool checkPathDistance();
    bool checkAccelerationStoppingDistance();
    bool checkOffsetStoppingDistance();
    bool checkIfInBothSides();
    bool checkLapsNumber();
    bool checkVelocityZero();
    bool checkInspectionEndMission();
    
    /* other methods */
    void publishMissionFinished();
    void controlToggle(bool toggle);
    void requireBraking();
    void requireBraking(const ros::TimerEvent& event);
    void allowNewLapCount(const ros::TimerEvent& event);
    void publishPathPlannerActive();

    
    //attributes
    ros::NodeHandle _nodeHandle;

    ros::Publisher _pubMissionFinished;
    ros::Publisher _pubMonitoringControl;
    ros::Publisher _pubNumberOfLaps;
    ros::Publisher _pubPathPlannerActive;
    ros::Publisher _pubSlamActive;
    ros::Publisher _pubSpeedPlannerActive;
    ros::Publisher _pubInspectionSteering;
    ros::Subscriber _subCones;
    ros::Subscriber _subRes;
    ros::Subscriber _subVelocity;
    ros::Subscriber _subSlamPath;
    ros::Subscriber _subSlamPose;
    ros::Subscriber _subLoopClosure;
    ros::Timer _timerToFinishMission;
    ros::Timer _timerToCountNewLap;
    ros::Timer _timerASState;

    as_states _ASState;

    common_msgs::ConeDetections _coneDetections;
    uint32_t _vel;
    nav_msgs::Path _slamPath;
    nav_msgs::Odometry _slamPose;
    std_msgs::Bool _loopClosure;
    
    int _mission;
    bool _missionFinished = false;
    float _waitTimeToFinishMission;
    float _distanceToFinishMission;
    int _lapsCounter = 0;
    int _maxLaps;
    float _timeToAnotherLap;
    float _orangeConeDistanceThreshold;
    float _pathDistanceThreshold;
    bool _newLapCount = true;
    int _lastPathIndex;
    float _accelerationMaxDistanceThreshold;
    float _offsetFromStartX;
    float _offsetFromStartY;
    float _circuitStoppingDistanceThreshold;
    float _ebsMaximumVelocity;
    double _inspectionMaximumTime;
    bool _inspectionStart = true;
    bool _inspectionActive = false;
    ros::Time _inspectionTimer;
    ros::Time _timerMissionStart;
    double lastElapsedTime = 0;
    double inspectionReference = 0.25;
    bool _vsvBrake = false;
};

#endif // MISSION_TRACKER_HANDLE_HPP