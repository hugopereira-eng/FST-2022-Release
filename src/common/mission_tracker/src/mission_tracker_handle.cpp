#include "mission_tracker/mission_tracker_handle.hpp"

// Constructor
MissionTrackerHandle::MissionTrackerHandle(ros::NodeHandle &nodeHandle): 
                    _nodeHandle(nodeHandle),
                    _ASState(AS_READY) {
    
    ROS_INFO("Constructing Mission Tracker Handle");
    
    std::cout << R"(

      ____________  _________   __________         _____    ____  ____  ___  ____________
     /_  __/  _/  |/  / ____/  /_  __/ __ \       / /   |  / __ \/ __ \/   |/_  __/ ____/
      / /  / // /|_/ / __/      / / / / / /  __  / / /| | / /_/ / / / / /| | / / / __/   
     / / _/ // /  / / /___     / / / /_/ /  / /_/ / ___ |/ _, _/ /_/ / ___ |/ / / /___   
    /_/ /___/_/  /_/_____/    /_/  \____/   \____/_/  |_/_/ |_/_____/_/  |_/_/ /_____/   

    )" << std::endl;
    
    advertiseToTopics();
    subscribeToTopics();
    loadParameters();
    _timerASState = nodeHandle.createTimer(ros::Duration(0.5), &MissionTrackerHandle::printASState, this, false);
}

// Destructor
MissionTrackerHandle::~MissionTrackerHandle() {}

/* *********************
 * ros related methods *
 * ********************/

void MissionTrackerHandle::advertiseToTopics() {
    ROS_INFO("Mission Handler is advertising to topics");
    _pubMissionFinished = _nodeHandle.advertise<common_msgs::Mission>("/common/mission_tracker/mission_finished", 1);
    _pubMonitoringControl = _nodeHandle.advertise<common_msgs::ControlMonitoring>("/common/mission_tracker/control_monitoring", 1);
    _pubNumberOfLaps = _nodeHandle.advertise<std_msgs::Int16>("/common/mission_tracker/lap_counter", 1);
    _pubPathPlannerActive = _nodeHandle.advertise<std_msgs::Bool>("/common/mission_tracker/path_planner_active", 1);
    _pubInspectionSteering = _nodeHandle.advertise<std_msgs::Float64>("/common/mission_tracker/inspection_steering", 1);
}

void MissionTrackerHandle::subscribeToTopics() {
    ROS_INFO("Mission Handler is subscribing to topics");
    _subCones = _nodeHandle.subscribe("/perception/cone_detections", 1, &MissionTrackerHandle::conesCallback, this);
    _subRes = _nodeHandle.subscribe("/common/res_state", 1, &MissionTrackerHandle::RESCallback, this);
    
    //_subVelocity = _nodeHandle.subscribe("/estimation/velocity", 1, &MissionTrackerHandle::velocityCallback, this);
    _subVelocity = _nodeHandle.subscribe("/estimation/wheel_speeds", 1, &MissionTrackerHandle::velocityCallback, this);

    _subSlamPath = _nodeHandle.subscribe("/estimation/slam/centerline", 1, &MissionTrackerHandle::slamPathCallback, this);
    _subSlamPose = _nodeHandle.subscribe("/estimation/slam/odometry", 1, &MissionTrackerHandle::slamPoseCallback, this);
    _subLoopClosure = _nodeHandle.subscribe("/estimation/slam/loop_closure", 1, &MissionTrackerHandle::loopClosureCallback, this);
}

void MissionTrackerHandle::loadParameters() {
    if(!ros::param::get("common/mission_selected", _mission)) {
        ROS_WARN(" no mission provided, trackdrive will be assumed ");
        _mission = TRACKDRIVE;
        /* the problem of not having a mission should be take into account
         * bad approach maybe. Other thing should be attempy when dealing with this problem.
         */
    }
    _distanceToFinishMission = _nodeHandle.param<float>("distance_until_mission_finished", 0);
    _timeToAnotherLap = _nodeHandle.param<float>("time_to_another_lap", 0);
    _maxLaps = _nodeHandle.param<int>("number_of_laps", 1); 
    _orangeConeDistanceThreshold = _nodeHandle.param<float>("distance_to_orange_cone", 1);
    _pathDistanceThreshold = _nodeHandle.param<float>("distance_to_last_path_point", 1);
    _accelerationMaxDistanceThreshold = _nodeHandle.param<float>("acceleration_maximum_distance",1);
    _offsetFromStartX = _nodeHandle.param<float>("offset_from_start_x",1);
    _offsetFromStartY = _nodeHandle.param<float>("offset_from_start_y",1);
    _circuitStoppingDistanceThreshold = _nodeHandle.param<float>("maximum_distance_from_offset_position",2);
    _ebsMaximumVelocity = _nodeHandle.param<float>("ebs_maximum_velocity", 1);
    _inspectionMaximumTime = _nodeHandle.param<double>("inspection_maximum_time", 1);
}

void MissionTrackerHandle::run() {
    
    checkMissonState();
}

void MissionTrackerHandle::printASState(const ros::TimerEvent &event) {

    switch (_ASState)
    {
    case AS_READY:
        ROS_INFO("Mission Tracker: AS_READY");
        break;
    case AS_DRIVING:
        ROS_INFO("Mission Tracker: AS_DRIVING");
        break;
    case AS_FINISHED:
        ROS_INFO("Mission Tracker: AS_FINISHED");
        break;
    case AS_EMERGENCY:
        ROS_INFO("Mission Tracker: AS_EMERGENCY");
        break;
    default:
        break;
    }
}

void MissionTrackerHandle::conesCallback(const common_msgs::ConeDetections &conesDetected) {
   _coneDetections = conesDetected;
}

void MissionTrackerHandle::RESCallback(const common_msgs::RES &resState) {
    
    if(resState.emergency) {
        /* this if may seem bad coding but this is due to a simulator bug where they send RES EMERGENCY also if the mission finished  */
        if(_ASState != AS_FINISHED) {
            _ASState = AS_EMERGENCY;
            ROS_INFO("RECEIVED EMERGENCY");
            requireBraking();
        }
        return;
    }
    else if(_ASState == AS_READY && resState.push_button) {
        _ASState = AS_DRIVING;
        _timerMissionStart = ros::Time::now();
        controlToggle(true);
        ROS_INFO("Received RES GO, turning On the Control" );
        return;
    }
    else if(_ASState == AS_DRIVING && resState.vsv_brake && !_vsvBrake) {
        _vsvBrake = true;
        ROS_INFO("VSV BRAKE");
        requireBraking();
        return;
    }
    else if(_ASState == AS_DRIVING && !resState.vsv_brake && _vsvBrake) {
        controlToggle(true);
        _vsvBrake = false;   
    }
}

//void MissionTrackerHandle::velocityCallback(const common_msgs::CarVelocity &vel) {
void MissionTrackerHandle::velocityCallback(const common_msgs::CarMotor &vel) {
    //_vel = vel.velocity.x;
    _vel = vel.value2;
}

void MissionTrackerHandle::slamPathCallback(const nav_msgs::Path &slamPath) {
	_slamPath = slamPath;
	_lastPathIndex = _slamPath.poses.size() - 1;
}

void MissionTrackerHandle::slamPoseCallback(const nav_msgs::Odometry &slamPose) {
    _slamPose = slamPose;
}

void MissionTrackerHandle::loopClosureCallback(const std_msgs::Bool &loopClosure) {
    _loopClosure = loopClosure;
}

/* ***********************************************
 * Methods related to checking if misson is done *
 * **********************************************/

void MissionTrackerHandle::checkMissonState() {

    switch (_ASState) {
        case  AS_READY:
            break;
        case  AS_DRIVING:
            if(!_missionFinished) {
                if (checkMissionConditions()) {
                    //float speed = _vel;
                    float speed = (_vel/16.25/60)*2*M_PI*0.228;
                    if (_mission == INSPECTION) _waitTimeToFinishMission = 0;
                    else _waitTimeToFinishMission = (_distanceToFinishMission)/speed;
                    _timerToFinishMission = _nodeHandle.createTimer(ros::Duration(_waitTimeToFinishMission), 
                        &MissionTrackerHandle::requireBraking, this, true);
                    _missionFinished = true;
                }
                publishPathPlannerActive();
            }
            else {
                if(checkVelocityZero() && (ros::Time::now().toSec() - _timerMissionStart.toSec() > 3.0)) {
                     _ASState = AS_FINISHED;
                    publishMissionFinished();
                    controlToggle(false);
                }
            }
            break;
        case  AS_FINISHED:
            break;
        case  AS_EMERGENCY:
            if(checkVelocityZero()) {
                controlToggle(false);
            }
    }
}

bool MissionTrackerHandle::checkMissionConditions() {
    /* acceleration */
    if(_mission == ACCELERATION || _mission == EBS_TEST) {
        return (checkAccelerationStoppingDistance());
    }
    else if (_mission == AUTOCROSS || _mission == TRACKDRIVE) {
        if(_newLapCount) {
            if(checkOffsetStoppingDistance()){
                return checkLapsNumber();
            }
        }
        return false;
    }
    else if (_mission == SKIDPAD) {
		return (checkAllOrange() && checkPathDistance());
    }
    else if (_mission == INSPECTION) {
        if (_inspectionStart) {
            _inspectionTimer = ros::Time::now();
            _inspectionStart = false;
        }
        return checkInspectionEndMission();
    }
    return false;
}

bool MissionTrackerHandle::checkInspectionEndMission() {

    double elapsedTime = static_cast<double>(ros::Time::now().toSec() -_inspectionTimer.toSec());

    // if (elapsedTime - lastElapsedTime >= 1){
    if (elapsedTime <= _inspectionMaximumTime) {
  
        std_msgs::Float64 steeringData;
        steeringData.data = 0.25*sin(elapsedTime);
        // inspectionReference = -inspectionReference;
        // steeringData.data = inspectionReference;
        _pubInspectionSteering.publish(steeringData);
        return false;
        
    }
    return true;
}

bool MissionTrackerHandle::checkPathDistance() {
    float distance = hypot(_slamPose.pose.pose.position.x - _slamPath.poses[_lastPathIndex].pose.position.x, _slamPose.pose.pose.position.y - _slamPath.poses[_lastPathIndex].pose.position.y);
    if(distance <= _pathDistanceThreshold){
    	return true;
    }
    return false;
}

bool MissionTrackerHandle::checkAccelerationStoppingDistance() {
	if ((_slamPose.pose.pose.position.x != 0 && _slamPose.pose.pose.position.y != 0))
    {
        float distance = hypot(_slamPose.pose.pose.position.x, _slamPose.pose.pose.position.y);
        if(distance >= _accelerationMaxDistanceThreshold){
            return true;
        }
    }
    return false;
}

// Checks if the car is with a radius (float distance) from a known point in the map where it must stop (_offsetFromStart)
bool MissionTrackerHandle::checkOffsetStoppingDistance() {
    float distance = hypot(_slamPose.pose.pose.position.x - _offsetFromStartX, _slamPose.pose.pose.position.y - _offsetFromStartY);
    if(distance <= _circuitStoppingDistanceThreshold){
    	return true;
    }   
    
    return false;
}   

bool MissionTrackerHandle::checkAllOrange() {
    return _coneDetections.cone_detections.size() > 0 &&
        (std::all_of(_coneDetections.cone_detections.begin(), _coneDetections.cone_detections.end(), 
        [] (common_msgs::Cone cone) {
            return cone.color == ORANGE_CONE || cone.color == BIG_ORANGE_CONE || cone.color == UNKNOWN_CONE;
        }) && !std::all_of(_coneDetections.cone_detections.begin(), _coneDetections.cone_detections.end(), 
        [] (common_msgs::Cone cone) {
            return cone.color == UNKNOWN_CONE;
        }));  
}

int MissionTrackerHandle::checkOrangeDistance() {
    int count = 0;
    for(const auto &cone : _coneDetections.cone_detections) {
        if(cone.color == ORANGE_CONE || cone.color == BIG_ORANGE_CONE) {
            if(cone.position.x < _orangeConeDistanceThreshold) {
                count++;  
            }  
        }
    }
    return count;
}

bool MissionTrackerHandle::checkIfInBothSides() {
    return std::any_of(_coneDetections.cone_detections.begin(), _coneDetections.cone_detections.end(), 
            [] (common_msgs::Cone cone) {
                return (cone.color == ORANGE_CONE || cone.color == BIG_ORANGE_CONE) && cone.position.y > 0;
            }) && std::any_of(_coneDetections.cone_detections.begin(), _coneDetections.cone_detections.end(), 
            [] (common_msgs::Cone cone) {
                return (cone.color == ORANGE_CONE || cone.color == BIG_ORANGE_CONE) && cone.position.y < 0;
            });
}

bool MissionTrackerHandle::checkLapsNumber() {

    /* before updating the lap number or signal that all laos were completed,
     * publishes to the automatic tests how many laps where successfully done 
     */ 
    std_msgs::Int16 nrLapsMsg;
    nrLapsMsg.data = _lapsCounter;
    _pubNumberOfLaps.publish(nrLapsMsg);
    
    if(_lapsCounter == _maxLaps) {
        return true;
    }
    else {
        _lapsCounter++;
        std::cout << "Lap: " << _lapsCounter << std::endl;
        _newLapCount = false;
        _timerToCountNewLap = _nodeHandle.createTimer(ros::Duration(_timeToAnotherLap), &MissionTrackerHandle::allowNewLapCount, this, true);  
    }
    return false;
}

bool MissionTrackerHandle::checkVelocityZero() {
    return _vel == 0;
}

/*****************
 * Other Methods *
 * **************/

void MissionTrackerHandle::publishMissionFinished() {
    common_msgs::Mission mission;
    mission.finished = true;
    _pubMissionFinished.publish(mission);
    ROS_INFO("sent misson finished");
}

void MissionTrackerHandle::controlToggle(bool toggle) {
    common_msgs::ControlMonitoring msg;
    msg.control_on = toggle;
    msg.require_braking = false;
    _pubMonitoringControl.publish(msg);
}

void MissionTrackerHandle::requireBraking() {
    common_msgs::ControlMonitoring msg;
    msg.require_braking = true;
    _pubMonitoringControl.publish(msg);
}

void MissionTrackerHandle::requireBraking(const ros::TimerEvent& event) {
    requireBraking();
}

void MissionTrackerHandle::allowNewLapCount(const ros::TimerEvent& event) {
    _newLapCount = true;
}

/* Deactivates the path planner after one lap
 */
void MissionTrackerHandle::publishPathPlannerActive() {
    std_msgs::Bool pathPlannerActive;

    if(_loopClosure.data == 1) {
        pathPlannerActive.data = false;
    }
    else {
        pathPlannerActive.data = true;
    }
    _pubPathPlannerActive.publish(pathPlannerActive);
}
