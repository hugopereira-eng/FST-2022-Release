#include "controller/controller_pipeline.hpp"
#define PI 3.14159265359
#include <tf/tf.h>

// Constructor
Controller::Controller() : _pid(0.01, 1, -1, 0.15, 0, 0.01),  _controlState(ControlState::CONTROL_OFF) {
	ROS_INFO("Constructing Controller");
	
	LoadParameters();

	// set PID gains (P,I,D)
	_pid.setPIDParams((1/_controllerfrequency), _accelK,_accelKi,0);

}

/**
*	Name: LoadParameters.
*	Description: Executed in the constructor to load config parameters.
*	Inputs: None.
*	Output: void
*/
void Controller::LoadParameters(){

	if (!ros::param::get("controller/controller_freq", _controllerfrequency)) {
		ROS_WARN_STREAM("Could not load controller/controller_freq. Default value is 100");
		_controllerfrequency = 100;
	}
	if (!ros::param::get("controller/use_adaptive_control", _UseAdaptiveControl)) {
		ROS_WARN_STREAM("Could not load controller/use_adaptive_control. Default value is false");
		_UseAdaptiveControl = false;
	}
	if (!ros::param::get("controller/accelerating_kp", _accelK)) {
		ROS_WARN_STREAM("Could not load controller/accelerating_kp. Default value is 0.03");
		_accelK = 0.03;
	}
	if (!ros::param::get("controller/accelerating_ki", _accelKi)) {
		ROS_WARN_STREAM("Could not load controller/accelerating_ki. Default value is 0.01");
		_accelKi = 0.01;
	}
	if (!ros::param::get("controller/braking_k", _brakeK))	{
		ROS_WARN_STREAM("Could not load controller/braking_k. Default value is 0.03");
		_brakeK = 0.03;
	}
	if (!ros::param::get("controller/steering_k", _steeringK)) {
		ROS_WARN_STREAM("Could not load controller//steering_k. Default value is 2.0");
		_steeringK = 2.0;
	}
	if (!ros::param::get("controller/look_ahead_time", _lookAheadTime)) {
		ROS_WARN_STREAM("Could not load controller/look_ahead_time. Default value is 0.7");
		_lookAheadTime = 0.7;
	}
	if (!ros::param::get("controller/further_look_ahead_time", _furtherLookAheadTime)) {
		ROS_WARN_STREAM("Could not load controller/further_look_ahead_time. Default value is 1.2");
		_lookAheadTime = 1.2;
	}
	if (!ros::param::get("controller/min_lookahead_distance", _minDistanceSetPoint)) {
		ROS_WARN_STREAM("Could not load controller/min_lookahead_distance. Default value is 5");
		_minDistanceSetPoint = 5;
	}
	if (!ros::param::get("controller/speed_angle", _speedAngle)) {
		ROS_WARN_STREAM("Could not load controller/speed_angle. Default value is 30");
		_speedAngle = 30;
	}
	if (!ros::param::get("controller/speed_ref", _speedRef)) {
		ROS_WARN_STREAM("Could not load controller/speed_ref. Default value is 4.0");
		_speedRef = 4.0;
	}
	if (!ros::param::get("controller/max_speed", _maxSpeed)) {
		ROS_WARN_STREAM("Could not load controller/max_speed. Default value is 11.0");
		_maxSpeed = 11.0;
	}
	if (!ros::param::get("controller/min_speed", _minSpeed)) {
		ROS_WARN_STREAM("Could not load controller/min_speed. Default value is 6.0");
		_minSpeed = 6.0;
	}
	if (!ros::param::get("controller/min_steering", _minStaSteering)) {
		ROS_WARN_STREAM("Could not load controller/min_steering. Default value is -24000.0");
		_minStaSteering = -24000.0;
	}
	if (!ros::param::get("controller/max_steering", _maxStaSteering)) {
		ROS_WARN_STREAM("Could not load controller/max_steering. Default value is 24000.0");
		_maxStaSteering = 24000.0;
	}
	if (!ros::param::get("controller/max_steering_change", _maxSteeringChange)) {
		ROS_WARN_STREAM("Could not load controller/max_steering_change. Default value is 0.8");
		_maxSteeringChange = 0.8;
	}
	if (!ros::param::get("controller/steering_filter_k", _steeringFilterK)) {
		ROS_WARN_STREAM("Could not load controller/steering_filter_k. Default value is 0.05");
		_steeringFilterK = 0.05;
	}
	if (!ros::param::get("common/mission_selected", _mission)) {
		ROS_WARN_STREAM("Could not load mission. Trackdrive will be assumed");
		_mission = TRACKDRIVE;
	}

}

/******************************** GETTERS ********************************/

common_msgs::ControlCmd const &Controller::getControlCommand() const { return _controlCmd; }
common_msgs::PointToFollow const &Controller::getPointToFollow() const { return _pointToFollow; }
common_msgs::PointToFollow const &Controller::getPointAhead() const { return _pointAhead; }
visualization_msgs::Marker const &Controller::getMarkerPointToFollow() const { return _markerPointToFollow; }
visualization_msgs::Marker const &Controller::getMarkerPointAhead() const { return _markerPointAhead; }
std_msgs::Float64 const &Controller::getSpeedTarget() const {return _speedTarget; };

bool const Controller::shouldPublish() const { return _publish; }


/******************************** SETTERS ********************************/

void Controller::setPathToFollow(const nav_msgs::Path &path) {
	if (!path.poses.empty()) {
		_pathPlannerPath = path;
		_pathPlannerPath.header = path.header;
	}
}

void Controller::setSlamPath(const nav_msgs::Path &slamPath) {
	if (!slamPath.poses.empty()) {
		_slamPath = slamPath;
	}
}

void Controller::setSlamPose(const nav_msgs::Odometry &slamPose) {
	_carPose = slamPose;
}

void Controller::setInspectionSteering(const std_msgs::Float64 &inspectionSteering) {
	_steeringInspection = inspectionSteering.data;
}

void Controller::changeControllerState(const common_msgs::ControlMonitoring &msg) {

	// ordered by priority
	if (msg.require_braking) {
		_controlState = ControlState::BRAKING_MODE;
		//ROS_INFO("BRAKING REQUIRED!");
	}
	else if (!msg.control_on) {
		_controlState = ControlState::CONTROL_OFF;
		//ROS_INFO("CONTROL IS OFF");
	}
	else if (_mission == INSPECTION) {
		_controlState = ControlState::INSPECTION;
		//ROS_INFO("INSPECTION MODE");
	}
	else {
		_controlState = ControlState::CONTROL_ON;
		//ROS_INFO("CONTROL IS ON");
	}
}

void Controller::setCurrentVelocity(const common_msgs::CarVelocity &vel) {
	_currentVelocity = vel;
}

/**************************** MAIN FUNCTIONS (STATES) ****************************/

/**
*	Name: runAlgorithm.
*	Description: Runs the controller pipeline, executing different algorithms depending on the control state
*	Inputs: Control State
*	Output: void
*/
void Controller::runAlgorithm() {
	switch (_controlState) {
		case ControlState::CONTROL_ON:
			_publish = true;
			purePursuitControlCmd();
			break;
		case ControlState::BRAKING_MODE:
			_publish = true;
			brakingControlCmd();
			break;
		case ControlState::CONTROL_OFF:
			_publish = false;
			break;
		case ControlState::INSPECTION:
			_publish = true;
			inspectionControlCmd();
			break;
	}
}

/**
*	Name: printControlState.
*	Description: Used to print the control state
*	Inputs: Control State
*	Output: void
*/
void Controller::printControlState() {
	switch (_controlState) {
		case ControlState::CONTROL_ON:
			ROS_INFO("ControlState: CONTROL_ON");
			break;
		case ControlState::BRAKING_MODE:
			ROS_INFO("ControlState: BRAKING_MODE");
			break;
		case ControlState::CONTROL_OFF:
			ROS_INFO("ControlState: CONTROL_OFF");
			break;
		case ControlState::INSPECTION:
			ROS_INFO("ControlState: INSPECTION");
			break;
	}
}

/**
*	Name: purePursuitControlCmd.
*	Description: High level implementation of the control algorithm. 
*   			 Decides the priority of which path to follow and how to operate when there is no path.
*	Inputs: The path to be followed.
*	Output: Control Commands (throttle and steering).
*/
void Controller::purePursuitControlCmd() {
	
	// follow slam trajectory
	if (!_slamPath.poses.empty()) {	

		getNearestIndex();
		_pathToFollow = _slamPath;
		followPath();
	}
	// follow path planner
	else if (!_pathPlannerPath.poses.empty()) {

		_carPose.pose.pose.position.x = 0;
		_carPose.pose.pose.position.y = 0;
		_carPose.pose.pose.position.z = 0;
		_carPose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
		_index = 0;

		_pathToFollow = _pathPlannerPath;

		followPath();
	}
	// no path or trajectory to follow - reduce speed and maintain steering angle
	else {
		const double vel = std::hypot(_currentVelocity.velocity.x, _currentVelocity.velocity.y);
		_controlCmd.throttle = static_cast<float>(_brakeK * (_minSpeed - vel));
		_controlCmd.steering_angle = _previousControlCmd.steering_angle;
		_pointToFollow.speed_ref = _minSpeed;
	}
}

/**
*	Name: brakingControlCmd.
*	Description: Sets the control commands in order for the car to brake .
*	Inputs: None.
*	Output: Control Commands (throttle and steering).
*/
void Controller::brakingControlCmd() {
	_controlCmd.throttle = -1;
	_controlCmd.steering_angle = 0;
	_controlCmd.header = _pathToFollow.header;
	_pointToFollow.speed_ref = 0;
}

/**
*	Name: brakingControlCmd.
*	Description: Sets the control commands for the inspection mission. 
*	Inputs: None.
*	Output: Control Commands (throttle and steering) .
*/
void Controller::inspectionControlCmd() {
	
	//const double vel = std::hypot(_currentVelocity.velocity.x, _currentVelocity.velocity.y);
	_controlCmd.header.stamp = ros::Time::now();
	_controlCmd.throttle = 0.9; //static_cast<float>(_accelK * (_minSpeed - vel));
	_controlCmd.steering_angle = _steeringInspection;
	_pointToFollow.speed_ref = _minSpeed;
}


/******************************** AUX FUNCTIONS ********************************/

/**
*	Name: followPath.
*	Description: Given a path, computes the point to follow and corresponding control commands. 
*	Inputs: Path of type nav_msgs::Path.
*	Output: Control Commands (throttle and steering). 
*/
void Controller::followPath() {

	// first point (actual point to follow)
	float distanceSetPoint = getDistance(_lookAheadTime);
	
	//get the point to follow index in the path 
	int iNext = point2Follow(_index, _pathToFollow.poses.size(), distanceSetPoint, _pathToFollow);
	geometry_msgs::PoseStamped nextPoint = _pathToFollow.poses[iNext];
	
	//point to follow visualization
	setMarkerPointToFollow(nextPoint.pose.position.x, nextPoint.pose.position.y, _pathToFollow.header, _pathToFollow.header.frame_id);

	//compute throttle and steering
	computeCommands(nextPoint, _pathToFollow.header);

	float actualDistance = std::hypot(nextPoint.pose.position.y - _carPose.pose.pose.position.y, nextPoint.pose.position.x - _carPose.pose.pose.position.x);
	_pointToFollow.header = _carPose.header;
	_pointToFollow.x =  nextPoint.pose.position.x;
	_pointToFollow.y =  nextPoint.pose.position.y;
	_pointToFollow.distance = actualDistance;
	_pointToFollow.speed_ref = _speedRef;

}

/**
*	Name: computeCommands.
*	Description: Given a point to follow, computes the control commands (steering and throttle).
*	Inputs: Point to follow of type geometry_msgs::PoseStamped, message Header of type std_msgs::Header.
*	Output: Control Commands (throttle and steering) .
*/
void Controller::computeCommands(geometry_msgs::PoseStamped pointToFollow, std_msgs::Header header) {

	_controlCmd.header = header;

	// steering angle calculation (Pure Pursuit)
	const double beta_est = _controlCmd.steering_angle * 0.5;
	double eta = std::atan2(pointToFollow.pose.position.y - _carPose.pose.pose.position.y, pointToFollow.pose.position.x - _carPose.pose.pose.position.x);
	eta = eta - getYawFromQuaternion(_carPose) - beta_est;
	const double length = std::hypot(pointToFollow.pose.position.y - _carPose.pose.pose.position.y, pointToFollow.pose.position.x - _carPose.pose.pose.position.x);
	float steeringAngle = static_cast<float>(_steeringK * std::atan(2.0 / length * std::sin(eta)));

	//limiting steering values using STA limits 
	if(steeringAngle >= _maxStaSteering/81920.0) {
		steeringAngle = _maxStaSteering/81920.0; 
	}
	else if (steeringAngle <= _minStaSteering/81920.0) {
		steeringAngle = _minStaSteering/81920.0;
	}

	// applying a first order filter to the steering command
	if(_filteredSteering == -1) _filteredSteering = steeringAngle;
	_filteredSteering = _filteredSteering + (steeringAngle - _filteredSteering)*_steeringFilterK;
	
	// // Setting the steering angle
	_controlCmd.steering_angle = _filteredSteering;

	// Throttle command computation 
	const double vel = std::hypot(_currentVelocity.velocity.x, _currentVelocity.velocity.y);
	_controlCmd.throttle = _pid.calculate(_speedRef,vel);
	//_controlCmd.throttle = saturateThrottle(static_cast<float>(_accelK * (_speedRef - vel)));
	_pointToFollow.speed_ref = _speedRef;

	//adaptive controller
	if (_UseAdaptiveControl) computeAdaptiveControl();

	//saving previous control cmd for next iteration 
	_previousControlCmd = _controlCmd;
}


/**
*	Name: computeAdaptiveControl.
*	Description: Sets the throttle depending on the tracks curvature.
*	Inputs: further look ahead time parameter.
*	Output: Throttle control command.
*/
void Controller::computeAdaptiveControl(){

	// get the distance of a point given by _FurtherLookAheadTime
	float distanceSetPoint = getDistance(_furtherLookAheadTime);
	
	//get the point's index in the path 
	int iNext = point2Follow(_index, _pathToFollow.poses.size(), distanceSetPoint, _pathToFollow);
	geometry_msgs::PoseStamped nextPoint = _pathToFollow.poses[iNext];

	//point visualization
	setMarkerPointAhead(nextPoint.pose.position.x, nextPoint.pose.position.y, _pathToFollow.header, _pathToFollow.header.frame_id);

	// calculate the angle between the car and the point
	double eta = std::atan2(nextPoint.pose.position.y - _carPose.pose.pose.position.y, nextPoint.pose.position.x - _carPose.pose.pose.position.x);
	eta = eta - getYawFromQuaternion(_carPose);

	const double vel = std::hypot(_currentVelocity.velocity.x, _currentVelocity.velocity.y); 

	double vel_ref = _maxSpeed - (exp(fabs(eta)) - 1) * (_maxSpeed - _minSpeed);
	if (vel_ref < _minSpeed) vel_ref = _minSpeed;
	_controlCmd.throttle = _pid.calculate(vel_ref,vel);
	_pointAhead.speed_ref = vel_ref;

	
	// //if curvature is too high, reduce speed
	// if (fabs(eta) > _speedAngle * M_PI / 180.0 && _mission != ACCELERATION) {
	// 	_controlCmd.throttle = _pid.calculate(_minSpeed, vel);
	// 	//_controlCmd.throttle = saturateThrottle(static_cast<float>(_brakeK *(_minSpeed - vel)));
	// 	_pointAhead.speed_ref = _minSpeed;
	// 	// ROS_WARN("Braking caralhe");
	// }
	// // else jardate
	// else {
	// 	// _controlCmd.throttle = saturateThrottle(static_cast<float>(_accelK * (_maxSpeed - vel)));
	// 	_controlCmd.throttle = _pid.calculate(_maxSpeed, vel);
	// 	_pointAhead.speed_ref = _maxSpeed;
	// 	// ROS_WARN("Jardate pa frente");
	// }

	_pointAhead.header = _carPose.header;
	_pointAhead.x = nextPoint.pose.position.x;
	_pointAhead.y = nextPoint.pose.position.y;
	_pointAhead.eta = eta * 180.0 / M_PI;
	_pointAhead.distance = distanceSetPoint;
}

/**
*	Name: point2Follow.
*	Description: Used to find the path point distanceSetPoint meters away from current position.
*	Inputs: Path to follow.
*	Output: Index of the resulting point in the path array. 
*/
int Controller::point2Follow(int start, int count, float distanceSetPoint, nav_msgs::Path path) {

	int iNext;
	float minDistance = INFINITY, distance;

	/** look for the trajectory point that is @param distanceSetPoint meters aways from our current position */
	for (int i = start, x = 0; x < count; i++, x++){	
		if (i == path.poses.size()) i = 0;
		
		int dx = path.poses[start].pose.position.x - path.poses[i].pose.position.x;
		int dy = path.poses[start].pose.position.y - path.poses[i].pose.position.y;
		distance = fabs(std::hypot(dx, dy) - distanceSetPoint);

		if (distance < minDistance){
			minDistance = distance;
			iNext = i;
		}else if(distance > minDistance + 1){
			break;
		}
	}

	return iNext;
}

/**
*	Name: getDistance.
*	Description: Computes the distance to the point to follow from the _lookAheadTime parameter.
*	Inputs: _lookAheadTime, current velocity. 
*	Output: Distance to the point to follow.
*/
float Controller::getDistance( double lookAheadTime ) {

	double currentSpeed = std::hypot(_currentVelocity.velocity.x, _currentVelocity.velocity.y);
	float distanceSetPoint; 
    distanceSetPoint = currentSpeed * lookAheadTime;

	// if the distance is lower than minimum or speed = 0, set it to minimum
	if (currentSpeed == 0 || distanceSetPoint < _minDistanceSetPoint) distanceSetPoint = _minDistanceSetPoint;

	return distanceSetPoint;
}

/**
*	Name: getNearestIndex.
*	Description: Looks for the point in the path closest to the current position.
*	Inputs: Path and car's pose.
*	Output: closes index to the car's pose.
*/
void Controller::getNearestIndex(){

	float distance, minDistance = INFINITY;

	// look for the trajectory point that is closest to our current position
	for (int i = _index, x = 0; x < _slamPath.poses.size() ; i++, x++) {
		if (i == _slamPath.poses.size()) i = 0;
		int dx = _carPose.pose.pose.position.x - _slamPath.poses[i].pose.position.x;
		int dy = _carPose.pose.pose.position.y - _slamPath.poses[i].pose.position.y;
		distance = hypot(dx, dy);
		
		if (distance < minDistance) {
			_index = i;
			minDistance = distance;

		// if we are going away from the closest point, stop searching
		}else if(distance > minDistance + 1){
			break;
		}

	}
	
}


/**
*	Name: getYawFromQuaternion.
*	Description: Obtain (roll, pitch,yaw) values from quaternion.
*	Inputs: Car's pose of type  nav_msgs::Odometry.
*	Output: yaw.
*/
double Controller::getYawFromQuaternion(nav_msgs::Odometry pose) {
	
	double roll, pitch, yaw;

	tf::Quaternion q(
		pose.pose.pose.orientation.x,
		pose.pose.pose.orientation.y,
		pose.pose.pose.orientation.z,
		pose.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);

	m.getRPY(roll, pitch, yaw);
	return yaw;
}

/**
*	Name: saturateThrottle.
*	Description: Saturates the throttle values to the [-1,1] interval.
*	Inputs: throttle.
*	Output: saturated throttle.
*/
float Controller::saturateThrottle(float _throttle){

	if(_throttle > 1) _throttle = 1;
	else if (_throttle < -1) _throttle = -1;

	return _throttle;
}



/******************************** VISUALIZATION ********************************/

/**
*	Name: setMarkerPointToFollow()
*	Description: Used to visualize the Point to Follow in Rviz 
*	Inputs: Point's position, header and frameID.
*	Output: Void.
*/
void Controller::setMarkerPointToFollow(double x_pos, double y_pos, std_msgs::Header header, std::string frameID) {

	visualization_msgs::Marker marker;

	marker.color.b = 1.0;
	marker.color.a = 1.0;
	marker.pose.position.x = x_pos;
	marker.pose.position.y = y_pos;
	marker.pose.orientation.w = 1.0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.id = 0;
	marker.scale.x = 0.4;
	marker.scale.y = 0.4;
	marker.scale.z = 0.4;
	marker.header = header;
	marker.header.frame_id = frameID;
	marker.lifetime = ros::Duration(0.05);

	_markerPointToFollow = marker;
}


/**
*	Name: setMarkerPointToAhead()
*	Description: Used to visualize the Point Ahead in Rviz 
*	Inputs: Point's position, header and frameID.
*	Output: Void.
*/
void Controller::setMarkerPointAhead(double x_pos, double y_pos, std_msgs::Header header, std::string frameID) {

	visualization_msgs::Marker marker;

	marker.color.r = 1.0;
	marker.color.a = 1.0;
	marker.pose.position.x = x_pos;
	marker.pose.position.y = y_pos;
	marker.pose.orientation.w = 1.0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.id = 0;
	marker.scale.x = 0.4;
	marker.scale.y = 0.4;
	marker.scale.z = 0.4;
	marker.header = header;
	marker.header.frame_id = frameID;
	marker.lifetime = ros::Duration(0.05);

	_markerPointAhead = marker;
}
