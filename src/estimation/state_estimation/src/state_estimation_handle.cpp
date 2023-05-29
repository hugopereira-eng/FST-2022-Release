#include "state_estimation/state_estimation_handle.hpp"

StateEstimationHandle::StateEstimationHandle(ros::NodeHandle &nodeHandle): _nodeHandle(nodeHandle), _stateEstimation(nodeHandle) {
    advertiseToTopics();
    subscribeToTopics();
}

void StateEstimationHandle::advertiseToTopics(){
    _pubCarVelocity = _nodeHandle.advertise<common_msgs::CarVelocity>("/estimation/state_estimation/velocity", 1);
    _pubCovarianceTrace = _nodeHandle.advertise<std_msgs::Float32>("/estimation/state_estimation/P_trace", 1);
    _pubCarPosition = _nodeHandle.advertise<common_msgs::CarPose>("/estimation/state_estimation/position", 1);
    _pubEstimatedWheelSpeedRear = _nodeHandle.advertise<common_msgs::CarMotor>("/estimation/state_estimation/rear_wheel_speeds", 1);
    _pubEstimatedWheelSpeedFront = _nodeHandle.advertise<common_msgs::CarMotor>("/estimation/state_estimation/front_wheel_speeds", 1);

    //Topics published when debugging the package
    // _pubWheelSpeeds = _nodeHandle.advertise<geometry_msgs::Vector3Stamped>("/estimation/state_estimation/debug/wheelSpeeds", 1);
    // _pubSteering = _nodeHandle.advertise<geometry_msgs::Vector3Stamped>("/estimation/state_estimation/debug/steering", 1);
    // _pubTorque = _nodeHandle.advertise<geometry_msgs::Vector3Stamped>("/estimation/state_estimation/debug/torque", 1);
    // _pubGpsPositionConverted = _nodeHandle.advertise<common_msgs::CarPose>("/estimation/state_estimation/gps_position", 1);
    _pubGpsPositionOdometry = _nodeHandle.advertise<nav_msgs::Odometry>("/estimation/state_estimation/gps_odom", 1);
    // _pubStaSteering = _nodeHandle.advertise<geometry_msgs::Vector3Stamped>("/estimation/state_estimation/debug/sta_steering", 1);
}

void StateEstimationHandle::publishToTopics(){
    _pubCarVelocity.publish(_stateEstimation.getCarVelocity());
    _pubCovarianceTrace.publish(_stateEstimation.getPMatrixTrace());
    _pubCarPosition.publish(_stateEstimation.getCarPosition());
    _pubEstimatedWheelSpeedRear.publish(_stateEstimation.getEstimatedWheelSpeedRear());
    _pubEstimatedWheelSpeedFront.publish(_stateEstimation.getEstimatedWheelSpeedFront());

    //Topics published when debugging the package
    // _pubWheelSpeeds.publish(_stateEstimation.getWheelSpeeds());
    // _pubSteering.publish(_stateEstimation.getSteering());
    // _pubTorque.publish(_stateEstimation.getTorque());
    // _pubGpsPositionConverted.publish(_stateEstimation.getGpsPositionConverted());
    _pubGpsPositionOdometry.publish(_stateEstimation.getGpsPositionOdometry());
    // _pubStaSteering.publish(_stateEstimation.getStaSteering());
}

void StateEstimationHandle::runAlgorithm(){
    _stateEstimation.run();
    publishToTopics();
}

void StateEstimationHandle::subscribeToTopics(){
    _subXsensAccel = _nodeHandle.subscribe("/estimation/odom3D", 1, &StateEstimationHandle::ImuCallback, this);
    _subDashSteering = _nodeHandle.subscribe("/control/controller/steering_actual", 1, &StateEstimationHandle::SteeringCallback, this);
    _subWheelSpeeds = _nodeHandle.subscribe("/estimation/wheel_speeds", 1, &StateEstimationHandle::WheelSpeedCallback, this);
    _subMotorTorque = _nodeHandle.subscribe("/estimation/motor_torque", 1, &StateEstimationHandle::MotorTorqueCallback, this);
    _subGpsPosition = _nodeHandle.subscribe("/estimation/gps_position", 1, &StateEstimationHandle::GpsPositionCallback, this);
    _subGpsVelocity = _nodeHandle.subscribe("/estimation/gps_velocity", 1, &StateEstimationHandle::GpsVelocityCallback, this);
    _subStaSteering = _nodeHandle.subscribe("/common/can_sniffer/sta_position", 1, &StateEstimationHandle::StaSteeringCallback, this);
}

void StateEstimationHandle::ImuCallback(const sensor_msgs::Imu &odom3d){
    _stateEstimation.setImuMeasure(odom3d);
}

void StateEstimationHandle::SteeringCallback(const common_msgs::ControlCmd &steering){
    _stateEstimation.setSteeringMeasure(steering);
}

void StateEstimationHandle::WheelSpeedCallback(const common_msgs::CarMotor &wheel_speeds){
    _stateEstimation.setWheelSpeedsMeasure(wheel_speeds);
}

void StateEstimationHandle::MotorTorqueCallback(const common_msgs::CarMotor &torque){
    _stateEstimation.setTorqueMeasure(torque);
}

void StateEstimationHandle::GpsPositionCallback(const sensor_msgs::NavSatFix &gpsPosition){
    _stateEstimation.setGpsPosition(gpsPosition);
}

void StateEstimationHandle::GpsVelocityCallback(const geometry_msgs::TwistWithCovarianceStamped &gpsVelocity) {
    _stateEstimation.setGpsVelocity(gpsVelocity);
}

void StateEstimationHandle::StaSteeringCallback(const common_msgs::StaPositionInfo &steering) {
    _stateEstimation.setStaSteering(steering);
}