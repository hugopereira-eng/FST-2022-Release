#include "state_estimation/state_estimation_pipeline.hpp"

StateEstimation::StateEstimation(ros::NodeHandle &nh) {
    loadCarParameters(nh);
    loadEkfParameters(nh);
    
    _kalmanFilter.loadParameters(_params);

    // Parameters to estimate position with GPS

    //Start position 6 meters behind the line
    _position.setZero();

    _JApos = Eigen::Matrix3d::Identity();
    _JHpos = Eigen::Matrix3d::Identity();

    _gpsCoordinates.setZero();
    _gpsCoordinatesConverted.setZero();
    _gpsCoordTransformed.setZero();
}

void StateEstimation::run() {
    _kalmanFilter.update();
}

void StateEstimation::loadCarParameters(ros::NodeHandle &nh) {

    //Load Inercia Params
    _params.inercia.m = nh.param<double>("car/inertia/m", 0); 
    _params.inercia.g = nh.param<double>("car/inertia/g", 0);
    _params.inercia.Iz = nh.param<double>("car/inertia/Iz", 0);
    
    //Load Kinematics Params
    _params.kinematics.track = nh.param<double>("car/kinematics/track", 0);
    _params.kinematics.a = nh.param<double>("car/kinematics/a", 0);
    _params.kinematics.b = nh.param<double>("car/kinematics/b", 0);

    //Load Engine Params
    _params.engine.gr = nh.param<double>("car/engine/gr", 0);

    //Load Tire Params equal for 4 tires
    Tire tire;
    tire.sigma(0,0) = nh.param<double>("car/tire/sigma_00", 0);
    tire.sigma(0,1) = nh.param<double>("car/tire/sigma_01", 0);
    tire.sigma(1,0) = nh.param<double>("car/tire/sigma_10", 0);
    tire.sigma(1,1) = nh.param<double>("car/tire/sigma_11", 0);
    tire.sigma(2,0) = nh.param<double>("car/tire/sigma_20", 0);
    tire.sigma(2,1) = nh.param<double>("car/tire/sigma_21", 0);

    tire.mu_k(0,0) = nh.param<double>("car/tire/mu_k_00", 0);
    tire.mu_k(0,1) = nh.param<double>("car/tire/mu_k_01", 0);
    tire.mu_k(1,0) = nh.param<double>("car/tire/mu_k_10", 0);
    tire.mu_k(1,1) = nh.param<double>("car/tire/mu_k_11", 0);

    tire.mu_s(0,0) = nh.param<double>("car/tire/mu_s_00", 0);
    tire.mu_s(0,1) = nh.param<double>("car/tire/mu_s_01", 0);
    tire.mu_s(1,0) = nh.param<double>("car/tire/mu_s_10", 0);
    tire.mu_s(1,1) = nh.param<double>("car/tire/mu_s_11", 0);

    tire.gamma = nh.param<double>("car/tire/gamma", 0);
    tire.vs = nh.param<double>("car/tire/vs", 0);
    tire.kx = nh.param<double>("car/tire/kx", 0);
    tire.ky = nh.param<double>("car/tire/ky", 0);
    tire.kz = nh.param<double>("car/tire/kz", 0);
    tire.r = nh.param<double>("car/tire/r", 0);

    //fl tire
    tire.p << _params.kinematics.a, _params.kinematics.track/2.0;
    tire.J = nh.param<double>("car/tire/J_f", 0);
    _params.tireArray.push_back(tire);
    //fr tire
    tire.p << _params.kinematics.a, -_params.kinematics.track/2.0;
    tire.J = nh.param<double>("car/tire/J_f", 0);
    _params.tireArray.push_back(tire);
    //rl tire
    tire.p << -_params.kinematics.b, _params.kinematics.track/2.0;
    tire.J = nh.param<double>("car/tire/J_r", 0);
    _params.tireArray.push_back(tire);
    //rr tire
    tire.p << -_params.kinematics.b, -_params.kinematics.track/2.0;
    tire.J = nh.param<double>("car/tire/J_r", 0);
    _params.tireArray.push_back(tire);

    //Load Fn array
    _params.Fn(0) = _params.kinematics.a/(_params.kinematics.a+_params.kinematics.b)*_params.inercia.m*_params.inercia.g/2.0;
    _params.Fn(1) = _params.kinematics.a/(_params.kinematics.a+_params.kinematics.b)*_params.inercia.m*_params.inercia.g/2.0;
    _params.Fn(2) = _params.kinematics.b/(_params.kinematics.a+_params.kinematics.b)*_params.inercia.m*_params.inercia.g/2.0;
    _params.Fn(3) = _params.kinematics.b/(_params.kinematics.a+_params.kinematics.b)*_params.inercia.m*_params.inercia.g/2.0;

    //Load gps coordinates in the car frame
    _gpsCoordOffset(0) = nh.param<double>("car/gps/coord_x", 0);
    _gpsCoordOffset(1) = nh.param<double>("car/gps/coord_y", 0);

    ROS_INFO("Loaded car parameters");
}

void StateEstimation::loadEkfParameters(ros::NodeHandle &nh) {

    XmlRpc::XmlRpcValue covarConfig;
    Eigen::MatrixXd P, Q, R;

    //Load Lugre EKF Config
    nh.getParam("/state_estimation/lugre_ekf/initial_estimate_covariance", covarConfig);
    convertXMLRPCVector(P, covarConfig, 15);

    nh.getParam("/state_estimation/lugre_ekf/process_covariance", covarConfig);
    convertXMLRPCVector(Q, covarConfig, 15);

    nh.getParam("/state_estimation/lugre_ekf/measurement_covariance", covarConfig);
    convertXMLRPCVector(R, covarConfig, 7);

    //Parse the matrices to kalman object
    _kalmanFilter.loadCovarianceMatrices(P,Q,R);


    //Load GPS EKF Config
    _useGPS = nh.param<bool>("gps_ekf/use_gps", true);

    nh.getParam("/state_estimation/gps_ekf/initial_estimate_covariance", covarConfig);
    convertXMLRPCVector(P, covarConfig, 3);

    nh.getParam("/state_estimation/gps_ekf/process_covariance", covarConfig);
    convertXMLRPCVector(Q, covarConfig, 3);

    nh.getParam("/state_estimation/gps_ekf/measurement_covariance", covarConfig);
    convertXMLRPCVector(R, covarConfig, 3);

    _Ppos = P; _Qpos = Q; _Rpos = R;
}

void StateEstimation::convertGpsCoordinates() {
    constexpr double EARTH_RADIUS = 6378388.0;

    double arc = 2.0*M_PI*(EARTH_RADIUS+_gpsCoordinates(2))/360.0;

    if (_initialYaw != 0) {
        _gpsCoordinatesConverted(0) = arc*cosf(_gpsCoordinates(0)*M_PI/180.0)*(_gpsCoordinates(1)) - arc*cosf(_gpsInitial(0)*M_PI/180.0)*(_gpsInitial(1));
        _gpsCoordinatesConverted(1) = arc*(_gpsCoordinates(0)) - arc*(_gpsInitial(0));
    
        Eigen::Matrix<double,2,2> rotation;
        float angle = _initialYaw;
        rotation << cosf(angle), sinf(angle),
                    -sinf(angle), cosf(angle);

        _gpsCoordinatesConverted.block<2,1>(0,0) = rotation * _gpsCoordinatesConverted.block<2,1>(0,0);
        _gpsCoordinatesConverted(2) = clampAnglePi2Pi(_currentXsensYaw - _initialYaw);
    }

    //Add a transform to transform the position in the gps frame to cog frame
    transformGpsToCG();
}

void StateEstimation::estimatePosition() {

    //Prediciton from the velocity

    //Another EKF
    Eigen::Matrix<double,2,2> rotation;  

    const double dt = _kalmanFilter._currentTimeStamp.toSec() - _lastUpdateTime.toSec();
    _lastUpdateTime = _kalmanFilter._currentTimeStamp;
    _lastPosition = _position;

    //Prediction
    //Get Heading

    _position(2) += _wz*dt;
    
    rotation << cosf(_position(2)), -sinf(_position(2)),
                sinf(_position(2)), cosf(_position(2));

    Eigen::Matrix<double,2,1> vel = rotation*Eigen::Matrix<double,2,1>(_x(12), _x(13)); 

    _position(0) += vel(0)*dt;
    _position(1) += vel(1)*dt;

    //Calculate Jacobian of x
    _JApos(1,2) = cosf((-_x(14))*dt + _position(2))-sinf((-_x(14))*dt + _position(2));
    _JApos(0,2) = -sinf((-_x(14))*dt + _position(2))-cosf((-_x(14))*dt + _position(2));

    _Ppos = _JApos*_Ppos*_JApos.transpose() + _Qpos;

    if(_gpsCoordinates.isZero() || _useGPS == false || _initialYaw == 0) {
        _JHpos.setZero();
    } else {
        _JHpos = Eigen::Matrix3d::Identity();
    }

    //Update
    Eigen::MatrixXd S = _JHpos*_Ppos*_JHpos.transpose() + _Rpos;
    Eigen::MatrixXd K = _Ppos*_JHpos.transpose()*S.inverse();

    Eigen::MatrixXd y = _gpsCoordTransformed-_position;

    _position = _position + K*y;
    _distanceIncrement += hypot(_position.x()-_lastPosition.x(), _position.y()-_lastPosition.y());

    _Ppos = (Eigen::Matrix3d::Identity()-K*_JHpos)*_Ppos;
    _position(2) = clampAnglePi2Pi(_position(2));
}

//Setters
void StateEstimation::setImuMeasure(const sensor_msgs::Imu &odom3d) {
    _kalmanFilter.updateInertia(odom3d.linear_acceleration.x, odom3d.linear_acceleration.y, odom3d.angular_velocity.z);
    _wz = odom3d.angular_velocity.z;
    _ahrsFrameId = odom3d.header.frame_id;

    tf2::Quaternion q(odom3d.orientation.x, odom3d.orientation.y, odom3d.orientation.z, odom3d.orientation.w);
    tf2::Matrix3x3 m(q);
    Eigen::Matrix3d _rotationMatrixXsens;
    Eigen::Vector3d acc(0,0,0);
    double Roll, Pitch, Yaw;

    m.getRPY(Roll, Pitch, Yaw);

    if (_initialYaw == 0 && _distanceIncrement >= 7.0) {
        _initialYaw = Yaw;
    }
    _currentXsensYaw = Yaw;    
}

void StateEstimation::setWheelSpeedsMeasure(const common_msgs::CarMotor &wheel_speeds){

    _velocityFrameId = wheel_speeds.header.frame_id;

    if(wheel_speeds.value2 < 30000 ) {
        double w_rl = wheel_speeds.value2*M_PI/30.0/_params.engine.gr;
        _kalmanFilter.updateWheelSpeedsRL(w_rl);
        //DEBUG
        _wheelSpeedsVector(0) = w_rl;
    }
    if (wheel_speeds.value3 < 30000) {
        double w_rr = wheel_speeds.value3*M_PI/30.0/_params.engine.gr;
        _kalmanFilter.updateWheelSpeedsRR(w_rr);
        //DEBUG
        _wheelSpeedsVector(1) = w_rr;
    }
}

void StateEstimation::setTorqueMeasure(const common_msgs::CarMotor &torque){
    double u_rl = torque.value2*_params.engine.gr*0.001*M_PI/2.0;
    double u_rr = torque.value3*_params.engine.gr*0.001*M_PI/2.0;
    _kalmanFilter.updateMotorTorque(u_rl,u_rr);
    //DEBUG
    _torqueVector(0) = u_rl;
    _torqueVector(1) = u_rr;
}

void StateEstimation::setSteeringMeasure(const common_msgs::ControlCmd &steering){
    double wheel_st = steering.steering_angle*M_PI/6.0;
    double steering_fr = atanf(1.0/(1.0/(tanf(wheel_st)) + (_params.kinematics.track/2.0)/(_params.kinematics.a + _params.kinematics.b)));
    double steering_fl = atanf(1.0/(1.0/(tanf(wheel_st)) - (_params.kinematics.track/2.0)/(_params.kinematics.a + _params.kinematics.b)));
    // _kalmanFilter.updateSteeringAngle(steering_fl,steering_fr);
    //DEBUG
    _steeringVector(0) = steering_fl;
    _steeringVector(1) = steering_fr;
}

void StateEstimation::setGpsPosition(const sensor_msgs::NavSatFix &gpsPosition) {
    _gpsCoordinates.setZero();
    _gpsCoordinates(0) = gpsPosition.latitude;
    _gpsCoordinates(1) = gpsPosition.longitude;
    _gpsCoordinates(2) = gpsPosition.altitude;
    
    if (_init) {
        _init = false;
        _gpsInitial(0) = _gpsCoordinates(0);
        _gpsInitial(1) = _gpsCoordinates(1);
    }
    convertGpsCoordinates();
}

void StateEstimation::setGpsVelocity(const geometry_msgs::TwistWithCovarianceStamped &gpsVelocity) {
    //Xsens Vy seems to be inverted in relation to the YR
    _kalmanFilter.updateGpsVelocity(gpsVelocity.twist.twist.linear.x, -gpsVelocity.twist.twist.linear.y);
}

void StateEstimation::setStaSteering(const common_msgs::StaPositionInfo &steering) {
    
    //Sta Steering is inverted in relation to the dash,
    //If the signal is not inverted, vy gives poor results

    double wheel_st = -steering.steeringEncoderSTA*M_PI/180.0/6.0;
    double steering_fr = atanf(1.0/(1.0/(tanf(wheel_st)) + (_params.kinematics.track/2.0)/(_params.kinematics.a + _params.kinematics.b)));
    double steering_fl = atanf(1.0/(1.0/(tanf(wheel_st)) - (_params.kinematics.track/2.0)/(_params.kinematics.a + _params.kinematics.b)));
    _kalmanFilter.updateSteeringAngle(steering_fl,steering_fr);
    // DEBUG
    _staSteeringVector(0) = steering_fl;
    _staSteeringVector(1) = steering_fr;
}

//Getters
common_msgs::CarVelocity StateEstimation::getCarVelocity() {

    //Update State Vector
    _x = _kalmanFilter.getStateVector();

    //Create Velocity Message
    _carVelocity.header.stamp = ros::Time::now();
    _carVelocity.header.frame_id = _velocityFrameId;
    _carVelocity.velocity.x = _x(12);
    _carVelocity.velocity.y = _x(13);
    _carVelocity.velocity.theta = _wz; //-_x(14) to use estimated yaw (increase Iz to get better estimate)
    return _carVelocity; 
}

common_msgs::CarPose StateEstimation::getCarPosition() {

    //Obtain position from Velocity and GPS
    estimatePosition();

    _carPosition.header.stamp = ros::Time::now();
    _carPosition.header.frame_id = "map";
    _carPosition.x = _position(0);
    _carPosition.y = _position(1);
    _carPosition.theta = _position(2);
    return _carPosition;
}

std_msgs::Float32 StateEstimation::getPMatrixTrace() { 

    //Update Covariance Matrix
    _P = _kalmanFilter.getStateCovariance();
    
    //Create Trace Message
    _traceP.data = _P.trace();
    return _traceP;
}

common_msgs::CarMotor StateEstimation::getEstimatedWheelSpeedRear() {

    //Create Velocity Message
    _wheelSpeedsRear.header.stamp = ros::Time::now();
    _wheelSpeedsRear.header.frame_id = _velocityFrameId;
    _wheelSpeedsRear.value2 = _x(8);
    _wheelSpeedsRear.value3 = _x(11);
    return _wheelSpeedsRear; 
}

common_msgs::CarMotor StateEstimation::getEstimatedWheelSpeedFront() {

    //Create Velocity Message
    _wheelSpeedsFront.header.stamp = ros::Time::now();
    _wheelSpeedsFront.header.frame_id = _velocityFrameId;
    _wheelSpeedsFront.value2 = _x(2);
    _wheelSpeedsFront.value3 = _x(5);
    return _wheelSpeedsFront; 
}

geometry_msgs::Vector3Stamped StateEstimation::getWheelSpeeds() {

    _wheelSpeeds.header.stamp = ros::Time::now();
    _wheelSpeeds.header.frame_id = _velocityFrameId;
    _wheelSpeeds.vector.x = _wheelSpeedsVector(0);
    _wheelSpeeds.vector.y = _wheelSpeedsVector(1);
    return _wheelSpeeds; 
}

geometry_msgs::Vector3Stamped StateEstimation::getSteering() {
    _steering.header.stamp = ros::Time::now();
    _steering.header.frame_id = _velocityFrameId;
    _steering.vector.x = _steeringVector(0);
    _steering.vector.y = _steeringVector(1);
    return _steering; 
}

geometry_msgs::Vector3Stamped StateEstimation::getTorque() {
    _torque.header.stamp = ros::Time::now();
    _torque.header.frame_id = _velocityFrameId;
    _torque.vector.x = _torqueVector(0);
    _torque.vector.y = _torqueVector(1);
    return _torque;
}

common_msgs::CarPose StateEstimation::getGpsPositionConverted() {

    _gpsPosition.header.stamp = ros::Time::now();
    _gpsPosition.header.frame_id = "map";
    _gpsPosition.x = _gpsCoordinatesConverted(0);
    _gpsPosition.y = _gpsCoordinatesConverted(1);
    _gpsPosition.theta = _gpsCoordinatesConverted(2);
    return _gpsPosition;
}

nav_msgs::Odometry StateEstimation::getGpsPositionOdometry() {
    _gpsOdom.header.stamp = ros::Time::now();
    _gpsOdom.header.frame_id = "map";
    _gpsOdom.pose.pose.position.x = _gpsCoordinatesConverted(0);
    _gpsOdom.pose.pose.position.y = _gpsCoordinatesConverted(1);
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0, 0.0, _gpsCoordinatesConverted(2));
    geometry_msgs::Quaternion quat_msg;
    tf2::convert(quat_tf, quat_msg);
    _gpsOdom.pose.pose.orientation = quat_msg;
    return _gpsOdom;
}

geometry_msgs::Vector3Stamped StateEstimation::getStaSteering() {
    _steering.header.stamp = ros::Time::now();
    _steering.header.frame_id = _velocityFrameId;
    _steering.vector.x = _staSteeringVector(0);
    _steering.vector.y = _staSteeringVector(1);
    return _steering; 
}

void StateEstimation::transformGpsToCG() {

    Eigen::Matrix<double, 2,1> outputCoor; 

    outputCoor = Eigen::Matrix2d::Identity()*Eigen::Matrix<double, 2,1>(_gpsCoordinatesConverted(0), _gpsCoordinatesConverted(1)) - _gpsCoordOffset;
    _gpsCoordTransformed(0) = outputCoor(0);
    _gpsCoordTransformed(1) = outputCoor(1);
    _gpsCoordTransformed(2) = _gpsCoordinatesConverted(2); 
}