#include "can_sniffer/can_sniffer_handle.hpp"

CanSnifferHandle::CanSnifferHandle(ros::NodeHandle &nodeHandle, uint16_t fd1, uint16_t fd2, uint16_t fd3) : _nodeHandle(nodeHandle), _essentialfd(fd1), _sensorsfd(fd2), _canopenfd(fd3) {
    ROS_INFO("Constructing can_sniffer Handle");

    advertiseToTopics();
    subscribeToTopics();
    _timer1 = nodeHandle.createTimer(ros::Duration(0.1), &CanSnifferHandle::sendDataLoggerMessage, this, false);

    std::string fileName = _nodeHandle.param<std::string>("fcp_json_path", "rip");
    _fcp.decompile_file(fileName);
}

void CanSnifferHandle::advertiseToTopics() {
    ROS_INFO("Controller will advertise to topics");
    _pubASMission = _nodeHandle.advertise<common_msgs::Mission>("/missionTopic", 1);
    _pubASState = _nodeHandle.advertise<common_msgs::RES>("/common/res_state", 1);
    _pubXsensOdom = _nodeHandle.advertise<common_msgs::CarVelocity>("/estimation/velocity", 1);
    _pubGpsVelocity = _nodeHandle.advertise<geometry_msgs::TwistWithCovarianceStamped>("/estimation/gps_velocity", 1);
    _pubXsensAccel = _nodeHandle.advertise<sensor_msgs::Imu>("/estimation/odom3D", 1);
    _pubXsensGravityAccel = _nodeHandle.advertise<sensor_msgs::Imu>("/estimation/accel_gravity", 1);
    _pubXsensGpsPosition = _nodeHandle.advertise<sensor_msgs::NavSatFix>("/estimation/gps_position", 1);
    _pubDashSteering = _nodeHandle.advertise<common_msgs::ControlCmd>("/control/controller/steering_actual", 1);
    _pubWheelSpeeds = _nodeHandle.advertise<common_msgs::CarMotor>("/estimation/wheel_speeds", 1);
    _pubMotorTorque = _nodeHandle.advertise<common_msgs::CarMotor>("/estimation/motor_torque", 1);
    _pubSTAPower = _nodeHandle.advertise<common_msgs::CarMotor>("/common/can_sniffer/sta_power", 1);
    _pubSTAPosition = _nodeHandle.advertise<common_msgs::StaPositionInfo>("/common/can_sniffer/sta_position", 1);

}

void CanSnifferHandle::subscribeToTopics() {
    ROS_INFO("can_sniffer will subscribe to topics");
    _subControlCmd = _nodeHandle.subscribe("/control/controller/control_cmd", 1, &CanSnifferHandle::controlCmdCallback, this);
    _subMissionFinished = _nodeHandle.subscribe("/common/mission_tracker/mission_finished", 1, &CanSnifferHandle::missionFinishedCmdCallback, this);
    _subLapCounter = _nodeHandle.subscribe("/common/mission_tracker/lap_counter", 1, &CanSnifferHandle::lapCounterCallback, this);
    _subDetectedCones = _nodeHandle.subscribe("/perception/cone_detections", 1, &CanSnifferHandle::detectedConesCallback, this);
    _subTotalCones = _nodeHandle.subscribe("/estimation/slam/cones", 1, &CanSnifferHandle::totalConesCallback, this);
    _subSpeedTarget = _nodeHandle.subscribe("/control/controller/speed_target", 1, &CanSnifferHandle::speedTargetCallback, this);
}

void CanSnifferHandle::run() {
    essentialHandle();
    sensorsHandle();
    canopenHandle();
}

void CanSnifferHandle::lapCounterCallback(const std_msgs::Int16 &lap_counter) {
    _systemStatus.lapCounter = lap_counter.data;
}

void CanSnifferHandle::detectedConesCallback(const common_msgs::ConeDetections &detections) {
    _systemStatus.conesCountActual = detections.cone_detections.size();
}

void CanSnifferHandle::totalConesCallback(const sensor_msgs::PointCloud2 &totalCones) {
    _systemStatus.conesCountAll = totalCones.width;
}

void CanSnifferHandle::speedTargetCallback(const std_msgs::Float64 &speed_target){
    _drivingDynamics1.speedTarget = speed_target.data*3.6;
}

void CanSnifferHandle::controlCmdCallback(const common_msgs::ControlCmd &controlCmd) {
    _drivingDynamics1.motorMomentTarget = controlCmd.throttle*100;
    // _drivingDynamics1.steeringAngleTarget = controlCmd.steering_angle*-150;

    sendPedalToCan(controlCmd.throttle);
    sendSteeringToCan(controlCmd.steering_angle);
}

void CanSnifferHandle::missionFinishedCmdCallback(const common_msgs::Mission &mission) {
    CANdata msg;
    struct can_frame frame;
    msg = _fcp.encode_cmd("as", "ebs_as_state", "ebs", AS_FINISHED, 0, 0);

    CANdata_to_CANFrame(msg, &frame);
    send_to_CAN_Bus_Line(_essentialfd, frame);
}

/**
 * 
 * 
 *      END OF USUALL ROS STUFF
 * 
 * 
 */


/**
 * 
 * 
 *      ESSENTIAL CAN LINE STUFF
 * 
 * 
 */

bool CanSnifferHandle::essentialFilter (CANdata message) {

    std::string devName = _fcp.get_dev_name(message.sid);
    std::string msgName = _fcp.get_msg_name(message.sid);

    if ((msgName == "send_cmd"))
        return 1;

    if (devName == "ebs" && (msgName == "ebs_status" || msgName == "nmt_go_to")) //data2
        return 1;

    if (devName == "iib" && (msgName == "iib_amk_values_1")) {
        return 1;
    }
 
    if (devName == "iib" && (msgName == "iib_motor")) //Speed Actual
        return 1;

    if (devName == "dash" && (msgName == "dash_se")) //Steering Actual (later change to value from the STA)
        return 1;
        
    if (devName == "te" && (msgName == "te_press")) //brake hydr actual
        return 1;

    if (devName == "sta")
        return 1;

    return 0;
}

void CanSnifferHandle::essentialHandle() {

    CANdata message;
    struct can_frame frame;

    if (!receive_from_CAN_Bus_Line(_essentialfd, &frame)) return;
    CANFrame_to_CANdata(frame, &message);

    if (essentialFilter(message)){
        std::pair<std::string, std::map<std::string, double>> decodedMsg;
        try {   
            decodedMsg = _fcp.decode_msg(message);

        }
        catch (...) {
            std::cout << "msg.sid: " << message.sid;
            return;
        }
        std::string devName = _fcp.get_dev_name(message.sid);
       
        if ((devName == "ebs") && (decodedMsg.first == "ebs_as_mission")){ //add as mission to ebs status message
            _systemStatus.AMIState = decodedMsg.second["arg1"];
        }

        if ((devName == "ebs") && (decodedMsg.first == "ebs_status")){
            _systemStatus.EBSState = decodedMsg.second["ebs_ebs_state"];
            _systemStatus.steeringState = decodedMsg.second["ebs_sta"];
            _systemStatus.serviceBrakeState = decodedMsg.second["ebs_sb_state"]; //this signal doesnt exist
            _systemStatus.ASState = decodedMsg.second["ebs_as_state"];
            handleASState((as_states)decodedMsg.second["ebs_as_state"]); // change AS 
            publishASMission((as_missions)decodedMsg.second["ebs_as_mission"]);
        }

        if ((devName =="iib") && (decodedMsg.first == "iib_motor")) { //converter de RPM para km/h 
            _drivingDynamics1.speedActual = decodedMsg.second["motor_speed"];
            //_pubWheelSpeeds.publish(_drivingDynamics1.speedActual);
        }

        if ((devName == "iib") && (decodedMsg.first == "iib_motor")) { 
            _drivingDynamics1.motorMomentActual = decodedMsg.second["motor_torque"];
        }

        if (devName == "iib" && decodedMsg.first == "iib_amk_values_1") {
            
            _carVelocity.header.stamp = _carMotor.header.stamp =  ros::Time::now();
            _carVelocity.header.frame_id = _carMotor.header.frame_id = "cog";

            if(decodedMsg.second["n_amk_values1"] == 2) {
                _carVelocity.value2=static_cast<int32_t>(decodedMsg.second["amk_actual_speed2"]);
                _carMotor.value2=static_cast<int32_t>(decodedMsg.second["amk_torque_c2"]); //*0.0014 Nm
                //std::cout << decodedMsg.second["n_amk_values1"] << ": " << decodedMsg.second["amk_actual_speed2"] << std::endl;
            } else if(decodedMsg.second["n_amk_values1"] == 3) {            
                _carVelocity.value3=static_cast<int32_t>(decodedMsg.second["amk_actual_speed3"]);
                _carMotor.value3=static_cast<int32_t>(decodedMsg.second["amk_torque_c3"]);
                //std::cout << decodedMsg.second["n_amk_values1"] << ": " << decodedMsg.second["amk_actual_speed3"] << std::endl;
                
                _pubWheelSpeeds.publish(_carVelocity);
                _pubMotorTorque.publish(_carMotor);
            }

            float motor_torque = (_carMotor.value2 + _carMotor.value3)*0.0014; //Nm
            _drivingDynamics1.motorMomentActual = int(motor_torque / 21000.0 * 100.0);
        }

        if ((devName == "dash") && (decodedMsg.first == "dash_se")) { 
            // _drivingDynamics1.steeringAngleActual = decodedMsg.second["dash_se"];
            common_msgs::ControlCmd msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "ahrs_link";
            msg.steering_angle = decodedMsg.second["dash_se"] / 1800.0;
            _pubDashSteering.publish(msg);
        }

        if ((devName == "te") && (decodedMsg.first == "te_press")) { //tem de ser em percetangem (nao sei qual e a percentagem maxima)
            _drivingDynamics1.brakeHydrActual = decodedMsg.second["te_press_f"] / 120.0 * 100.0;
        }

        _drivingDynamics1.brakeHydrTarget = 0;

        // //torque actual and current actual;
        // if (devName == "sta" && decodedMsg.first == "sta_power"){
        //     //std_msgs::Int32MultiArray msg;
        //     common_msgs::CarMotor msg;
        //     msg.header.stamp = ros::Time::now();
        //     msg.header.frame_id = "cog";
        //     //msg.data.resize(2);
        //     msg.value2 /* msg.data[0] */ = static_cast<int16_t>(decodedMsg.second["torque"]);
        //     msg.value3 /* msg.data[1] */ = static_cast<int16_t>(decodedMsg.second["current"]);

        //     _pubSTAPower.publish(msg);
        // }

        // //position actual and position demanded
        // if (devName == "sta" && decodedMsg.first == "sta_position"){
        //     //std_msgs::Int32MultiArray msg;
        //     common_msgs::CarMotor msg;
        //     msg.header.stamp = ros::Time::now();
        //     msg.header.frame_id = "cog";
        //     //msg.data.resize(2);
        //     msg.value2 /* msg.data[0] */ = static_cast<int32_t>(decodedMsg.second["position_actual"]);
        //     msg.value3 /* msg.data[1] */ = static_cast<int32_t>(decodedMsg.second["position_demand"]);

        //     _pubSTAPosition.publish(msg);
        // }
    }
}

void CanSnifferHandle::sendPedalToCan(float pedalValue) {
    // Send the throttle as a CAN message to the AS
    CANdata pedalSetPointMessage;
    struct can_frame frame;

    if (pedalValue >= 0)
        pedalSetPointMessage = _fcp.encode_cmd("as", "pedal_setpoint", "iib", pedalValue * 10000, 0, 0);
    else
        pedalSetPointMessage = _fcp.encode_cmd("as", "pedal_setpoint", "iib", 0, pedalValue * -10000, 0);

    CANdata_to_CANFrame(pedalSetPointMessage, &frame);
    send_to_CAN_Bus_Line(_essentialfd, frame);
}

void CanSnifferHandle::sendSteeringToCan(float steeringValue) {

    CANdata msg;
    struct can_frame frame;
    msg.sid = 0x320;
    msg.dlc = 6;
    _positionDemandController = int(steeringValue * 81920.0);
    _msgStaPositionInfo.positionDemandController = _positionDemandController;
    float posDemandController = (static_cast<float>(_positionDemandController) / 20.0 / 4096.0) * 360.0;
    _drivingDynamics1.steeringAngleTarget = floor(posDemandController/0.5 + 0.5) * 0.5; // Applying the scale for data logger;
    
    sendStaPositionInfoMsg();

    msg.data[0] = 0x003F;
    msg.data[1] = _positionDemandController         & 0xFFFF; 
    msg.data[2] = (_positionDemandController >> 16) & 0xFFFF;

    CANdata_to_CANFrame(msg, &frame);
    send_to_CAN_Bus_Line(_canopenfd, frame);

    //patinho time
    CANdata msg_;
    struct can_frame frame_;
    msg_.sid = 0x220;
    msg_.dlc = 2;
    msg_.data[0] = 0x002F;
    CANdata_to_CANFrame(msg_, &frame_);
    send_to_CAN_Bus_Line(_canopenfd, frame_);


}

void CanSnifferHandle::publishASMission(as_missions arg1){ //not called

    common_msgs::Mission mission;
    mission.header.stamp = ros::Time::now();
    mission.mission = arg1;

    _pubASMission.publish(mission);
}

void CanSnifferHandle::handleASState(as_states state) {

    switch (state) {
        
        case AS_OFF:
            break;

        case AS_READY:
            _state = as_states::AS_READY;
            break;

        case AS_DRIVING:
            publishRESState(true, false, _regen);
        
            break;

        case AS_FINISHED:
            break;

        case AS_EMERGENCY:
            break;

        case AS_MANUAL:
            break;
        
        default:
            break;
    }
}

void CanSnifferHandle::publishRESState(bool go, bool emergency, bool vsv_brake) {

    common_msgs::RES controlState;
    controlState.header.stamp = ros::Time::now();
    controlState.emergency = emergency;
    controlState.push_button = go;
    controlState.vsv_brake = vsv_brake;
    _pubASState.publish(controlState);
}

/**
 * 
 * 
 *      SENSORS CAN LINE STUFF
 * 
 * 
 */

bool CanSnifferHandle::sensorsFilter (CANdata msg){

    std::string devName = _fcp.get_dev_name(msg.sid);
    std::string msgName = _fcp.get_msg_name(msg.sid);
    
    if (devName == "xsens")
        return true;

    return false;
}

void CanSnifferHandle::sensorsHandle () {

    struct can_frame frame;
    CANdata msg;

    if (!receive_from_CAN_Bus_Line(_sensorsfd, &frame)) return;
    CANFrame_to_CANdata(frame, &msg);

    if (sensorsFilter(msg)){
        std::pair<std::string, std::map<std::string, double>>  decodedMsg;
        try {
            decodedMsg = _fcp.decode_msg(msg);
        }
        catch (...) {
            std::cout << "msg.sid: " << msg.sid;
            return;
        }
        std::string devName = _fcp.get_dev_name(msg.sid);

        if (devName == "xsens" && decodedMsg.first == "XCDI_EulerAngles"){
            
            double Yaw = _odom.yaw = decodedMsg.second["Yaw"];            
            double Pitch = _odom.pitch = decodedMsg.second["Pitch"];
            double Roll = _odom.roll = decodedMsg.second["Roll"];
            

            //Convert Degree to Rad
            Yaw = Yaw * (PI/180.0);
            Pitch = Pitch * (PI/180.0);
            Roll = Roll * (PI/180.0);

            tf2::Quaternion quaternion;
            quaternion.setRPY(Roll, Pitch, Yaw);

            _odom3D.orientation.w = quaternion.w();
            _odom3D.orientation.x = quaternion.x();
            _odom3D.orientation.y = quaternion.y();
            _odom3D.orientation.z = quaternion.z();


            _rotMat << cosf(Pitch)*cosf(Yaw), sinf(Roll)*sinf(Pitch)*cosf(Yaw)-cosf(Roll)*sinf(Yaw), cosf(Roll)*sinf(Pitch)*cosf(Yaw)+sinf(Roll)*sinf(Yaw),
                       cosf(Pitch)*sinf(Yaw), sinf(Roll)*sinf(Pitch)*sinf(Yaw)+cosf(Roll)*cosf(Yaw), cosf(Roll)*sinf(Pitch)*sinf(Yaw)-sinf(Roll)*cosf(Yaw),
                       -sinf(Pitch), sinf(Roll)*cosf(Pitch), cosf(Roll)*cosf(Pitch);
        }

        if (devName == "xsens" && decodedMsg.first == "XCDI_RateOfTurn"){
            _yawRate = decodedMsg.second["gyrZ"];
            float yawScale = 0.0078125;
            _drivingDynamics2.yawRate = _yawRate*180.0/M_PI;
            //printf("Rate of turn: %lf\n", _yawRate);            
        }

        if (devName == "xsens" && decodedMsg.first == "XCDI_VelocityXYZ"){
            // ROS_WARN("Received message: %s\n", decodedMsg.first.c_str());
            double velX = decodedMsg.second["velX"];
            double velY = decodedMsg.second["velY"];
            double velZ = decodedMsg.second["velZ"];

            Eigen::Vector3d vel = Eigen::Vector3d(velX, velY, velZ);

            //Coordinates of the sensor in the car frame
            Eigen::Vector3d sensor_coord = Eigen::Vector3d(0, -0.7, 0);

            //Convert vx and vy to sensor frame
            vel = _rotMat.transpose()*vel;
            //vel = vel + _yawRate*sensor_coord;           

            _odom.header.frame_id = "ahrs_link";
            _odom.header.stamp = ros::Time::now();
            _odom.velocity.x = vel[0];
            _odom.velocity.y = vel[1];
            _odom.velocity.theta = _yawRate;

            _gpsVelocity.header.frame_id = "ahrs_link";
            _gpsVelocity.header.stamp = ros::Time::now();
            _gpsVelocity.twist.twist.linear.x = vel[0];
            _gpsVelocity.twist.twist.linear.y = vel[1];

            //printf("vx: %lf | vy: %lf | r: %lf\n", vel.x(), vel.y(), _yawRate);
            //printf("v: %lf\n", vel.norm());
            _pubGpsVelocity.publish(_gpsVelocity);
            _pubXsensOdom.publish(_odom);
        }

        if (devName == "xsens" && decodedMsg.first == "XCDI_Acceleration"){
            _gravityAccel.linear_acceleration.x = decodedMsg.second["accX"];
            _gravityAccel.linear_acceleration.y = decodedMsg.second["accY"];
            _gravityAccel.linear_acceleration.z = decodedMsg.second["accZ"];

            _gravityAccel.header.frame_id = "ahrs_link";
            _gravityAccel.header.stamp = ros::Time::now();
            _pubXsensGravityAccel.publish(_gravityAccel);
        }
        
        if (devName == "xsens" && decodedMsg.first == "XCDI_FreeAcceleration"){
            _odom3D.linear_acceleration.x = decodedMsg.second["freeAccX"];
            _odom3D.linear_acceleration.y = decodedMsg.second["freeAccY"];
            _odom3D.linear_acceleration.z = decodedMsg.second["freeAccZ"];

            _drivingDynamics2.logitudinalAcc = _odom3D.linear_acceleration.x;
            _drivingDynamics2.lateralAcc = _odom3D.linear_acceleration.y;
        }

        /* if (devName == "xsens" && decodedMsg.first == "XCDI_Quaternion"){
            _odom3D.orientation.w = decodedMsg.second["Q0"];
            _odom3D.orientation.x = decodedMsg.second["Q1"];
            _odom3D.orientation.y = decodedMsg.second["Q2"];
            _odom3D.orientation.z = decodedMsg.second["Q3"];
        } */

        if (devName == "xsens" && decodedMsg.first == "XCDI_RateOfTurn"){
            _odom3D.angular_velocity.x = decodedMsg.second["gyrX"];
            _odom3D.angular_velocity.y = decodedMsg.second["gyrY"];
            _odom3D.angular_velocity.z = decodedMsg.second["gyrZ"];

            _odom3D.header.frame_id = "ahrs_link";
            _odom3D.header.stamp = ros::Time::now();
            _pubXsensAccel.publish(_odom3D);
        }

        if(devName == "xsens" && decodedMsg.first == "XCDI_Latitude") {
            _gpsPosition.latitude = decodedMsg.second["lat"];
            _gpsPosition.longitude = decodedMsg.second["lon"];
        }

        if(devName == "xsens" && decodedMsg.first == "XCDI_AltitudeEllipsoid") {
            _gpsPosition.altitude = decodedMsg.second["altEllipsoid"];
            _gpsPosition.header.stamp = ros::Time::now();
            _gpsPosition.header.frame_id = "ahrs_link";

            _pubXsensGpsPosition.publish(_gpsPosition);
        }
    }
}



/**
 * 
 * 
 *      CANOPEN CAN LINE STUFF
 * 
 * 
 */

bool CanSnifferHandle::canopenFilter (CANdata msg){
    return true;
}

void CanSnifferHandle::canopenHandle () {

    struct can_frame frame;
    CANdata msg;

    if (!receive_from_CAN_Bus_Line(_canopenfd, &frame)) return;
    
    if (frame.can_id == 0x191){
        // RES stuff
        bool emergency_signal = !((frame.data[0] & 1) | ((frame.data[3] >> 7) & 1));
        _regen = !((frame.data[0] >> 1) & 0b1);


        if (emergency_signal)
            publishRESState(false, true, false);
    } 
    
    if      (frame.can_id == 0x2A0) frame.can_id = 0x70b;
    else if (frame.can_id == 0x3A0) frame.can_id = 0x72b;
    else if (frame.can_id == 0x4A0) frame.can_id = 0x00;
    else if (frame.can_id == 0x1A0) {
        _limitSTA = (frame.data[1]  >> 3) & 0b1;
        _msgStaPositionInfo.limit = _limitSTA;
    }
    

    CANFrame_to_CANdata(frame, &msg); 
    std::pair<std::string, std::map<std::string, double>>  decodedMsg;
    try {
        decodedMsg = _fcp.decode_msg(msg);
    }
    catch (...) {
        //std::cout << "msg.sid: " << msg.sid;
        return;
    }

    std::string devName = _fcp.get_dev_name(msg.sid);

    if (devName == "sta" && decodedMsg.first == "sta_TPDO2"){
        _positionActualSTA = static_cast<int>(decodedMsg.second["position_actual"]);
        _msgStaPositionInfo.positionActual = _positionActualSTA;
        _msgStaPositionInfo.steeringEncoderSTA = (static_cast<float>(_positionActualSTA) / 20.0 / 4096.0) * 360.0 ;
        _drivingDynamics1.steeringAngleActual = floor(_msgStaPositionInfo.steeringEncoderSTA/0.5 + 0.5) * 0.5;
        _msgStaPositionInfo.torqueActual = decodedMsg.second["torque_actual"] * 0.001;
    }

    if (devName == "sta" && decodedMsg.first == "sta_TPDO3"){
        _positionDemandSTA = decodedMsg.second["position_demand"];
        _msgStaPositionInfo.positionDemandSTA = _positionDemandSTA;
        _msgStaPositionInfo.velocityActual = (decodedMsg.second["velocity_actual"] / 60.0)*2*M_PI;
    }
}

void CanSnifferHandle::sendStaPositionInfoMsg () {
    _msgStaPositionInfo.deltaController =   std::abs(_positionActualSTA - _positionDemandController);
    _msgStaPositionInfo.deltaSTA =          std::abs(_positionActualSTA - _positionDemandSTA);
    _msgStaPositionInfo.powerActual = _msgStaPositionInfo.velocityActual * _msgStaPositionInfo.torqueActual;

    _msgStaPositionInfo.header.stamp = ros::Time::now();
    _msgStaPositionInfo.header.frame_id = "cog";
    _pubSTAPosition.publish(_msgStaPositionInfo);

    _msgStaPositionInfo.deltaController = 0;
    _msgStaPositionInfo.deltaSTA = 0;
    _msgStaPositionInfo.positionActual = 0;
    _msgStaPositionInfo.positionDemandController = 0;
    _msgStaPositionInfo.positionDemandSTA = 0;
    _msgStaPositionInfo.steeringEncoderSTA = 0;
    _msgStaPositionInfo.torqueActual = 0;
    _msgStaPositionInfo.velocityActual = 0;
    _msgStaPositionInfo.powerActual = 0;
}

/**
 * 
 * 
 *      DATA LOGGER STUFF
 * 
 * 
 */

void CanSnifferHandle::updateMessage() {

    _data0 = {{"speed_actual", (_drivingDynamics1.speedActual/16/60)*2*M_PI*0.2*3.6}, //DONE
              {"speed_target", _drivingDynamics1.speedTarget}, // NOT DONE
              {"steering_angle_actual", _drivingDynamics1.steeringAngleActual}, //DONE
              {"steering_angle_target", _drivingDynamics1.steeringAngleTarget}, //DONE
              {"brake_hydr_actual", _drivingDynamics1.brakeHydrActual}, //DONE
              {"brake_hydr_target", _drivingDynamics1.brakeHydrTarget}, // DONE
              {"motor_momentum_actual", _drivingDynamics1.motorMomentActual}, //DONE
              {"motor_momentum_target", _drivingDynamics1.motorMomentTarget}}; //DONE BUT NEEDS REVIEW

    _data1 = {{"acc_longitudinal", _drivingDynamics2.logitudinalAcc}, //DONE NEEDS SCALING
              {"acc_lateral", _drivingDynamics2.lateralAcc}, //DONE NEEDS SCALING
              {"yaw_rate", _drivingDynamics2.yawRate}}; //DONE NEEDS SCALING

    _data2 = {{"AS_state", _systemStatus.ASState}, //DONE
              {"EBS_state", _systemStatus.EBSState}, //DONE
              {"AMI_STATE", _systemStatus.AMIState}, //DONE
              {"steering_state", _systemStatus.steeringState}, //DONE
              {"service_brake", _systemStatus.serviceBrakeState}, //DONE
              {"lap_counter", _systemStatus.lapCounter}, //DONE
              {"cone_count_actual", _systemStatus.conesCountActual}, //DONE
              {"cones_count_all", _systemStatus.conesCountAll}}; //DONE
}

/*
ID 0 - DV driving dynamics 1
    [x] - Speed_actual - IIB - MSG - iib_motor - motor_speed
    [x] - Speed_target - Fazer parse do valor m/s to km/h
    [x] - Steering_angle_actual - DASH - dash_se - dash_se
    [x] - Steering_angle_target  - Fazer parse do valor no callback controlCmdCallback
    [x] - Brake_hydr_actual - message.dev_id == DEVICE_ID_TE && message.msg_id == MSG_ID_TE_TE_PRESS
    [x] - Brake_hydr_target -  ebs armed -  , ebs actived 60/70
    [x] - Motor_moment_actual - IIB -         ?? TE - MSG - te_main - te_main_APPS ????? Dubio! TORQUE??? who knows? Maybe mamas? , Pedro deixa de ser javardo, desculpa Vai mesmo mamas
    [x] - Motor_moment_target - % -> manda o pedal,   estamos a mandar o pedal * torque maximo (ver da iib) ?? fazer parse do valor no callback ControlCmdCallback 

ID 1 - DV driving dynamics 2
    [ ] - Acceleration longitudinal - ARHS stuff
    [ ] - Acceleration lateral - ARHS stuff
    [ ] - Yaw rate - ARHS stuff

ID 2 - DV system status: DONE DEAL 
    [x] - AS_STATE - EBS - CMD - ebs_as_state 
    [x] - EBS_STATE - EBS - MSG - ebs_ebs_state 
    [x] - AMI_STATE - EBS - CMD - ebs_as_mission 
    [x] - Steering_state - EBS - MSG - ebs_sta
    [x] - Service_brake_state - EBS - MSG - ebs_sb_states
    [x] - Lap_counter - Callback Done
    [x] - Cones_count - Callback Done
    [x] - Cones_count_all - Callback Done
*/

void CanSnifferHandle::dataLogger(std::string dev_id, std::string msg_id, std::map<std::string, double> data, uint16_t sid) {

    CANdata message;
    struct can_frame frame;

    message = _fcp.encode_msg(dev_id, msg_id, data);
    message.sid = sid;
    CANdata_to_CANFrame(message, &frame);
    send_to_CAN_Bus_Line(_sensorsfd, frame);
}

void CanSnifferHandle::sendDataLoggerMessage(const ros::TimerEvent &event) {
    updateMessage();
    dataLogger("as", "DV_driving_dynamics_1", _data0, _drivingDynamics1.sid);
    dataLogger("as", "DV_driving_dynamics_2", _data1, _drivingDynamics2.sid);
    dataLogger("as", "DV_system_status", _data2, _systemStatus.sid);
}
