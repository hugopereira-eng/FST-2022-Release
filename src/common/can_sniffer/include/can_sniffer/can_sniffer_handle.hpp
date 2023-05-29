#ifndef CAN_SNIFFER_HANDLE_HPP
#define CAN_SNIFFER_HANDLE_HPP

// c++ includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// networking includes
#include <netinet/in.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>

// fcp includes
#include "lib/fcp-cpp/fcp.hpp"
#include "lib/as_common.h"
#include "lib/vcan_bridge/server.hpp"

// eigen includes
#include <eigen3/Eigen/Eigen>

// ros includes
#include "ros/ros.h"
#include <tf2/LinearMath/Quaternion.h>

// ros standard msgs
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Int32MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"

// fst lisboa custom msgs
#include "common_msgs/ControlCmd.h"
#include "common_msgs/Mission.h"
#include "common_msgs/RES.h"
#include "common_msgs/ConeDetections.h"
#include "common_msgs/CarVelocity.h"
#include "common_msgs/CarMotor.h"
#include "common_msgs/StaPositionInfo.h"

#define PI 3.14159265

typedef struct drivingDynamics1 {
    uint16_t sid = 0x500;
    uint16_t speedActual;
    uint16_t speedTarget;
    int16_t steeringAngleActual;
    int16_t steeringAngleTarget;
    uint16_t brakeHydrActual;
    uint16_t brakeHydrTarget = 0;
    int16_t motorMomentActual;
    int16_t motorMomentTarget;
} DrivingDynamics1;

typedef struct drivingDynamics2 {
    uint16_t sid = 0x501;
    int16_t logitudinalAcc = 0;
    int16_t lateralAcc = 0;
    int16_t yawRate = 0;
} DrivingDynamics2;

typedef struct systemStatus {
    uint16_t sid = 0x502;
    uint16_t ASState;
    uint16_t EBSState;
    uint16_t AMIState;
    bool steeringState;
    uint16_t serviceBrakeState;
    uint16_t lapCounter;
    uint16_t conesCountActual;
    uint16_t conesCountAll;
} SystemStatus;

class CanSnifferHandle {

public:
    CanSnifferHandle(ros::NodeHandle&, uint16_t, uint16_t, uint16_t);

    void advertiseToTopics();
    void subscribeToTopics();
    void publishToTopics();
    void run();

private:
    //constructor inputs
    ros::NodeHandle _nodeHandle;
    uint16_t _essentialfd;
    uint16_t _sensorsfd;
    uint16_t _canopenfd;

    Fcp _fcp;

    //res init variables
    bool _resOperational = false;
    time_t _resOperationalSent = 0;

    //sta init variables
    // time_t _staOperationalSent = 0;
    // int stage = 0;
    // int _staPower;
    // bool _staStage1  = false;

    //data logger stuff
    DrivingDynamics1 _drivingDynamics1;
    DrivingDynamics2 _drivingDynamics2;
    SystemStatus _systemStatus;
    std::map<std::string, double> _data0;
    std::map<std::string, double> _data1;
    std::map<std::string, double> _data2;
    ros::Timer _timer1;

    //publishers
    ros::Publisher _pubASMission;
    ros::Publisher _pubASState;
    ros::Publisher _pubXsensOdom;
    ros::Publisher _pubXsensAccel;
    ros::Publisher _pubXsensGravityAccel;
    ros::Publisher _pubXsensGpsPosition;
    ros::Publisher _pubDashSteering;
    ros::Publisher _pubGpsVelocity;
    ros::Publisher _pubWheelSpeeds;
    ros::Publisher _pubMotorTorque;
    ros::Publisher _pubSTAPower;
    ros::Publisher _pubSTAPosition;


    //subscribers
    ros::Subscriber _subControlCmd;
    ros::Subscriber _subMissionFinished;
    ros::Subscriber _subLapCounter;
    ros::Subscriber _subDetectedCones;
    ros::Subscriber _subTotalCones;
    ros::Subscriber _subSpeedTarget;

    //ros callbacks
    void controlCmdCallback(const common_msgs::ControlCmd&);
    void missionFinishedCmdCallback(const common_msgs::Mission&);
    void lapCounterCallback(const std_msgs::Int16&);
    void detectedConesCallback(const common_msgs::ConeDetections&);
    void totalConesCallback(const sensor_msgs::PointCloud2&);
    void speedTargetCallback(const std_msgs::Float64&);

    //essential can
    bool essentialFilter(CANdata);
    void essentialHandle();
    void sendPedalToCan(float);
    void sendSteeringToCan(float);
    void publishASMission(as_missions);
    void handleASState(as_states);
    void publishRESState(bool, bool, bool);

    //sensors can
    bool sensorsFilter(CANdata);
    void sensorsHandle();

    //canopen can
    bool canopenFilter(CANdata);
    void canopenHandle();
    void sendStaPositionInfoMsg();

    //data logger methods
    void updateMessage();
    void dataLogger(std::string, std::string, std::map<std::string, double>, uint16_t);
    void sendDataLoggerMessage(const ros::TimerEvent&);

    //globals    
    double _yawRate = 0.0;
    Eigen::Matrix3d _rotMat;
    common_msgs::CarVelocity _odom;
    geometry_msgs::TwistWithCovarianceStamped _gpsVelocity;
    sensor_msgs::Imu _odom3D;
    sensor_msgs::Imu _gravityAccel;
    sensor_msgs::NavSatFix _gpsPosition;
    common_msgs::CarMotor _carVelocity;
    common_msgs::CarMotor _carMotor;
    as_states _state;
    
    int _positionActualSTA;
    int _positionDemandSTA;
    int _positionDemandController;
    bool _limitSTA;
    common_msgs::StaPositionInfo _msgStaPositionInfo;

    bool _regen = false;
};

#endif
