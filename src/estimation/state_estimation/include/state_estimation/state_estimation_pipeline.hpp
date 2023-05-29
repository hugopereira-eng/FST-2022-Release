#ifndef STATE_ESTIMATION_PIPELINE_HPP
#define STATE_ESTIMATION_PIPELINE_HPP

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include "state_estimation/state_estimation_pipeline.hpp"
#include "state_estimation/state_estimation_params.hpp"
#include "state_estimation/state_estimation_ekf.hpp"
#include "common_msgs/CarMotor.h"
#include "sensor_msgs/Imu.h"
#include "common_msgs/CarVelocity.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "common_msgs/ControlCmd.h"
#include "common_msgs/CarPose.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include "common_msgs/StaPositionInfo.h"


class StateEstimation {

    public:
        StateEstimation(ros::NodeHandle &nh);
        void run();

        //Setters
        void setImuMeasure(const sensor_msgs::Imu &odom3d);
        void setWheelSpeedsMeasure(const common_msgs::CarMotor &wheel_speeds);
        void setTorqueMeasure(const common_msgs::CarMotor &torque);
        void setSteeringMeasure(const common_msgs::ControlCmd &steering);
        void setGpsPosition(const sensor_msgs::NavSatFix &gpsPosition);
        void setGpsVelocity(const geometry_msgs::TwistWithCovarianceStamped &gpsVelocity);
        void setStaSteering(const common_msgs::StaPositionInfo &steering);

        //Getters
        common_msgs::CarVelocity getCarVelocity();
        std_msgs::Float32 getPMatrixTrace();
        common_msgs::CarMotor getEstimatedWheelSpeedRear();
        common_msgs::CarMotor getEstimatedWheelSpeedFront();
        geometry_msgs::Vector3Stamped getWheelSpeeds();
        geometry_msgs::Vector3Stamped getSteering();
        geometry_msgs::Vector3Stamped getTorque();
        geometry_msgs::Vector3Stamped getStaSteering();
        common_msgs::CarPose getCarPosition();
        common_msgs::CarPose getGpsPositionConverted();
        nav_msgs::Odometry getGpsPositionOdometry();

    private:
        void loadCarParameters(ros::NodeHandle &nh);
        void loadEkfParameters(ros::NodeHandle &nh);
        void convertGpsCoordinates();
        void estimatePosition();
        void transformGpsToCG();

        Params _params;
        KalmanFilter _kalmanFilter;
        ros::Time _lastUpdateTime;
        Eigen::Vector2d _gpsCoordOffset;
        Eigen::Vector2d _gpsInitial;
        bool _useGPS;
        bool _init = true;
        double _wz;
        double _initialYaw = 0;
        double _currentXsensYaw = 0;
        double _lastDx;
        double _lastDy;
        float _distanceIncrement = 0;
        std::string _velocityFrameId;
        std::string _ahrsFrameId;

        Eigen::Matrix<double,15,1> _x;
        Eigen::Matrix<double,15,15> _P;

        //Messages to publish
        common_msgs::CarVelocity _carVelocity;
        common_msgs::CarPose _carPosition;
        std_msgs::Float32 _traceP;
        common_msgs::CarMotor _wheelSpeedsRear;
        common_msgs::CarMotor _wheelSpeedsFront;

        //Debug Messages
        Eigen::Matrix<double,2,1> _wheelSpeedsVector;
        Eigen::Matrix<double,2,1> _torqueVector;
        Eigen::Matrix<double,2,1> _steeringVector;
        Eigen::Matrix<double,2,1> _staSteeringVector;
        
        geometry_msgs::Vector3Stamped _wheelSpeeds;
        geometry_msgs::Vector3Stamped _torque;
        geometry_msgs::Vector3Stamped _steering;
        geometry_msgs::Vector3Stamped _staSteering;
        common_msgs::CarPose _gpsPosition;
        nav_msgs::Odometry _gpsOdom;

        //Atributes to estimate position
        Eigen::Matrix<double,3,1> _lastGpsCoordinates;
        Eigen::Matrix<double,3,1> _gpsCoordinates;
        Eigen::Matrix<double,3,1> _gpsCoordinatesConverted;
        Eigen::Matrix<double,3,1> _gpsCoordTransformed;
        Eigen::Matrix<double,3,1> _position;
        Eigen::Matrix<double,3,1> _lastPosition;
        Eigen::Matrix<double,3,3> _Ppos;
        Eigen::Matrix<double,3,3> _Qpos;
        Eigen::Matrix<double,3,3> _Rpos;
        Eigen::Matrix<double,3,3> _JApos;
        Eigen::Matrix<double,3,3> _JHpos;
};

#endif