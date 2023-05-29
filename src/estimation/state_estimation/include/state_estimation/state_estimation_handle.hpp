#ifndef STATE_ESTIMATION_HANDLE_HPP
#define STATE_ESTIMATION_HANDLE_HPP

#include <ros/ros.h>
#include "state_estimation/state_estimation_pipeline.hpp"


class StateEstimationHandle {

    public:

        // Constructor
        StateEstimationHandle(ros::NodeHandle &nodeHandle);

        // Methods
        void advertiseToTopics(); 
        void subscribeToTopics();
        void publishToTopics();
        void runAlgorithm();

    private:

        ros::NodeHandle _nodeHandle;
        StateEstimation _stateEstimation;

        //Subscribers
        ros::Subscriber _subXsensAccel;
        ros::Subscriber _subDashSteering;
        ros::Subscriber _subWheelSpeeds;
        ros::Subscriber _subMotorTorque;
        ros::Subscriber _subGpsPosition;
        ros::Subscriber _subGpsVelocity;
        ros::Subscriber _subStaSteering;

        //Publishers
        ros::Publisher _pubCarVelocity;
        ros::Publisher _pubCovarianceTrace;
        ros::Publisher _pubWheelSpeeds;
        ros::Publisher _pubSteering;
        ros::Publisher _pubTorque;
        ros::Publisher _pubEstimatedWheelSpeedFront;
        ros::Publisher _pubEstimatedWheelSpeedRear;
        ros::Publisher _pubCarPosition;
        ros::Publisher _pubGpsPositionConverted;
        ros::Publisher _pubStaSteering;
        ros::Publisher _pubGpsPositionOdometry;

        void ImuCallback(const sensor_msgs::Imu &accel);
        void SteeringCallback(const common_msgs::ControlCmd &steering);
        void WheelSpeedCallback(const common_msgs::CarMotor &wheel_speeds);
        void MotorTorqueCallback(const common_msgs::CarMotor &torque);
        void GpsPositionCallback(const sensor_msgs::NavSatFix &gpsPosition);
        void GpsVelocityCallback(const geometry_msgs::TwistWithCovarianceStamped &gpsVelocity);
        void StaSteeringCallback(const common_msgs::StaPositionInfo &steering);


};

#endif