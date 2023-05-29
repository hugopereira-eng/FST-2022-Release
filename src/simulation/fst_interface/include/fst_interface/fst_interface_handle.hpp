#ifndef FST_INTERFACE_HANDLE_HPP
#define FST_INTERFACE_HANDLE_HPP

#include "shared/as_lib/common.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "fssim_common/State.h"
#include "fssim_common/Cmd.h"
#include "fssim_common/Mission.h"
#include "fssim_common/ResState.h"
#include "fssim_common/WheelSpeeds.h"
#include "common_msgs/ConeDetections.h"
#include "common_msgs/ControlCmd.h"
#include "common_msgs/Mission.h"
#include "common_msgs/RES.h"
#include "common_msgs/CarVelocity.h"
#include "common_msgs/CarPose.h"
#include "common_msgs/CarMotor.h"
#include <visualization_msgs/MarkerArray.h>

class FstInterfaceHandle {

public:
    // Constructor
    FstInterfaceHandle(ros::NodeHandle &nodeHandle);
    // Methods
    void advertiseToTopics(); 
    void subscribeToTopics();
private:
    // Methods
    void lidarConesCallback(const sensor_msgs::PointCloud2 &pointCloud);
    void cameraConesCallback(const sensor_msgs::PointCloud2 &pointCloud);
    void odomCallback (const fssim_common::State &state);
    void fstCmdCallback (const common_msgs::ControlCmd &cmd);
    void fstMissionCallback(const common_msgs::Mission &msg); 
    void resCallback(const fssim_common::ResState &msg); 
    void wheelSpeedsCallback (const fssim_common::WheelSpeeds &wheelSpeeds);

    //attributes
    ros::NodeHandle _nodeHandle;
    
    ros::Subscriber _subLidarCones;
    ros::Subscriber _subCameraCones;
    ros::Subscriber _subFssimOdom;
    ros::Subscriber _subFssimRes;
    ros::Subscriber _subFstMission;
    ros::Subscriber _subFstCmd;
    ros::Subscriber _subFssimWheelSpeeds;

    ros::Publisher _pubFssimCmd;
    ros::Publisher _pubFssimMission;
    ros::Publisher _pubConeMarkerColor;
    ros::Publisher _pubConeMarkerNoColor;
    ros::Publisher _pubSFOut;
    ros::Publisher _pubLidarOut;
    ros::Publisher _pubFstRes;
    ros::Publisher _pubFstVel;
    ros::Publisher _pubFstPose;
    ros::Publisher _pubFstWheelSpeeds;

    common_msgs::ConeDetections _coneDetections;
};

#endif