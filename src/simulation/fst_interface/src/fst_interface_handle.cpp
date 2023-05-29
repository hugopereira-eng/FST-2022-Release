#include "fst_interface/fst_interface_handle.hpp"
#include "nav_msgs/Odometry.h"
const double pi = std::acos(-1);

/**
*	Name: FstInterfaceHandle.
*	Description: Fst Interface Handle construction. It advertises and subscribes all the necessary topics.
*	Inputs: ROS NodeHandle
*	Output: void
*/
FstInterfaceHandle::FstInterfaceHandle(ros::NodeHandle &nodeHandle) : _nodeHandle(nodeHandle){
    ROS_INFO("Constructing fst_interface_handle");
    advertiseToTopics();
    subscribeToTopics(); 
}

/**
*	Name: advertiseToTopics.
*	Description: Creates all the necessary ROS publishers. It's also where the name of the topics is defined.
*	Inputs: none
*	Output: void
*/
void FstInterfaceHandle::advertiseToTopics(){
    _pubFssimCmd = _nodeHandle.advertise<fssim_common::Cmd>("/fssim/cmd",1);
    _pubFssimMission = _nodeHandle.advertise<fssim_common::Mission>("/fssim/mission_finished", 1);
    _pubConeMarkerColor = _nodeHandle.advertise<visualization_msgs::MarkerArray>("/simulation/fst_interface/vis/markers_color", 1);
    _pubSFOut = _nodeHandle.advertise<common_msgs::ConeDetections>("/perception/cone_detections", 1);
    _pubFstRes = _nodeHandle.advertise<common_msgs::RES>("/common/res_state", 1);
    _pubFstVel = _nodeHandle.advertise<common_msgs::CarVelocity>("/estimation/state_estimation/velocity", 1);
    _pubFstPose = _nodeHandle.advertise<common_msgs::CarPose>("/estimation/state_estimation/position", 1);
    _pubFstWheelSpeeds = _nodeHandle.advertise<common_msgs::CarMotor>("/estimation/wheel_speeds", 1);
}

/**
*	Name: subscribeToTopics.
*	Description: Subscribes to the necessary topics.
*	Inputs: none
*	Output: void
*/
void FstInterfaceHandle::subscribeToTopics(){
    _subFssimOdom = _nodeHandle.subscribe("/fssim/base_pose_ground_truth", 1, &FstInterfaceHandle::odomCallback, this);
    _subFssimRes = _nodeHandle.subscribe("/fssim/res_state", 1, &FstInterfaceHandle::resCallback, this);
    _subCameraCones = _nodeHandle.subscribe("/lidar/cones", 1, &FstInterfaceHandle::cameraConesCallback, this);
    _subFstCmd = _nodeHandle.subscribe("/control/controller/control_cmd", 1, &FstInterfaceHandle::fstCmdCallback, this);
    _subFstMission = _nodeHandle.subscribe("/common/mission_tracker/mission_finished", 1, &FstInterfaceHandle::fstMissionCallback, this);
    _subFssimWheelSpeeds = _nodeHandle.subscribe("/fssim/wheelspeeds_ground_truth", 1, &FstInterfaceHandle::wheelSpeedsCallback, this);
}

/**
*	Name: wheelSpeedsCallback.
*	Description: Callback to update FstWheelSpeeds publisher.
*	Inputs: cmd of the type fssim_common::WheelSpeeds
*	Output: void
*/

void FstInterfaceHandle::wheelSpeedsCallback (const fssim_common::WheelSpeeds &wheelSpeeds) {
    common_msgs::CarMotor wSpeeds;

    wSpeeds.value2 = wheelSpeeds.rpm_rear_left * 10;
    wSpeeds.value3 = wheelSpeeds.rpm_rear_right * 10;

    _pubFstWheelSpeeds.publish(wSpeeds);
}

/**
*	Name: fstCmdCallback.
*	Description: Callback to update FssimCmd publisher.
*	Inputs: cmd of the type common_msgs::ControlCmd
*	Output: void
*/
void FstInterfaceHandle::fstCmdCallback (const common_msgs::ControlCmd &cmd) {
    fssim_common::Cmd fssimCmd;

    fssimCmd.dc = cmd.throttle;
    fssimCmd.delta = cmd.steering_angle;

    _pubFssimCmd.publish(fssimCmd);
}

/**
*	Name: cameraConesCallback.
*	Description: Callback to update the camera cone detections.
*	Inputs: pointCloud with the camera detections
*	Output: void
*/
void FstInterfaceHandle::cameraConesCallback (const sensor_msgs::PointCloud2 &pointCloud){

    float X = 0.0, Y = 0.0, Z = 0.0, I = 0.0, PROB_ORANGE = 0.0;

    common_msgs::ConeDetections cones;
    common_msgs::Cone cone;
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray markers;

    marker.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(marker);

    marker.header = pointCloud.header;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = 1.0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;

    cones.header = pointCloud.header;

    for (int i = 0; i < pointCloud.row_step/64; i++){
        memcpy(&X, &pointCloud.data[i*64 + 0], sizeof(float));
        memcpy(&Y, &pointCloud.data[i*64 + 4], sizeof(float));
        memcpy(&Z, &pointCloud.data[i*64 + 8], sizeof(float));
        memcpy(&I, &pointCloud.data[i*64 + 16], sizeof(float));
        memcpy(&PROB_ORANGE, &pointCloud.data[i*64 + 44], sizeof(float));

        float FOV;
        FOV = 0.4167*pi;
        if (std::fabs(Y) > std::tan(FOV/2)*X && std::hypot(X, Y) >= 5){
            marker.color.r = 0.8;
            marker.color.g = 0.8;
            marker.color.b = 0.8;
            cone.color = UNKNOWN_CONE;
        } else if (I == 0){ //orange  
            if(PROB_ORANGE == 0){
                marker.color.r = 0.8;
                marker.color.g = 0.8;
                marker.color.b = 0.8;
                cone.color = UNKNOWN_CONE; 
            }else{
                marker.color.r = 1.0;
                marker.color.g = 0.5;
                marker.color.b = 0.3;
                cone.color = ORANGE_CONE;  
            }
        }else if(I <= 0.5){ //blue
            marker.color.r = 0.0;
            marker.color.g = 0.1;
            marker.color.b = 0.8;
            cone.color = BLUE_CONE;
        } else { //yellow
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.2;
            cone.color = YELLOW_CONE;
        }

        cone.position.x = X;
        cone.position.y = Y;
        cone.position.z = Z;
        
        marker.id = i+1;
        marker.pose.position.x = X;
        marker.pose.position.y = Y;
        marker.pose.position.z = Z;
        cones.cone_detections.push_back(cone);
        markers.markers.push_back(marker);
    }
    _pubSFOut.publish(cones);
    _pubConeMarkerColor.publish(markers);
}

/**
*	Name: odomCallback.
*	Description: Callback to update the odometry publisher.
*	Inputs: fssim_common::State state
*	Output: void
*/
void FstInterfaceHandle::odomCallback (const fssim_common::State &state) {
    common_msgs::CarVelocity velocity;
    common_msgs::CarPose pose;

    velocity.header = state.header;
    velocity.velocity.x = state.vx;
    velocity.velocity.y = state.vy;
    velocity.velocity.theta = state.r;

    pose.header = state.header;
    pose.x = state.x;
    pose.y = state.y;
    pose.theta = state.yaw;

    _pubFstVel.publish(velocity);
    _pubFstPose.publish(pose);
}

/**
*	Name: odomCallback.
*	Description: Callback to update the mission publisher.
*	Inputs: common_msgs::Mission fstMission
*	Output: void
*/
void FstInterfaceHandle::fstMissionCallback(const common_msgs::Mission &fstMission) {
    fssim_common::Mission fssimMission;

    fssimMission.mission = fstMission.mission;
    fssimMission.finished = fstMission.finished;

    _pubFssimMission.publish(fssimMission);
}

/**
*	Name: resCallback.
*	Description: Callback to update the res state.
*	Inputs: fssim_common::ResState msg
*	Output: void
*/
void FstInterfaceHandle::resCallback(const fssim_common::ResState &msg) {
    common_msgs::RES res;
    res.emergency = msg.emergency;
    res.push_button = msg.push_button;

    _pubFstRes.publish(res);
}