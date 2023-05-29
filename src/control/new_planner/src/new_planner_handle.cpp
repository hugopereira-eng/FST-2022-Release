#include "new_planner/new_planner_handle.hpp"

NewPlannerHandle::NewPlannerHandle(ros::NodeHandle &nodeHandle) : _nodeHandle(nodeHandle) {
    this->advertiseToTopics();
    this->subscribeToTopics();
    this->loadParameters();

    this->_particles.resize(this->_particleCount, Particle());

    #if 0
    common_msgs::Cone cone;
    cone.position.x = 0.2; cone.position.y = -2; //(0, -2)
    this->_cones.cone_detections.push_back(cone);
    cone.position.x = 0.2; cone.position.y = 2;  //(0, 2)
    this->_cones.cone_detections.push_back(cone);
    cone.position.x = 3; cone.position.y = -2; //(3, -2)
    this->_cones.cone_detections.push_back(cone);
    cone.position.x = 3; cone.position.y = 2;  //(3, 2)
    this->_cones.cone_detections.push_back(cone);
    cone.position.x = 6; cone.position.y = 1; //(6, 1)
    this->_cones.cone_detections.push_back(cone);
    cone.position.x = 6; cone.position.y = -3; //(6, -3)
    this->_cones.cone_detections.push_back(cone);
    cone.position.x = 9; cone.position.y = -1; //(9, -1)
    this->_cones.cone_detections.push_back(cone);
    cone.position.x = 8; cone.position.y = -4; //(8, -4)
    this->_cones.cone_detections.push_back(cone);
    cone.position.x = 12; cone.position.y = -3; //(12, -3)
    this->_cones.cone_detections.push_back(cone);
    cone.position.x = 9; cone.position.y = -6; //(9, -6)
    this->_cones.cone_detections.push_back(cone);
    cone.position.x = 13; cone.position.y = -5; //(13, -5)
    this->_cones.cone_detections.push_back(cone);

    std::cout << "Cone detections size: " << this->_cones.cone_detections.size() << std::endl;

    this->run();
    exit(0);
    #endif

}

void NewPlannerHandle::advertiseToTopics() {
    this->_pubPath = this->_nodeHandle.advertise<nav_msgs::Path>("/control/path_planner/centerline", 1);
}

void NewPlannerHandle::publishToTopics() {
    this->_pubPath.publish(this->_path);
}

void NewPlannerHandle::subscribeToTopics() {
    this->_subCones = this->_nodeHandle.subscribe("/estimation/slam/cone_detections", 1, &NewPlannerHandle::coneDetections, this);
}

void NewPlannerHandle::loadParameters() {
    int fail = 0;

    fail += !ros::param::get("/new_planner/particle_count", _particleCount);
    fail += !ros::param::get("/new_planner/distribution_angle", _distributionAngle);
    fail += !ros::param::get("/new_planner/distribution_radius", _distributionRadius);
    fail += !ros::param::get("/new_planner/search_region/radius", _searchRegionRadius);
    fail += !ros::param::get("/new_planner/step_count", _stepCount);

    if (fail){
        ROS_WARN("A failure occured during parameters loading");
        exit(EXIT_FAILURE);
    }
}

void NewPlannerHandle::run() {
    this->computePath();
    this->publishToTopics();
}

void NewPlannerHandle::coneDetections(const common_msgs::ConeDetections &conesDetected) {
    this->_cones = conesDetected;
    this->run();
}

void NewPlannerHandle::computePath() {
    //place to store close cones
    static std::vector<common_msgs::Cone> closeCones;

    geometry_msgs::PoseStamped stamped;
    geometry_msgs::Pose pose;
    
    //not enough cones
    if (this->_cones.cone_detections.size() <= 3)
        return;

    //clear previous path, update header with new detections, add (0, 0) as 0th pose
    this->_path.poses.clear();
    this->_path.header = this->_cones.header;
    this->_path.poses.push_back(stamped);

    // loop _stepCount times (each step is a 1m segment)
    for (size_t i = 0; i < this->_stepCount; i++)
    {   
        closeCones.clear();

        //get cones within the search region
        for (auto &cone: this->_cones.cone_detections){
            if (this->coneInRegion(pose, cone))
                closeCones.push_back(cone);
        }

        //if empty, break
        if (closeCones.empty())
            break;

        // loop over all particles
        for (auto &particle: this->_particles)
        {
            //sample new particle
            particle.sample(this->_distributionRadius, this->_distributionAngle, pose);

            //calculate particle weight
            particle.weight(closeCones);
        }

        //keep particle with lowest weight
        pose = std::min_element(this->_particles.begin(), this->_particles.end(),
                                [&] (Particle &a, Particle &b){
                                        return a.getWeight() < b.getWeight();
                                    })->getPose();

        //push back new pose
        stamped.pose = pose;
        stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.orientation.z);
        this->_path.poses.push_back((stamped));   
    }
}

bool NewPlannerHandle::coneInRegion (geometry_msgs::Pose pose, common_msgs::Cone cone) {
    int flag = 0;
    
    flag += !(hypot(cone.position.x - pose.position.x, cone.position.y - pose.position.y) 
            < this->_searchRegionRadius);

    float angle = atan2(cone.position.y - pose.position.y, cone.position.x - pose.position.x) - pose.orientation.z; 

    flag += !(angle < M_PI_2);
    flag += !(angle > -M_PI_2);

    return !flag;
}