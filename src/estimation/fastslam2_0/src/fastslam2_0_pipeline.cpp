#include <ros/ros.h>
#include <fastslam2_0/fastslam2_0_pipeline.hpp>
#include <chrono>

std::default_random_engine FastSlam::rng;

//Constructor
FastSlam::FastSlam(ros::NodeHandle &nodeHandle) {
    ROS_WARN_STREAM("Fast Slam 2.0 loading parameters");

    if(!ros::param::get("fastslam2_0/load_map", _loadMap)) {
        ROS_WARN_STREAM("Could not read SLAM phase, mapping will be assumed");
        _loadMap = false;
    }
    if(!ros::param::get("fastslam2_0/map_path", _mapLocation)) {
        ROS_WARN_STREAM("Could not load map path, saving to home");
        _mapLocation = "/home";
    }

    _particleNumber = nodeHandle.param<int>("particle_number", 50);
    _newLandmarkThreshold = nodeHandle.param<float>("new_landmark_threshold", 0.02);
    _startingHeadingDeviation = nodeHandle.param<float>("starting_heading_deviation", 10.0);
    // _startingHeadingDeviation = (_startingHeadingDeviation / 180.0f * M_PI);
    _observationIncrement = nodeHandle.param<float>("observation_increment", 10);
    _loopClosureFactor = nodeHandle.param<float>("loop_closure_factor", 0.8);
    _loopClosureParticleFactor = nodeHandle.param<float>("loop_closure_particle_factor", 0.3);
    _resampleFactor = nodeHandle.param<float>("resample_factor", 0.5);
    float observationNoiseDistance = nodeHandle.param<float>("observation_noise_distance", 0.4);
    float observationNoiseTheta = nodeHandle.param<float>("observation_noise_theta", 3.5);
    _observationNoise << powf(observationNoiseDistance, 2), 0,
                        0, powf(observationNoiseTheta * M_PI / 180, 2);
    float controlNoiseX = nodeHandle.param<float>("control_noise_x", 0.01);
    float controlNoiseY = nodeHandle.param<float>("control_noise_y", 0.01);
    float controlNoiseTheta = nodeHandle.param<float>("control_noise_theta", 0.01);
    _controlNoise << powf(controlNoiseX, 2), 0, 0,
                     0, powf(controlNoiseY, 2), 0, 
                     0, 0, powf(controlNoiseTheta * M_PI / 180, 2);
    _mapName = nodeHandle.param<std::string>("map_name", "trackdrive");

    if(_loadMap) {
        loadMap(nodeHandle);     
    }

    if (_mapName == "skidpad") {
        if (!_centerLine.poses.empty() && !_landmarks.empty()) {
            Pose initialPosition;
            initialPosition.s = Eigen::Vector3f(-16.5, 0, 0);
            initParticles(initialPosition);
            _mostLikelyParticle = *_particles.begin();
            _currentSlamPhase = SLAM_PHASE_LOCALIZATION;
            ROS_WARN_STREAM("Skidpad Successfully loaded");
        } else {
            ROS_ERROR_STREAM("Fast Slam could not load skidpad map file");
        }
    } else {
        if (!_centerLine.poses.empty() && !_landmarks.empty()) {
            _currentSlamPhase = SLAM_PHASE_LOCALIZATION; 
            ROS_WARN_STREAM("Map Successfully loaded, Localization Phase");
        } else {
            _currentSlamPhase = SLAM_PHASE_MAP_BUILDING;
            ROS_WARN_STREAM("Fast Slam could not load map file, mapping phase will be assumed");            
        }
        Pose initialPosition;
        initialPosition.s = Eigen::Vector3f(0, 0, 0);
        initParticles(initialPosition);
        _mostLikelyParticle = *_particles.begin();
    }
}

//Setters
void FastSlam::updateOdometry(const common_msgs::CarPose &odom) {
    // ROS_INFO_STREAM("Begin Odometry update");
    // Control control (Eigen::Vector3f(static_cast<float>(odom.velocity.x), static_cast<float>(odom.velocity.y), static_cast<float>(odom.velocity.theta)));
    // float delta;

    // if(_firstUpdate) {
    //     delta = 0;
    //     _firstUpdate = false;
    // }
    // else delta = static_cast<float>((odom.header.stamp - _lastUpdateTime).toSec());
    // Pose newPose;
    // newPose.s = Eigen::Vector3f(odom.x, odom.y, odom.theta);
    // // // std::cout << "DELTA TIME: " << delta << " Vx: " << control.vx << " Vy: " << control.vy << " YawRate: " << control.theta << std::endl;
    // for(Particle &particle : _particles) {
    //     particle.updatePose(newPose);
    // }
    // _lastUpdateTime = odom.header.stamp;
    // ROS_INFO_STREAM("Finish Odometry update");
}

void FastSlam::updateVelocity(const common_msgs::CarVelocity &vel) {
    Control control (Eigen::Vector3f(static_cast<float>(vel.velocity.x), static_cast<float>(vel.velocity.y), static_cast<float>(vel.velocity.theta)));
    
    float delta;
    if(_firstUpdate) {
        delta = 0;
        _firstUpdate = false;
    }
    else delta = static_cast<float>((vel.header.stamp - _lastUpdateTime).toSec());
    
    for(Particle &particle: _particles) {
        particle.updatePose(control, delta);
    }

    _lastUpdateTime = vel.header.stamp;
}

void FastSlam::updateConeDetections(const common_msgs::ConeDetections &coneDetections) {
    //ROS_INFO_STREAM("Begin Cone Detections update");
    std::vector<Observation> observations;
    _detectionsHeader = coneDetections.header;
    // transform ConeDetections (x, y, z, color) into Observations (d, r, color)
    // std::transform(coneDetections.cone_detections.begin(), coneDetections.cone_detections.end(), std::back_inserter(observations),
    //     [&](common_msgs::Cone cone)->Observation {
    //         return cone2Observation(cone, _detectionsHeader.stamp);
    //     });
    if(!coneDetections.cone_detections.empty()) {
        for (auto i = coneDetections.cone_detections.begin(); i != coneDetections.cone_detections.end(); i++) {
            if(i->color == UNKNOWN_CONE &&  hypot(i->position.x, i->position.y) > 5) {
                continue;
            } else {
                float d = hypot(i->position.x, i->position.y); // + _coneRadius;     might add cone radius later
                float bearing = atan2f(i->position.y, i->position.x);
                observations.push_back(Observation(d, bearing, i->color, i->id, _detectionsHeader.stamp));
            }
        }
        // sort observations by distance
        std::sort(observations.begin(), observations.end(),
            [](Observation &a, Observation &b) -> bool { return a.z.x() < b.z.x(); });
        
        // ROS_INFO_STREAM("Finish Cone Detections update");
        updateParticleKalman(observations);    
    }
    
}

void FastSlam::updateCenterline(const nav_msgs::Path &centerline) {
    _centerLine = centerline;
    if(!_centerLine.poses.empty()) {
        saveMap();
    }
}

//Getters
visualization_msgs::MarkerArray const & FastSlam::getLandmarks() const { return _coneMarkers; }
geometry_msgs::PoseArray const & FastSlam::getParticlePoses() const { return _particlePoses; }
nav_msgs::Path const & FastSlam::getCenterLine() const { return _centerLine; }
nav_msgs::Odometry const & FastSlam::getOdometry() const { return _odometry; }
common_msgs::ConeDetections const & FastSlam::getMapCones() const {return _mapConeDetections; }
common_msgs::ConeDetections const & FastSlam::getCurrentConeDetections() const { return _currentConeDetections; }
visualization_msgs::MarkerArray const & FastSlam::getCurrentConeDetectionsVisualization() const { return _currentConeMarkers; }
visualization_msgs::MarkerArray const & FastSlam::getCurrentObservationsVisualization() const { return _currentObservations; }


void FastSlam::initParticles(Pose initialPosition) {
    _particles.clear();
    _particles.reserve(_particleNumber);
    //creating particles with a normal distribution around the initial position
    for(size_t i = 0; i <_particleNumber; i++) {
        Particle particle (_observationIncrement, _loopClosureFactor, _observationNoise, _controlNoise, _newLandmarkThreshold, _startingHeadingDeviation);
        particle._pose.s.x() = std::normal_distribution<float>(initialPosition.s.x(), 0.01)(rng);
        particle._pose.s.y() = std::normal_distribution<float>(initialPosition.s.y(), 0.01)(rng);
        particle._pose.s.z() = std::normal_distribution<float>(initialPosition.s.z(), 0.01)(rng);
        if(!_landmarks.empty()) {
            for(Landmark &landmark: _landmarks) {
                particle._landmarks.push_back(landmark);
            }
        }
        _particles.push_back(particle);
    }
}

void FastSlam::updateParticleKalman(std::vector<Observation> observations) {
    
    for(Particle &particle : _particles) {
        particle.updateLandmarks(observations, _currentSlamPhase);
    }

    // std::cout << "Line 127 - After Update Particle" << std::endl;
    int numberLoopClosures = 0;
    _weights.clear();
    _weights.reserve(_particles.size());
    Eigen::ArrayXf weights(_particles.size());
    for (size_t i = 0; i < _particleNumber; i++) {
        _weights.push_back(_particles[i]._weight);
        weights(i) = _particles[i]._weight;         // temporary, delete later        
        
        if (_particles[i]._loopClosureDetected)
            numberLoopClosures++;
    }
    
    float effSampleSize = 1 / ((weights / weights.sum()).square()).sum();
    
    std::cout << "EFFECTIVE SAMPLE SIZE " << effSampleSize / _particleNumber << std::endl;
    if (effSampleSize < _resampleFactor * _particleNumber || isnan(effSampleSize)) {       // resample if effective sample size lower than resample factor
         _particles = resampleParticles();
         std::cout << "I RESAMPLE" << std::endl;
    }
    else {                                                          // only update most likely particle
        std::vector<float>::iterator maxIt = std::max_element(_weights.begin(), _weights.end());
        _mostLikelyParticle = _particles[std::distance(_weights.begin(), maxIt)];
    }
    std::cout << "Lopp closure " << numberLoopClosures << " --> " << _particleNumber * _loopClosureParticleFactor << std::endl;
    if (_currentSlamPhase == SLAM_PHASE_MAP_BUILDING && numberLoopClosures > _particleNumber * _loopClosureParticleFactor) {
        
        _currentSlamPhase = SLAM_PHASE_LOCALIZATION;

        std::cout << "SLAM LOCALIZING" << std::endl;
        
        for (Particle &particle: _particles) {
            particle._landmarks = _mostLikelyParticle._landmarks;
        }
        computeCenterLine();
    }

    computeOdometry();
    landmarkVisualization();
    particleVisualization();
    currentDetectionsVisualization();
    currentObservationsVisualization(observations);
}

std::vector<Particle> FastSlam::resampleParticles() {

    std::vector<Particle> newParticleSet;
    newParticleSet.reserve(_particles.size());

    std::uniform_real_distribution<float> distribution(0, 1);
    size_t index = (size_t)(distribution(rng) * _particles.size());
    float beta = 0;
    std::vector<float>::iterator maxIt = std::max_element(_weights.begin(), _weights.end());
    float maxWeight = *maxIt;

    // perform weighted random samping
    for (size_t i = 0; i < _particles.size(); i++) {
        beta += distribution(rng) * 2 * maxWeight;

        while (beta > _weights[index]) {
            beta -= _weights[index];
            index = (index + 1) % _particles.size();
        }

        Particle newParticle = _particles[index];
        newParticle._weight = 1;
        newParticleSet.push_back(newParticle);

        // Update most likely particle
        _mostLikelyParticle = _particles[std::distance(_weights.begin(), maxIt)];
    }
    
    return newParticleSet;
}

void FastSlam::computeOdometry() {
    //must clean odometry    
    Eigen::Vector3f mean(0, 0, 0), stdDev (0, 0, 0);

    _odometry.header.frame_id = "map";
    _odometry.header.stamp = _detectionsHeader.stamp;

    //Compute Mean and StdDev
    tempParticlePoseAndStdDev(mean, stdDev);

    _odometry.pose.pose.position.x = mean.x();
    _odometry.pose.pose.position.y = mean.y();
    _odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(mean.z());

    _odometry.pose.covariance = {pow(stdDev.x(), 2), 0, 0, 0, 0, 0,
                            0, pow(stdDev.y(), 2), 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, pow(stdDev.z(), 2)};
    
    publishTf(mean.x(), mean.y(), mean.z());
}

void FastSlam::publishTf(const double x, const double y, const double theta) {
    // Position
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0.0));

    // Orientation
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    transform.setRotation(q);

    // Send TF
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_footprint"));
}

void FastSlam::tempParticlePoseAndStdDev(Eigen::Vector3f &mean, Eigen::Vector3f &std) {
    Eigen::ArrayXf x, y, theta;
    particlePose(x, y, theta);
    particlePoseAndStdDev(mean, std, x, y, theta);

}

void FastSlam::particlePose(Eigen::ArrayXf &x, Eigen::ArrayXf &y, Eigen::ArrayXf &theta) {
    
    x.resize(_particleNumber);
    y.resize(_particleNumber);
    theta.resize(_particleNumber);

    for (size_t i = 0; i < _particleNumber; i++) {
        x[i] = _particles[i]._pose.s.x();
        y[i] = _particles[i]._pose.s.y();
        theta[i] = _particles[i]._pose.s.z();
    }
}

void FastSlam::particlePoseAndStdDev(Eigen::Vector3f &mean, Eigen::Vector3f &std,Eigen::ArrayXf &x, Eigen::ArrayXf &y, Eigen::ArrayXf &theta) {
    
    mean.x() = x.sum() / _particleNumber;
    mean.y() = y.sum() / _particleNumber;
    mean.z() = theta.sum() / _particleNumber;

    float xSquaredSum = (x - mean.x()).square().sum();
    float ySquaredSum = (y - mean.y()).square().sum();
    float thetaSquaredSum = (theta - mean.z()).square().sum();
    
    std.x() = sqrtf(xSquaredSum / _particleNumber);
    std.y() = sqrtf(ySquaredSum / _particleNumber);
    std.z() = sqrtf(thetaSquaredSum / _particleNumber);   

}

void FastSlam::loadMap(ros::NodeHandle &nodeHandle) {
    _landmarks.clear();
    _centerLine.poses.clear();

    XmlRpc::XmlRpcValue coneArray;
    std::vector<Eigen::Vector2d> yellowCones, blueCones, leftOrangeCones, rightOrangeCones, bigOrangeCones, centerline;
    
    //Load Yellow Cones
    nodeHandle.getParam("/map/cones/yellow_cones", coneArray);
    convertXMLRPCVector(yellowCones, coneArray);
    convert2Landmark(yellowCones, _landmarks, YELLOW_CONE, LANDMARK_SIDE_LEFT, _observationIncrement, _observationNoise);
    
    //Load Blue Cones
    nodeHandle.getParam("/map/cones/blue_cones", coneArray);
    convertXMLRPCVector(blueCones, coneArray);
    convert2Landmark(blueCones, _landmarks, BLUE_CONE, LANDMARK_SIDE_RIGHT, _observationIncrement, _observationNoise);
    
    // //Load Orange Cones
    nodeHandle.getParam("/map/cones/orange_cones", coneArray);
    convertXMLRPCVector(leftOrangeCones, coneArray);
    convert2Landmark(leftOrangeCones, _landmarks, ORANGE_CONE, LANDMARK_SIDE_UNKNOWN, _observationIncrement, _observationNoise);
    
    //Load Big Orange Cones
    nodeHandle.getParam("/map/cones/big_orange_cones", coneArray);
    convertXMLRPCVector(bigOrangeCones, coneArray);
    convert2Landmark(bigOrangeCones, _landmarks, BIG_ORANGE_CONE, LANDMARK_SIDE_UNKNOWN, _observationIncrement, _observationNoise);
    
    //Load Centerline
    nodeHandle.getParam("/map/centerline", coneArray);
    convertXMLRPCVector(centerline, coneArray);
    convert2CenterPoint(centerline, _centerLine);
    _centerLine.header.frame_id = "map";
    _centerLine.header.stamp = ros::Time::now();    
}

void FastSlam::computeCenterLine() { 
    _mapConeDetections.cone_detections.clear();
    for(Landmark &landmark: _mostLikelyParticle._landmarks) {
        common_msgs::Cone cone;
        cone.position.x = landmark.mean.x();
        cone.position.y = landmark.mean.y();
        cone.position.z = 0;
        cone.color = landmark.color;
        _mapConeDetections.cone_detections.push_back(cone);
    }
    _mapConeDetections.header.frame_id = "map";
    _mapConeDetections.header.stamp = ros::Time::now();
}

void FastSlam::saveMap() {

    std::ofstream mapFile;
    std::string fileName = _mapLocation + "/trackdrive.yaml";
    mapFile.open(fileName);

    std::vector<Eigen::Vector2d> yellowCones, blueCones, orangeCones, bigOrangeCones;

    for (Landmark &landmark: _mostLikelyParticle._landmarks) {
        if (landmark.color == YELLOW_CONE) {
            yellowCones.push_back(Eigen::Vector2d(landmark.mean.x(), landmark.mean.y()));
        } else if (landmark.color == BLUE_CONE) {
            blueCones.push_back(Eigen::Vector2d(landmark.mean.x(), landmark.mean.y()));
        } else if (landmark.color == ORANGE_CONE) {
            orangeCones.push_back(Eigen::Vector2d(landmark.mean.x(), landmark.mean.y()));
        } else if (landmark.color == BIG_ORANGE_CONE) {
            bigOrangeCones.push_back(Eigen::Vector2d(landmark.mean.x(), landmark.mean.y()));
        }
    }

    mapFile << "map: \n";
    mapFile << "    cones: \n";
    //Yellow Cones
    mapFile << "        yellow_cones: [";
    //mapFile << "        [";

    for (int i = 0; i < yellowCones.size(); i++) {
        if (i == 0) {
            mapFile << "[";
        } else {
            mapFile << "        [";
        }
        mapFile << std::fixed << std::setprecision(3) << yellowCones[i].x();
        mapFile << ", ";
        mapFile << std::fixed << std::setprecision(3) << yellowCones[i].y();
        if (i == yellowCones.size()-1) {
            mapFile << "]";
        } else {
            mapFile << "],\n"; 
        }     
    }
    mapFile << "]\n\n";

    //Blue cones
    mapFile << "        blue_cones: [";
    //mapFile << "        [";

    for (int i = 0; i < blueCones.size(); i++) {
        if (i == 0) {
            mapFile << "[";
        } else {
            mapFile << "        [";
        }
        mapFile << std::fixed << std::setprecision(3) << blueCones[i].x();
        mapFile << ", ";
        mapFile << std::fixed << std::setprecision(3) << blueCones[i].y();
        if (i == blueCones.size()-1) {
            mapFile << "]";
        } else {
            mapFile << "],\n"; 
        }     
    }
    mapFile << "]\n\n";

    //Orange cones
    mapFile << "        orange_cones: [";
    //mapFile << "        [";

    for (int i = 0; i < orangeCones.size(); i++) {
        if (i == 0) {
            mapFile << "[";
        } else {
            mapFile << "        [";
        }
        mapFile << std::fixed << std::setprecision(3) << orangeCones[i].x();
        mapFile << ", ";
        mapFile << std::fixed << std::setprecision(3) << orangeCones[i].y();
        if (i == orangeCones.size()-1) {
            mapFile << "]";
        } else {
            mapFile << "],\n"; 
        }     
    }
    mapFile << "]\n\n";

    //Big Orange cones
    mapFile << "        big_orange_cones: [";
    //mapFile << "        [";
    
    for (int i = 0; i < bigOrangeCones.size(); i++) {
        if (i == 0) {
            mapFile << "[";
        } else {
            mapFile << "        [";
        }
        mapFile << std::fixed << std::setprecision(3) << bigOrangeCones[i].x();
        mapFile << ", ";
        mapFile << std::fixed << std::setprecision(3) << bigOrangeCones[i].y();
        if (i == bigOrangeCones.size()-1) {
            mapFile << "]";
        } else {
            mapFile << "],\n"; 
        }     
    }
    mapFile << "]\n\n";

    //Centerline
    mapFile << "    centerline: [";
    //mapFile << "    [";
    
    for(int i = 0; i < _centerLine.poses.size(); i++) {
        if (i == 0) {
            mapFile << "[";
        } else {
            mapFile << "    [";
        }
        mapFile << std::fixed << std::setprecision(3) << _centerLine.poses[i].pose.position.x;
        mapFile << ", ";
        mapFile << std::fixed << std::setprecision(3) << _centerLine.poses[i].pose.position.y;
        if (i == _centerLine.poses.size()-1) {
            mapFile << "]";
        } else {
            mapFile << "],\n"; 
        } 
    }
    mapFile << "]\n";
    mapFile.close();
}

// Visualization Shit

void FastSlam::landmarkVisualization() {
    
    _coneMarkers.markers.clear();
    visualization_msgs::Marker marker;
    marker.header.frame_id ="map";
    marker.action = visualization_msgs::Marker::DELETEALL;
    _coneMarkers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE;    
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0);
    marker.header.stamp = _detectionsHeader.stamp;

    for (int i = 0; i < _mostLikelyParticle._landmarks.size(); i++) {
        Landmark landmark = _mostLikelyParticle._landmarks[i];
        if(landmark.numObserved <= 0) 
            continue; // dont publish cones that were only observed once
            marker.pose.position.x = landmark.mean.x();
        marker.pose.position.y = landmark.mean.y();
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        marker.pose.position.z = 0;
        marker.id = i + 1;

        if(landmark.color == BLUE_CONE) {
            marker.color.r = 0;
            marker.color.g = 0;
            marker.color.b = 1;
        } else if(landmark.color == YELLOW_CONE) {
            marker.color.r = 1;
            marker.color.g = 1;
            marker.color.b = 0;
        } else if (landmark.color == ORANGE_CONE || landmark.color == BIG_ORANGE_CONE) {
            marker.color.r = 1;
            marker.color.g = 0.65;
            marker.color.b = 0;
        } else {
            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.5;
        }
        _coneMarkers.markers.push_back(marker);
    }        
}

void FastSlam::particleVisualization() {
    _particlePoses.poses.clear();
    
    for (Particle &particle : _particles) {
        geometry_msgs::Pose pose;
        tf::Quaternion rot;

        rot.setEuler(0, 0, particle._pose.s.z());
        pose.position.x = particle._pose.s.x();
        pose.position.y = particle._pose.s.y();
        tf::quaternionTFToMsg(rot, pose.orientation);

        _particlePoses.poses.push_back(pose);
    }
    _particlePoses.header.frame_id = "map";
    _particlePoses.header.stamp = _detectionsHeader.stamp;
}

void FastSlam::currentDetectionsVisualization() {
    
    _currentConeMarkers.markers.clear();
    _currentConeDetections.cone_detections.clear();

    float nearestLandmarkLeftDistance = std::numeric_limits<float>::max();
    float nearestLandmarkRightDistance = std::numeric_limits<float>::max();
    Landmark *nearestLandmarkLeft = nullptr;
    Landmark *nearestLandmarkRight = nullptr;
    int i = 1;
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::DELETEALL;
    _currentConeMarkers.markers.push_back(marker);

    Pose pose = _mostLikelyParticle._pose;

    for(Landmark &landmark : _mostLikelyParticle._landmarks) {
        
        if(landmark.numObserved <= 0) 
            continue; // dont publish cones that were only observed once
        Observation observation = landmark2Observation(&landmark, &pose);
        if(observation.z.y() > -M_PI_2 && observation.z.y() < M_PI_2 &&observation.z.x() <= 15) {
            //Cone Detections
            common_msgs::Cone cone;
            cone = observation2Cone(observation, &landmark);
            _currentConeDetections.cone_detections.push_back(cone);
            
            //Visualization            
            marker.header.stamp = _detectionsHeader.stamp;
            marker.header.frame_id = _detectionsHeader.frame_id;
            marker.ns = "Detections";
            marker.id = i;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;   
            marker.color.a = 1;	
            marker.type = 3;
            marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = cone.position.x;
            marker.pose.position.y = cone.position.y;
            marker.pose.position.z = 0;

            if (landmark.color == YELLOW_CONE) {
                marker.color.r = 1;
                marker.color.g = 1;
                marker.color.b = 0;
            } else if(landmark.color == BLUE_CONE) {
                marker.color.r = 0;
                marker.color.g = 0;
                marker.color.b = 1;
            } else if(landmark.color == ORANGE_CONE || landmark.color == BIG_ORANGE_CONE) {
                marker.color.r = 1;
                marker.color.g = 0.65;
                marker.color.b = 0;
            } else {
                marker.color.r = 0.5;
                marker.color.g = 0.5;
                marker.color.b = 0.5;
            }

            _currentConeMarkers.markers.push_back(marker);
            i++;

        } else if (observation.z.y() > M_PI_2 && observation.z.x() < nearestLandmarkLeftDistance) {
            nearestLandmarkLeftDistance = observation.z.x();
            nearestLandmarkLeft = &landmark;
        } else if (observation.z.y() < -M_PI_2 && observation.z.x() < nearestLandmarkRightDistance) {
            nearestLandmarkRightDistance = observation.z.x();
            nearestLandmarkRight = &landmark;
        }
    }
    
    // add the nearest landmarks from behind for better boundary performance
    if (nearestLandmarkLeft) {
        _currentConeDetections.cone_detections.push_back(observation2Cone(landmark2Observation(nearestLandmarkLeft, &pose), nearestLandmarkLeft));
    }
    if (nearestLandmarkRight) {
        _currentConeDetections.cone_detections.push_back(observation2Cone(landmark2Observation(nearestLandmarkRight, &pose), nearestLandmarkRight));
    }

    _currentConeDetections.header.frame_id = _detectionsHeader.frame_id;
    _currentConeDetections.header.stamp = _detectionsHeader.stamp;
}

void FastSlam::currentObservationsVisualization(std::vector<Observation> observations) {
	
    _currentObservations.markers.clear();
    Pose pose = _mostLikelyParticle._pose;
    
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::DELETEALL;
    _currentObservations.markers.push_back(marker);

	marker.header.stamp = _detectionsHeader.stamp;
	marker.header.frame_id = _detectionsHeader.frame_id;
	marker.ns = "Validated Detections";
	marker.scale.x = 0.5;
	marker.scale.y = 0.5;
	marker.scale.z = 0.5;   
	marker.color.a = 1;	
	marker.type = 2;
	marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);
	//marker.lifetime = ros::Duration(0.05);
	marker.action = visualization_msgs::Marker::ADD;

	for (size_t i = 0; i < observations.size(); ++i) {
        
	   	marker.id = i + 1;
		//marker.pose.position = coneDetections.cone_detections[i].position;

        common_msgs::Cone conePose = observation2Cone(observations[i], &_mostLikelyParticle._landmarks[0]);

        marker.pose.position.x = conePose.position.x;
        marker.pose.position.y = conePose.position.y;
        marker.pose.position.z = 0;

	    if (observations[i].color == YELLOW_CONE) {
	    	marker.color.r = 1;
			marker.color.g = 1;
			marker.color.b = 0;
	    } else if(observations[i].color == BLUE_CONE) {
	    	marker.color.r = 0;
			marker.color.g = 0;
			marker.color.b = 1;
	    } else if(observations[i].color == ORANGE_CONE || observations[i].color == BIG_ORANGE_CONE) {
			marker.color.r = 1;
			marker.color.g = 0.65;
			marker.color.b = 0;
		} else {
			marker.color.r = 0.5;
			marker.color.g = 0.5;
			marker.color.b = 0.5;
		}
        _currentObservations.markers.push_back(marker);
	}


}


