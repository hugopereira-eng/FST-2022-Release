#include "fastslam/fastslam_handle.hpp"

typedef boost::packaged_task<int> update_task_t;
typedef boost::shared_ptr<update_task_t> pupdate_task_t;

FastSlamHandle::FastSlamHandle(ros::NodeHandle &nodeHandle): _nodeHandle(nodeHandle)
    , _ioServiceLock(ioService)
    , _mapFrameId(nodeHandle.param<std::string>("map_frame_id", "map"))
    , _mapLocation(nodeHandle.param<std::string>("map_location", ""))
    , _loadTrack(nodeHandle.param<bool>("load_track", true))
    , _loadTrackName(nodeHandle.param<std::string>("load_track_name", ""))
    , _deleteCones(nodeHandle.param<bool>("delete_cones_when_left_fov", false))
    , _associateSameColorOnly(nodeHandle.param<bool>("associate_same_color_only", false))
    , _observationIncrement(nodeHandle.param<int>("observation_increment", 5))
    , _particleResampleFactor(nodeHandle.param<float>("particle_resample_factor", 0.5))
    , _loopClosureParticleFactor(nodeHandle.param<float>("loop_closure_particle_factor", 0.3))
    , _coneRadius(nodeHandle.param<float>("cone_radius", 0.0))
    , _particleCount(nodeHandle.param<int>("particle_count", 100))
{
    MotionModel::setStandardDeviations(nodeHandle.param<float>("motion_std_dev_x", 0.1),
                                       nodeHandle.param<float>("motion_std_dev_y", 0.05),
                                       nodeHandle.param<float>("motion_std_dev_theta", 0.02));

    setupThreadPool(nodeHandle.param<int>("thread_count", 4));
    advertiseToTopics();
    subscribeToTopics();
    init();
}

void FastSlamHandle::advertiseToTopics() {
    _pubSlamCones = _nodeHandle.advertise<visualization_msgs::MarkerArray>("/estimation/slam/landmarks", 1);
    _pubSlamConesAutoTests = _nodeHandle.advertise<common_msgs::Track>("/estimation/slam/cones_autotests", 1);
    _pubSlamLoopClosure = _nodeHandle.advertise<std_msgs::Bool>("/estimation/slam/loop_closure", 1);
    _pubFrontFacingCones = _nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/estimation/slam/vis/cones_front", 1);    
    _pubSlamFrontConeDetections = _nodeHandle.advertise<common_msgs::ConeDetections>("/estimation/slam/coneFrontDetections", 1);    
    _pubRelativeCones = _nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/estimation/slam/relative_cones", 1);
    _pubRelativeMeasurements = _nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/estimation/slam/relative_measurements", 1);
    _pubParticles = _nodeHandle.advertise<geometry_msgs::PoseArray>("/estimation/slam/particles", 1);
    _pubOdometry = _nodeHandle.advertise<nav_msgs::Odometry>("/estimation/slam/odometry", 1);
    _pubCenterLine = _nodeHandle.advertise<nav_msgs::Path>("/estimation/slam/centerline", 1);
}

void FastSlamHandle::subscribeToTopics() {
    _subOdom = _nodeHandle.subscribe("/estimation/state_estimation/velocity", 1, &FastSlamHandle::odometryCallback, this);
    _subCarPose = _nodeHandle.subscribe("/estimation/state_estimation/position", 1, &FastSlamHandle::poseCallback, this);
    _subConeDetections = _nodeHandle.subscribe("/perception/cone_detections", 1, &FastSlamHandle::coneDetectionsCallback, this);
}

void FastSlamHandle::odometryCallback(const common_msgs::CarVelocity &odo) {
    OdoMeasurement odoMeasurement;
    odoMeasurement.val = Eigen::Vector3f(static_cast<float>(odo.velocity.x),
                                          static_cast<float>(odo.velocity.y),
                                          static_cast<float>(odo.velocity.theta));
    odoMeasurement.timestamp = odo.header.stamp;
    odoMeasurements.push_back(odoMeasurement);
    
     float delta = static_cast<float>((odo.header.stamp - lastUpdateTime).toSec());

    if (firstUpdate) {
        delta = 0;
        firstUpdate = false;
    }
    Control control;
    control.vx = odoMeasurement.val.x();
    control.vy = odoMeasurement.val.y();
    control.dtheta = odoMeasurement.val.z();
    for (Particle &p : particles) {
        p.updatePose(control, delta);
    }    
    publishOdometry(odoMeasurement.timestamp);
    lastUpdateTime = odo.header.stamp;
}

void FastSlamHandle::poseCallback(const common_msgs::CarPose &position){
    // _pose.pos.x() = position.x;
    // _pose.pos.y() = position.y;
    // _pose.theta = position.theta;
}

void FastSlamHandle::coneDetectionsCallback(const common_msgs::ConeDetections &conesDetected) {

    std::vector<Observation> observations;
    _detectionsHeader = conesDetected.header;

    // transform ConeDetections (x, y, z, color) into Observations (d, r, color)
    std::transform(conesDetected.cone_detections.begin(), conesDetected.cone_detections.end(), std::back_inserter(observations),
                   [&](common_msgs::Cone cone) -> Observation {
                       float d = hypot(cone.position.x, cone.position.y) + _coneRadius;
                       return Observation(d, atan2f(cone.position.y, cone.position.x), cone.color);
                   });

    // sort observations by distance
    std::sort(observations.begin(), observations.end(),
              [](Observation &a, Observation &b) -> bool { return a.z.x() < b.z.x(); });


    // run main update function
    update(observations, conesDetected.header.stamp);
}

void FastSlamHandle::setupThreadPool(size_t count) {
    // start worker threads
    for (size_t i = 0; i < count; ++i)
    {
        workerThreads.create_thread(boost::bind(&boost::asio::io_service::run, &ioService));
    }
}

void FastSlamHandle::init() {
     loadMap(_nodeHandle);
    if (_loadTrackName == "skidpad") {
        std::cout << "SIZE -> " << _landmarks.size()  << " | " << _centerline.poses.size() << std::endl;
        if (!_centerline.poses.empty() && !_landmarks.empty()) {
            Pose initialPosition;
            initialPosition.pos = Eigen::Vector2f(-16.5, 0);
            initialPosition.theta = 0;
            initParticles(initialPosition);
            mostLikelyParticle = *particles.begin();
            currentSlamPhase = SLAM_PHASE_LOCALIZATION;
            ROS_WARN_STREAM("Skidpad Successfully loaded");
        } else {
            ROS_ERROR_STREAM("Fast Slam could not load skidpad map file");
        }
        
    } else {
        if (!_centerline.poses.empty() && !_landmarks.empty()) {
            Pose initialPosition;
            initialPosition.pos = Eigen::Vector2f(0, 0);
            initialPosition.theta = 0;
            initParticles(initialPosition);
            mostLikelyParticle = *particles.begin();
            currentSlamPhase = SLAM_PHASE_LOCALIZATION;
            ROS_WARN_STREAM("Trackdrive Successfully loaded");
        } else {
            ROS_ERROR_STREAM("Fast Slam could not load trackdrive map file");
        }
        // initParticles(initialPosition);
    }
    publishCones(); // do this for debug purposes
    return;
}

void FastSlamHandle::initParticles(Pose initialPosition) {
    Particle p;
    particles.clear();
    particles.reserve(_particleCount);

    std::normal_distribution<float> posx(initialPosition.pos.x(), 1);
    std::normal_distribution<float> posy(initialPosition.pos.y(), 1);
    // create particles with normal distributed positions around the origin
    for (size_t i = 0; i < _particleCount; i++) {
        Particle particle(_deleteCones, _observationIncrement, _associateSameColorOnly);
        particle.pose.pos = Eigen::Vector2f(posx(rng), posy(rng));
        particle.pose.theta = std::normal_distribution<float>(initialPosition.theta, static_cast<float>(10.0f * M_PI / 180.0f))(rng);
        if(!_landmarks.empty()) {
            for(Landmark &landmark: _landmarks) {
                particle.landmarks.push_back(landmark);
            }
        }
        particles.push_back(particle);
    }
}

void FastSlamHandle::loadMap(ros::NodeHandle &nodeHandle) {
    _landmarks.clear();
    _centerline.poses.clear();
    Eigen::Matrix2f observationNoise;
    observationNoise << 0.4f, 0, 0, 0.4f;

    XmlRpc::XmlRpcValue coneArray;
    std::vector<Eigen::Vector2d> yellowCones, blueCones, leftOrangeCones, rightOrangeCones, bigOrangeCones, centerline;
    
    //Load Yellow Cones
    nodeHandle.getParam("/map/cones/yellow_cones", coneArray);
    convertXMLRPCVector(yellowCones, coneArray);
    convert2Landmark(yellowCones, _landmarks, YELLOW_CONE, LANDMARK_SIDE_LEFT, _observationIncrement, observationNoise);
    
    //Load Blue Cones
    nodeHandle.getParam("/map/cones/blue_cones", coneArray);
    convertXMLRPCVector(blueCones, coneArray);
    convert2Landmark(blueCones, _landmarks, BLUE_CONE, LANDMARK_SIDE_RIGHT, _observationIncrement, observationNoise);
    
    // //Load Orange Cones
    nodeHandle.getParam("/map/cones/orange_cones", coneArray);
    convertXMLRPCVector(leftOrangeCones, coneArray);
    convert2Landmark(leftOrangeCones, _landmarks, ORANGE_CONE, LANDMARK_SIDE_UNKNOWN, _observationIncrement, observationNoise);
    
    //Load Big Orange Cones
    nodeHandle.getParam("/map/cones/big_orange_cones", coneArray);
    convertXMLRPCVector(bigOrangeCones, coneArray);
    convert2Landmark(bigOrangeCones, _landmarks, BIG_ORANGE_CONE, LANDMARK_SIDE_UNKNOWN, _observationIncrement, observationNoise);
    
    //Load Centerline
    nodeHandle.getParam("/map/centerline", coneArray);
    convertXMLRPCVector(centerline, coneArray);
    convert2CenterPoint(centerline, _centerline);
    _centerline.header.frame_id = "map";
    _centerline.header.stamp = ros::Time::now();    
}

int updateParticle(Particle *particle, Control &control, float delta, std::vector<Observation> observations, SLAM_PHASE slamPhase) {
    particle->updateLandmarks(observations, slamPhase);
    return 0;
}

void FastSlamHandle::update(std::vector<Observation> observations, ros::Time observationTime) {

    Eigen::Vector3f m = Eigen::Vector3f::Zero();
    float accUpdateTime = 0;

    // // find all odometry measurements between lastUpdateTime and observationTime + one more
    // // this is used to calculate the movement in between the last updates
    for (size_t i = 0; i < odoMeasurements.size(); i++) {
        OdoMeasurement &odo = odoMeasurements[i];
        // remove older measurements than 0.5s
        if ((odo.timestamp - lastUpdateTime).toSec() < -0.5f) {
            odoMeasurements.erase(odoMeasurements.begin() + i);
            i--;
        } else if (odo.timestamp > lastUpdateTime) {
            // this is an interesting odometry update since it occured after the last update time
            ros::Time start, end;
            bool breakFlag = false;
            if (i == 0 || odoMeasurements[i - 1].timestamp < lastUpdateTime) {
                start = lastUpdateTime;
                if (odo.timestamp > observationTime)
                    end = observationTime;
                else
                    end = odo.timestamp;
            } else {
                start = odoMeasurements[i - 1].timestamp;
                if (odo.timestamp > observationTime) {
                    end = observationTime;
                    breakFlag = true;
                } else {
                    end = odo.timestamp;
                }
            }
            m += odo.val * (end - start).toSec();
            accUpdateTime += (end - start).toSec();
            std::cout << "CARALHO " << accUpdateTime << std::endl;
            if (breakFlag)
                break;
        }
    }

    Control control;

    // if the update time is sufficiently large actually fill the control
    if (accUpdateTime >= 1e-10)
        control = Control(m / accUpdateTime);
    control.dtheta = clamp_angle_pi_pi(control.dtheta);

    // printf("control: %f %f %f\n", control.vx, control.vy, control.dtheta);
    float delta;
    // lastUpdateTime = observationTime;

    // distribute particle updates over thread pool
    std::vector<boost::shared_future<int>> pendingJobs;
    for (auto &p : particles) {
        pupdate_task_t task = boost::make_shared<update_task_t>(boost::bind(&updateParticle, &p, control, delta, observations, currentSlamPhase));
        boost::shared_future<int> fut(task->get_future());
        pendingJobs.push_back(fut);
        ioService.post(boost::bind(&update_task_t::operator(), task));
    }

    // wait for all updates to be done
    boost::wait_for_all(pendingJobs.begin(), pendingJobs.end());

    // get all particle weights and number of loop closures
    int numberLoopClosures = 0;
    Eigen::ArrayXf weights(_particleCount);
    for (size_t i = 0; i < _particleCount; i++) {
        weights(i) = particles[i].weight;
        if (particles[i].loop_closure_detected)
            numberLoopClosures++;
    }

    // calc effective sample size
    float effectiveSampleSize = 1 / ((weights / weights.sum()).square()).sum();

    // only resample particles if the effictive sample size is smaller than the resample factor
    if (effectiveSampleSize <= _particleResampleFactor * _particleCount) {
        particles = resampleParticles();
    } else {
        // only update most likely particle
        std::vector<float> weights;
        weights.reserve(particles.size());

        std::transform(particles.begin(), particles.end(), std::back_inserter(weights),
                       [](Particle p) -> float { return p.weight; });

        auto maxIt = std::max_element(weights.begin(), weights.end());
        mostLikelyParticle = particles[std::distance(weights.begin(), maxIt)];
    }

    // check if loop closure is probable in map building phase
    if (currentSlamPhase == SLAM_PHASE_MAP_BUILDING && numberLoopClosures > _particleCount * _loopClosureParticleFactor) {
        float xDev, yDev, thetaDev;
        particlesPoseStdDeviation(xDev, yDev, thetaDev);
        ROS_INFO_STREAM("Loop closure detected by " << static_cast<float>(numberLoopClosures) / _particleCount * 100 << "%");
        if (sqrtf(xDev * xDev + yDev * yDev) <= 0.2f) {
            // all particles estimate our position better than 0.1m
            std_msgs::Bool temp;
            temp.data = true;
            _pubSlamLoopClosure.publish(temp);

            std::stringstream nameStream;
            nameStream << "map_" << ros::WallTime::now() << ".map";
            // save the best particle to file
            std::ofstream fout(nameStream.str(), std::ios::out | std::ios::binary);
            mostLikelyParticle.save(fout);
            currentSlamPhase = SLAM_PHASE_LOCALIZATION;

            // copy most probable map to all particles
            for (auto &p : particles) {
                p.landmarks = mostLikelyParticle.landmarks;
            }
        }
    }
    _pubCenterLine.publish(_centerline);
    publishOdometry(lastUpdateTime);
    publishCones();
    publishFrontFacingCones();
    publishParticles();
    publishRelativeLandmarksAndMeasurements(observations);
}

std::vector<Particle> FastSlamHandle::resampleParticles() {
    std::vector<Particle> newParticles;
    std::vector<float> weights;
    // reserve space for new particles
    newParticles.reserve(particles.size());
    weights.reserve(particles.size());

    // fill weights
    std::transform(particles.begin(), particles.end(), std::back_inserter(weights),
                   [](Particle p) -> float { return p.weight; });

    std::uniform_real_distribution<float> dist(0, 1);
    size_t idx = (size_t)(dist(rng) * particles.size());
    float beta = 0;
    auto maxIt = std::max_element(weights.begin(), weights.end());
    float maxWeight = *maxIt;

    // perform weighted random samping
    for (size_t i = 0; i < particles.size(); i++) {
        beta += dist(rng) * 2 * maxWeight;

        while (beta > weights[idx]) {
            beta -= weights[idx];
            idx = (idx + 1) % particles.size();
        }

        Particle new_p = particles[idx];
        new_p.weight = 1;
        newParticles.push_back(new_p);
    }
    // determine most likely particle
    mostLikelyParticle = particles[std::distance(weights.begin(), maxIt)];
    return newParticles;
}

/**
 * 
 *  SKIDPAD CENTER LINE
 * 
 */
void FastSlamHandle::skidAccCenterLine() {
    // use the most likely particle
    Particle &p = mostLikelyParticle;

    // vectors for left and right cones
    std::vector<Eigen::Vector2f> leftCones;
    std::vector<Eigen::Vector2f> rightCones;
    nav_msgs::Path centerline;
    geometry_msgs::PoseStamped centerPoint;

    // load the landmarks
    std::vector<Landmark> landmarks = p.landmarks;

    // sort landmarks to left and right
    for (auto &lm : landmarks) {
        if (lm.side == LANDMARK_SIDE_LEFT) {
            leftCones.push_back(lm.mu);
        } else if (lm.side == LANDMARK_SIDE_RIGHT)
            rightCones.push_back(lm.mu);
    }

    for (int i = 0; i < leftCones.size(); i++) {
        centerPoint.pose.position.x = (leftCones[i](0) + rightCones[i](0)) / 2;
        centerPoint.pose.position.y = (leftCones[i](1) + rightCones[i](1)) / 2;
        centerPoint.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        centerPoint.header.frame_id = "map";
        centerline.poses.push_back(centerPoint);
    }
    centerline.header.stamp = ros::Time::now();
    centerline.header.frame_id = "map";
    _pubCenterLine.publish(centerline);
}

/**
 * 
 *  COMPUTE CENTER LINE ON LOOP CLOSURE
 * 
 */ 
void FastSlamHandle::centerline() {

    // use the most likely particle
    Particle &p = mostLikelyParticle;

    // vectors for left and right cones
    std::vector<common_msgs::Cone> leftCones;
    common_msgs::Cone leftCone;
    std::vector<common_msgs::Cone> rightCones;
    common_msgs::Cone rightCone;
    nav_msgs::Path centerline;
    centerline.poses.clear();

    // load the landmarks
    std::vector<Landmark> landmarks = p.landmarks;
    // sort landmarks to left and right
    for (auto &lm : landmarks) {
        if (lm.num_observed <= 0)
            continue;
            
        if ((lm.side == LANDMARK_SIDE_LEFT && lm.color != ORANGE_CONE)|| lm.color == BLUE_CONE ) {
            leftCone.position.x = lm.mu(0);
            leftCone.position.y = lm.mu(1);
            leftCone.position.z = 0.0;
            leftCones.push_back(leftCone);
        } else if ((lm.side == LANDMARK_SIDE_RIGHT && lm.color != ORANGE_CONE) || lm.color == YELLOW_CONE) {
            rightCone.position.x = lm.mu(0);
            rightCone.position.y = lm.mu(1);
            rightCone.position.z = 0.0;
            rightCones.push_back(rightCone);
        }
    }

    //for every yellow, find closest blue
    for (const auto &yellow : rightCones) {
        const auto itBlue = std::min_element(leftCones.begin(), leftCones.end(),
            [&](const common_msgs::Cone &a,
                const common_msgs::Cone &b) {
                const double da = std::hypot(yellow.position.x - a.position.x, yellow.position.y - a.position.y);
                const double db = std::hypot(yellow.position.x - b.position.x, yellow.position.y - b.position.y);
                return da < db;
            });

        geometry_msgs::PoseStamped p;
        p.pose.position.x = static_cast<float>((yellow.position.x + itBlue->position.x) / 2.0);
        p.pose.position.y = static_cast<float>((yellow.position.y + itBlue->position.y) / 2.0);
        p.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        centerline.poses.push_back(p);
    }

    //for every blue, find closest yellow
    for (const auto &blue : leftCones) {
        const auto itYellow = std::min_element(rightCones.begin(), rightCones.end(),
            [&](const common_msgs::Cone &a,
                const common_msgs::Cone &b) {
                const double da = std::hypot(blue.position.x - a.position.x, blue.position.y - a.position.y);
                const double db = std::hypot(blue.position.x - b.position.x, blue.position.y - b.position.y);
                return da < db;
            });

        geometry_msgs::PoseStamped p;
        p.pose.position.x = static_cast<float>((blue.position.x + itYellow->position.x) / 2.0);
        p.pose.position.y = static_cast<float>((blue.position.y + itYellow->position.y) / 2.0);
        p.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        centerline.poses.push_back(p);
    }

    //sort by distance to origin
    std::sort(centerline.poses.begin(), centerline.poses.end(), 
              [](geometry_msgs::PoseStamped &a, geometry_msgs::PoseStamped &b) -> bool { 
                    return hypot(a.pose.position.x, a.pose.position.y) < hypot(b.pose.position.x, b.pose.position.y); 
                });
      
    std::vector<geometry_msgs::PoseStamped>::iterator iter = centerline.poses.begin() +1;

    //remove duplicates
    for (; iter != centerline.poses.end();){
        geometry_msgs::Point p1 = iter->pose.position;
        geometry_msgs::Point p2 = (iter-1)->pose.position;
        if (p1.x == p2.x && p1.y == p2.y){
            iter = centerline.poses.erase(iter);
        }
        else {
            iter++;
        }
    }

    _newPath.poses.clear();
    _newPath.poses = centerline.poses;
    _newPath.header.stamp = ros::Time::now();
    _newPath.header.frame_id = "map";

    std::vector<geometry_msgs::PoseStamped>::iterator j = centerline.poses.begin();
    std::vector<geometry_msgs::PoseStamped>::iterator k = _newPath.poses.begin();

    k->pose = centerline.poses[0].pose;
    centerline.poses.erase(centerline.poses.begin());

    for(int i = 0; !centerline.poses.empty(); i++) {
        float minDistance = INFINITY;
        std::vector<geometry_msgs::PoseStamped>::iterator del;
        geometry_msgs::PoseStamped p;

        for (j = centerline.poses.begin(); j!= centerline.poses.end(); j++) {
            float distance = hypot(k->pose.position.x - j->pose.position.x, k->pose.position.y - j->pose.position.y);
            if(distance < minDistance) {
                if (!i && j->pose.position.x < k->pose.position.x) continue;
                minDistance = distance;
                del = j;
            }
        }

        k++;
        k->pose = del->pose;
        k->pose.orientation = tf::createQuaternionMsgFromYaw(0);

        centerline.poses.erase(del);
    }

    _newPath.poses.push_back(_newPath.poses[0]);

    _pubCenterLine.publish(_newPath);
}

/**
 * 
 *  MOTION MODEL SHIT
 * 
 */

void FastSlamHandle::particlePoses(Eigen::ArrayXf &x, Eigen::ArrayXf &y, Eigen::ArrayXf &theta) {
    x.resize(_particleCount);
    y.resize(_particleCount);
    theta.resize(_particleCount);

    for (size_t i = 0; i < _particleCount; i++) {
        x[i] = particles[i].pose.pos.x();
        y[i] = particles[i].pose.pos.y();
        theta[i] = particles[i].pose.theta;
    }
}

void FastSlamHandle::tempParticlePoses(Eigen::ArrayXf &x, Eigen::ArrayXf &y, Eigen::ArrayXf &theta) {
    x.resize(_particleCount);
    y.resize(_particleCount);
    theta.resize(_particleCount);

    for (size_t i = 0; i < _particleCount; i++) {
        x[i] = particles[i].temp_pose.pos.x();
        y[i] = particles[i].temp_pose.pos.y();
        theta[i] = particles[i].temp_pose.theta;
    }
}

void FastSlamHandle::particlesPoseMeanAndStdDev(Eigen::Vector3f &mean, Eigen::Vector3f &std, Eigen::ArrayXf &x, Eigen::ArrayXf &y, Eigen::ArrayXf &theta) {
    mean.x() = x.sum() / _particleCount;
    mean.y() = y.sum() / _particleCount;
    mean.z() = theta.sum() / _particleCount;

    float xSqrdSum = (x - mean.x()).square().sum();
    float ySqrdSum = (y - mean.y()).square().sum();
    float tSqrdSum = (theta - mean.z()).square().sum();

    if (xSqrdSum != 0)
        std.x() = sqrtf(xSqrdSum / _particleCount);

    if (ySqrdSum != 0)
        std.y() = sqrtf(ySqrdSum / _particleCount);

    if (tSqrdSum != 0)
        std.z() = sqrtf(tSqrdSum / _particleCount);
}

void FastSlamHandle::particlesPoseMeanAndStdDev(Eigen::Vector3f &mean, Eigen::Vector3f &std) {
    Eigen::ArrayXf x, y, theta;
    particlePoses(x, y, theta);
    particlesPoseMeanAndStdDev(mean, std, x, y, theta);
}

void FastSlamHandle::tempParticlesPoseMeanAndStdDev(Eigen::Vector3f &mean, Eigen::Vector3f &std) {
    Eigen::ArrayXf x, y, theta;
    tempParticlePoses(x, y, theta);
    particlesPoseMeanAndStdDev(mean, std, x, y, theta);
}

void FastSlamHandle::particlesPoseStdDeviation(float &x_std, float &y_std, float &theta_std) {
    Eigen::Vector3f mean, std;
    particlesPoseMeanAndStdDev(mean, std);
    x_std = std.x();
    y_std = std.y();
    theta_std = std.z();
}

/**
 * 
 * PUBLISH SHIT
 * 
 */
void FastSlamHandle::publishCones() {
   visualization_msgs::MarkerArray coneMarkers;
    visualization_msgs::Marker marker;
    marker.header.frame_id ="map";
    marker.action = visualization_msgs::Marker::DELETEALL;
    coneMarkers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE;    
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0);
    marker.header.stamp = _detectionsHeader.stamp;

    for (int i = 0; i < mostLikelyParticle.landmarks.size(); i++) {
        Landmark landmark = mostLikelyParticle.landmarks[i];
        if(landmark.num_observed <= 0) 
            continue; // dont publish cones that were only observed once
        marker.pose.position.x = landmark.mu.x();
        marker.pose.position.y = landmark.mu.y();
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
        coneMarkers.markers.push_back(marker);
    }
    _pubSlamCones.publish(coneMarkers);
    std::cout << "Published SLAM Cones" << std::endl;
}

void FastSlamHandle::publishFrontFacingCones() {
    pcl::PointCloud<pcl::PointXYZRGB> cones;
    common_msgs::Cone cone;
    std::vector<common_msgs::Cone> conesArray;
    common_msgs::ConeDetections slamFrontConeDetections;

    auto coneToPoint = [](float theta, float dNorm, int color) {
        pcl::PointXYZRGB p;
        p.x = cosf(theta) * dNorm;
        p.y = sinf(theta) * dNorm;
        p.z = 0;
        uint32_t rgb = getColor(color);
        p.rgb = *reinterpret_cast<float *>(&rgb);
        return p;
    };

    auto landmark_to_point = [&coneToPoint](Landmark *lm, Pose p) {
        Eigen::Vector2f d = lm->mu - p.pos;
        float theta = clamp_angle_pi_pi(atan2f(d.y(), d.x()) - p.theta);
        return coneToPoint(theta, d.norm(), lm->color);
    };

    float nearestLandmarkLeftDistance = std::numeric_limits<float>::max();
    float nearestLandmarkRightDistance = std::numeric_limits<float>::max();
    Landmark *nearestLandmarkLeft = nullptr;
    Landmark *nearestLandmarkRight = nullptr;

    Pose p = mostLikelyParticle.pose;
    pcl::PointXYZRGB point;

    for (auto &lm : mostLikelyParticle.landmarks) {
        if (lm.num_observed <= _observationIncrement)
            continue; // dont publish landmarks that we observed only once. good landmarks will be seen more often

        Eigen::Vector2f d = (lm.mu - p.pos);
        float dNorm = d.norm();
        float theta = clamp_angle_pi_pi(atan2f(d.y(), d.x()) - p.theta);

        // check if landmark is in front of car and not more than 12m away, or the nearest landmark on either side behind
        if (theta > -M_PI_2 && theta < M_PI_2 && dNorm <= 12) {
            point = coneToPoint(theta, dNorm, lm.color);
            cones.push_back(point);
            cone.position.x = point.x;
            cone.position.y = point.y;
            cone.position.z = point.z;

            if (lm.color == UNKNOWN_CONE){
                if (lm.side == LANDMARK_SIDE_LEFT)
                    cone.color = BLUE_CONE;
                else if (lm.side == LANDMARK_SIDE_RIGHT)
                    cone.color = YELLOW_CONE;
                else 
                    cone.color = lm.color;
            } else
                cone.color = lm.color;

            conesArray.push_back(cone);
        } else if (theta > M_PI_2 && dNorm < nearestLandmarkLeftDistance) {
            nearestLandmarkLeftDistance = dNorm;
            nearestLandmarkLeft = &lm;
        } else if (theta < -M_PI_2 && dNorm < nearestLandmarkRightDistance) {
            nearestLandmarkRightDistance = dNorm;
            nearestLandmarkRight = &lm;
        }
    }

    // add the nearest landmarks from behind for better boundary performance
    if (nearestLandmarkLeft) {
        point = landmark_to_point(nearestLandmarkLeft, p);
        cones.push_back(point);
    }
    if (nearestLandmarkRight) {
        point = landmark_to_point(nearestLandmarkRight, p);
        cones.push_back(point);
    }

    cones.header.frame_id = _detectionsHeader.frame_id;
    cones.header.stamp = _detectionsHeader.stamp.toNSec() / 1e3;
    _pubFrontFacingCones.publish(cones);
    slamFrontConeDetections.cone_detections = conesArray;
    slamFrontConeDetections.header.frame_id = _detectionsHeader.frame_id;
    slamFrontConeDetections.header.stamp = _detectionsHeader.stamp;
    _pubSlamFrontConeDetections.publish(slamFrontConeDetections);
}

void FastSlamHandle::publishRelativeLandmarksAndMeasurements(std::vector<Observation> &observations) {
    pcl::PointCloud<pcl::PointXYZRGB> cones;
    pcl::PointCloud<pcl::PointXYZRGB> measurements;

    auto coneToPoint = [](Landmark &lm, Pose &pose) {
        pcl::PointXYZRGB p;
        float dx = lm.mu.x() - pose.pos.x();
        float dy = lm.mu.y() - pose.pos.y();
        p.x = cosf(-pose.theta) * dx - sinf(-pose.theta) * dy;
        p.y = sinf(-pose.theta) * dx + cosf(-pose.theta) * dy;
        p.z = 0;
        uint32_t rgb = getColor(lm.color);
        p.rgb = *reinterpret_cast<float *>(&rgb);
        return p;
    };

    auto observationToPoint = [](Observation &ob) {
        pcl::PointXYZRGB p;
        p.x = cosf(ob.z.y()) * ob.z.x();
        p.y = sinf(ob.z.y()) * ob.z.x();
        p.z = 0;
        uint32_t rgb = getColor(ob.color);
        p.rgb = *reinterpret_cast<float *>(&rgb);
        return p;
    };

    for (auto &lm : mostLikelyParticle.landmarks) {
        if (lm.num_observed <= _observationIncrement)
            continue; // dont publish landmarks that we observed only once. good landmarks will be seen more often
        cones.push_back(coneToPoint(lm, mostLikelyParticle.pose));
    }
    cones.header.frame_id = _detectionsHeader.frame_id;
    cones.header.stamp = _detectionsHeader.stamp.toNSec() / 1e3;
    _pubRelativeCones.publish(cones);

    for (auto &ob : observations) {
        measurements.push_back(observationToPoint(ob));
    }
    measurements.header.frame_id = _detectionsHeader.frame_id;
    measurements.header.stamp = cones.header.stamp;
    _pubRelativeMeasurements.publish(measurements);
}

void FastSlamHandle::publishParticles() {
    geometry_msgs::PoseArray poses;

    for (auto &pa : particles) {
        geometry_msgs::Pose pose;
        tf::Quaternion rot;

        rot.setEuler(0, 0, pa.pose.theta);
        pose.position.x = pa.pose.pos.x();
        pose.position.y = pa.pose.pos.y();
        tf::quaternionTFToMsg(rot, pose.orientation);

        poses.poses.push_back(pose);
    }
    poses.header.stamp = ros::Time::now();
    poses.header.frame_id = "map";
    _pubParticles.publish(poses);
}

void FastSlamHandle::publishOdometry(ros::Time time) {
    nav_msgs::Odometry odom;
    Eigen::Vector3f mean(0, 0, 0), std(0, 0, 0);

    odom.header.frame_id = "map";
    odom.header.stamp = time;

    tempParticlesPoseMeanAndStdDev(mean, std);

    odom.pose.pose.position.x = mean.x();
    odom.pose.pose.position.y = mean.y();
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(mean.z());

    if (std::isnan(std.x()) || std::isnan(std.y()) || std::isnan(std.x())) {
        std::cout << mean << std::endl
                  << std << std::endl;
        return; // something went really wrong here;
    }

    odom.pose.covariance = {std.x() * std.x(), 0, 0, 0, 0, 0,
                            0, std.y() * std.y(), 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, std.z() * std.z()};

    _pubOdometry.publish(odom);
    publishTf(mean.x(), mean.y(), mean.z());
}

void FastSlamHandle::publishTf(const float x, const float y, const float yaw) {
    // Position
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0.0));

    // Orientation
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    transform.setRotation(q);

    // Send TF
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_footprint"));
}
