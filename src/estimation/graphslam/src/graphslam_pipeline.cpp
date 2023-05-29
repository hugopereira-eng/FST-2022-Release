#include <graphslam/graphslam_pipeline.hpp>

using namespace Eigen;

#define OFFSET -M_PI/2
float compute_yaw(float forward, float left);
float normalize_angle(float angle);
float sign(float x);
float distance_to_line(float x1,float y1, float x2, float y2, float x0, float y0);

constexpr float HEADING_DEVIATION = (60.0f / 180.0f * M_PI);

std::default_random_engine GraphSlam::rng;

// Getters
visualization_msgs::MarkerArray const & GraphSlam::getLandmarkMarkers() const {return _coneMarkers;}
visualization_msgs::MarkerArray const & GraphSlam::getVertexesMarkers() const {return _daMarkers;} //_graphTypes.getVertexesMarkers()
visualization_msgs::MarkerArray const & GraphSlam::getCurrentObservationsMarkers() const {return _coneObservationMarkers;}
visualization_msgs::MarkerArray const & GraphSlam::getCurrentConeDetectionsMarkers() const {return _currentConeMarkers;}
visualization_msgs::MarkerArray const & GraphSlam::getDataAssociationMarkers() const {return _dataAssociationMarkers;}
visualization_msgs::MarkerArray const & GraphSlam::getConeInfoMarkers() const {return _coneInfoMarkers;}
visualization_msgs::Marker const & GraphSlam::getEdgeMarkers() const {return _edges;} //
common_msgs::ConeDetections const & GraphSlam::getCurrentConeDetections() const {return _currentConeDetections;}
common_msgs::ConeDetections const & GraphSlam::getMapCones() const {return _mapConeDetections;}
nav_msgs::Odometry const & GraphSlam::getOdometry() const {return _odometry;}
nav_msgs::Path const & GraphSlam::getCenterline() const {return _centerLine;}
std_msgs::Bool const & GraphSlam::getLoopClosure() const {return _loopClosureDetected;}
//ICP
sensor_msgs::PointCloud2 const & GraphSlam::getAlignedPoincloud() const { return icp_c.getAlignedPoincloud();}
sensor_msgs::PointCloud2 const & GraphSlam::getSourcePointcloud() const { return icp_c.getSourcePointcloud();}
sensor_msgs::PointCloud2 const & GraphSlam::getTargetPointcloud() const { return icp_c.getTargetPointcloud();}
nav_msgs::Odometry const & GraphSlam::getIcpOdometry() const {return icp_c.getIcpOdometry();}


//Constructor
GraphSlam::GraphSlam(ros::NodeHandle &nh): tfListener(tf_buffer_) {

    loadParameters();
    _dataAssociation = DataAssociation(_daMethod, _observationNoise);
    
    if(_loadMap) loadMap(nh);

    // add first pose vertex to graph
    _graphTypes.addOdometryVertex(_pose, _graphIndex);
    _odometryIndex.push_back(_graphIndex);
    _graphIndex++;
    icp_c = ICP_odom();
}

void GraphSlam::loadParameters() {

    int fail = 0;
    double observationNoiseDistance, observationNoiseTheta;
    double controlNoiseX, controlNoiseY, controlNoiseTheta;
    double initX, initY, initTheta;
    std::string mission, daMethod;

    if(!ros::param::get("graphslam/load_map", _loadMap)) {
        ROS_WARN_STREAM("Could not read SLAM phase, mapping will be assumed");
        _loadMap = false;
    }
    if(!ros::param::get("graphslam/map_path", _mapPath)) {
        ROS_WARN_STREAM("Could not load map path, saving to home");
        _mapPath = "/home";
    }

    //mapping
    fail += !ros::param::get("graphslam/mapping/associationThreshold",      _associationThreshold);
    fail += !ros::param::get("graphslam/mapping/associationRange",          _associationRange);
    fail += !ros::param::get("graphslam/mapping/startingHeadingDeviation",  _startingHeadingDeviation);
    fail += !ros::param::get("graphslam/mapping/observationIncrement",      _observationIncrement);
    fail += !ros::param::get("graphslam/mapping/loopClosureFactor",         _loopClosureFactor);
    fail += !ros::param::get("graphslam/mapping/loopClosureRange",          _loopClosureRange);
    fail += !ros::param::get("graphslam/mapping/deleteLandmarkThreshold",   _deleteLandmarkThreshold);
    
    //localization
    fail += !ros::param::get("graphslam/localization/initialPosition/x",        initX);
    fail += !ros::param::get("graphslam/localization/initialPosition/y",        initY);
    fail += !ros::param::get("graphslam/localization/initialPosition/theta",    initTheta);

    //noise
    fail += !ros::param::get("graphslam/noise/obs/d",       observationNoiseDistance);
    fail += !ros::param::get("graphslam/noise/obs/theta",   observationNoiseTheta);
    fail += !ros::param::get("graphslam/noise/odom/x",      controlNoiseX);
    fail += !ros::param::get("graphslam/noise/odom/y",      controlNoiseY);
    fail += !ros::param::get("graphslam/noise/odom/theta",  controlNoiseTheta);
    
    //general
    fail += !ros::param::get("graphslam/mapName", _mission);
    fail += !ros::param::get("graphslam/daMethod", daMethod);

    if (fail){
        ROS_WARN("A failure occured during parameters loading");
        exit(EXIT_FAILURE);
    }

    _pose = Pose(initX, initY, initTheta);

    _startingHeadingDeviation = (_startingHeadingDeviation / 180.0f * M_PI);
    _observationNoise << powf(observationNoiseDistance, 2), 0,
                        0, powf(observationNoiseTheta * M_PI / 180, 2);
    _controlNoise << powf(controlNoiseX, 2), 0, 0,
                     0, powf(controlNoiseY, 2), 0, 
                     0, 0, powf(controlNoiseTheta * M_PI / 180, 2);
    
    if (daMethod == "ML")
        _daMethod = DaMethod::ML;
    
    if (daMethod == "JCBB")
        _daMethod = DaMethod::JCBB;

    if (daMethod == "Tracking")
        _daMethod = DaMethod::Tracking;
 
}

/**
 * 
 * 
 *      CALLBACKS
 * 
 */


// vx, vy and r callback, from state estimator
void GraphSlam::updateVelocity(const common_msgs::CarVelocity &vel) {
    
    // timer between current and last pose callback, used to integrate velocities in the motion model
    double delta = (vel.header.stamp - _lastUpdateTime).toSec();

    // special check for first update, avoids huge jump at the beginning
    if (_firstLandmarkUpdate){
        delta = 0.0;
        _firstLandmarkUpdate = false;
    }
    
    // create control from vx, vy and r, run motion model and save current timestamp for the next iteration
    Control control(vel.velocity.x, vel.velocity.y, vel.velocity.theta);
    _pose = MotionModel::updatePose(_pose, control, delta);
    _lastUpdateTime = vel.header.stamp;
}

// cones callback, from lidar
void GraphSlam::updateConeDetections(const common_msgs::ConeDetections &coneDetections) {

    _observations.clear();
    _coneDetections = coneDetections;

    transformConeDetections();
    
    //Transform cone detections into observations
    std::transform(_coneDetections.cone_detections.begin(), _coneDetections.cone_detections.end(), std::back_inserter(_observations),
                    [&](common_msgs::Cone cone)-> Observation {
                        return cone2Observation(cone, _coneDetections.header.stamp);
                    });

    // sort observations by distance
    std::sort(_observations.begin(), _observations.end(),
        [](Observation &a, Observation &b) -> bool { return a.measure.d < b.measure.d; });


    //Give the correct index to the new observations
    for (int i = 0; i < _observations.size(); i++) {
        _observations[i].index = i;
    }
 
    //Run ICP
    //icp_c.run(_currentConeDetections);

    // main update function
    run();
}

// main update function
void GraphSlam::run() {

    // add current _pose to the graph. _pose is updated at 200Hz by the velocity callback function
    graphAddPose();

    // set the new observations to the data association instance
    _dataAssociation.setObs(_observations);

    // iterate over all landmarks
    for (auto& lm: _landmarks){

        // skip landmarks that are not in view, 180º FoV and in a range of X meters
        if(!getInViewLm(lm)) continue;

        // transform the landmark into an observation, used in data association
        Observation obs = landmark2Observation(lm, _pose);
        lm.localObs = obs.measure;

        // for this landmark, go over all associations to see which fits best
        _dataAssociation.run(lm);
    }

    // the data association changes certain fields in _observations, so we need to bring it over
    _observations = _dataAssociation.getObs();

    // if we're in discovery mode, the map is changing
    if (_slamMode == SlamPhase::Slam){

        // after the data association loop, each observation has an associated probability
        // we now need to handle that data. based on a threshold we either update an existing
        // landmark or we create a new one from the observation
        handleAssociations();

        // keeps track of landmarks that were supposed to be seen and did get seen, or were missed
        // detects loop closure based on returning landmarks 
        // this happens when these landmarks that are seen with the same angle they were originally seen with
        updateStatistics();

        ///////////////////////////////////////////////////////////////////////

        _graphTypes.optimizeGraph(2,1);
        ROS_WARN("Pose before optimization: (%f, %f, %f)", _pose.pos.x(), _pose.pos.y(), _pose.theta);
        _pose = _graphTypes.getOptimizedPose(_odometryIndex.back());
        ROS_WARN("Pose after optimization: (%f, %f, %f)\n\n", _pose.pos.x(), _pose.pos.y(), _pose.theta);
        _graphTypes.fixGraph();

        ///////////////////////////////////////////////////////////////////////

        // once loop closure is detected by the updateStatistics(), we need to switch to localization mode
        if (_loopClosure)
            switchToLocal();
    }   
    
    // if we're in localization mode, map is static, so we just localize within it
    else if (_slamMode == SlamPhase::Local)
        localize();
    
    // computes the TF between map 0, 0 and car position and publishes both things (TF and car pose)
    computeOdometry();
    
    // top level visualization call, calls all other visualization functions
    visualization();
}

// adds landmark edges to the graph and optimizes graph (only non fixed nodes)
void GraphSlam::localize(){

    // similarly to updateLandmark, add edges for associated landmarks
    for (Observation& obs: _observations) {
        if(obs.minDistance < _associationThreshold)
            _graphTypes.addLandmarkEdge(_odometryIndex.back(), obs.associatedLm.index, obs, _observationNoise);        
    }

    // optimize current pose node based on the associations
    _graphTypes.optimizeGraph(5,1);

    // update pose based on the optimization
    _pose = _graphTypes.getOptimizedPose(_odometryIndex.back());

    // fixes the newly optimized pose. each iteration of localize() only changes the current pose node
    _graphTypes.fixGraph();
    
}

// checks whether a landmark is currently in view (180º FoV and inside a range of X meters)
bool GraphSlam::getInViewLm(Landmark& lm) {
    Observation obs = landmark2Observation(lm, _pose);
    if ((obs.measure.d < _associationRange && obs.measure.theta > -M_PI_2 && obs.measure.theta < M_PI_2) 
            || lm.observed)
        return true;
    else
        return false;   
}


// top level visualization functions
void GraphSlam::visualization() {
    
    // publishes the whole map
    visualizeLandmarks();

    // publishes all observations and draws a line between each observation and it's associated landmark
    // RVIZ TENDS TO CRASH WHEN SEEING THIS, VERY RESOURCE INTENSE, TRY AT YOUR OWN RISK (just keep the boxes unticked in rviz)
    validateDataAssociation();

    //Visualize SLAM detections
    currentDetectionsVisualization();
}

// optimizes the graph, fixes it and changes phase to localization
void GraphSlam::switchToLocal() {
    ROS_ERROR("OPTIMIZING");
    _graphTypes.unfixGraph();
    _loopClosure = false;
    _graphTypes.optimizeGraph(10, 0);
    _slamMode = SlamPhase::Local;
    _graphTypes.fixGraph();

    _pose = _graphTypes.getOptimizedPose(_odometryIndex.back());

    // once the graph optimizes, we need to update all landmarks in _landmarks with their new positions
    saveOptimizedLandmarks();
}

// creates new landmark and adds new lardmark vertex to the graph
void GraphSlam::graphAddLm(Observation& obs) {

    Landmark lm = createNewLandmark(obs);

    // we dont add a new landmark if another landmark exists within 0.5m
    auto it = std::find_if(_landmarks.begin(), _landmarks.end(), [&](Landmark& a){
        return (lm.mean - a.mean).norm() < 0.5;
    });

    if (it != _landmarks.end()) return;

    _landmarks.push_back(lm);
    _graphTypes.addLandmarkVertex(_landmarks.back(), _graphIndex);
    _graphTypes.addLandmarkEdge(_odometryIndex.back(), _graphIndex, obs, _observationNoise);
    _graphIndex++;
}

// adds pose vertex to graph
void GraphSlam::graphAddPose() {
    _graphTypes.addOdometryVertex(_pose, _graphIndex);
    _odometryIndex.push_back(_graphIndex);
    _graphTypes.addOdometryEdge(_odometryIndex.rbegin()[1], _odometryIndex.rbegin()[0], _controlNoise);
    _graphIndex++;   
}

// decides whether to update an existing landmark or create a new one, based on the association "factor"
void GraphSlam::handleAssociations() {
    for (Observation& obs: _observations) {
        if(obs.minDistance < _associationThreshold) 
            graphUpdateLm(obs);  
        else 
            graphAddLm(obs);
    }
}

// updates an existing landmark
void GraphSlam::graphUpdateLm(Observation& obs) {

    // first we need to find the landmark associated to the observation
    auto it = std::find(_landmarks.begin(), _landmarks.end(), obs.associatedLm); 
    if (it == _landmarks.end()) return;

    Landmark& lm = *it;
    
    // landmark position kalman update
    // essentially, the more we see a landmark, the more certain we are of it's actual position
    // initially the landmark has high uncertainty, but with each update the uncertainty lowers
    Eigen::Matrix2d kalmanGain = lm.covariance * obs.landmarkJacobian.transpose() * obs.innovationMatrix.inverse();
    //ROS_WARN("Kalman gain: %lf, %lf", kalmanGain(0, 0), kalmanGain(1, 1));
    lm.mean += kalmanGain * (Eigen::Vector2d(obs.measure.d, obs.measure.theta) - Eigen::Vector2d(obs.predicted.d, obs.predicted.theta));
    lm.covariance -= kalmanGain * obs.landmarkJacobian * lm.covariance;                       
    
    // later used by updateStatistics
    lm.observed = true;
    lm.numObserved += _observationIncrement;

    // add current observation to the list of observations for this landmark (visualization)
    lm.observations.push_back(observation2Landmark(obs, _pose));

    // add vertex and then edge connected to current pose
    _graphTypes.updateLandmarkVertex(lm);                 
    _graphTypes.addLandmarkEdge(_odometryIndex.back(), lm.index, obs, _observationNoise);   
}

// deletes a landmark from the graph
void GraphSlam::graphDeleteLm(Landmark& lm) {
    
    // first we delete the edge, then the vertex (g2o doesn't like loose edges, so delete them first)
    _graphTypes.deleteEdgesfromGraph(lm.index);  
    _graphTypes.deleteVertexfromGraph(lm.index);

    // delete landmark from _landmarks
    auto it = std::find(_landmarks.begin(), _landmarks.end(), lm);  
    _landmarks.erase(it); 
}

// initializes all the landmark struct fields
Landmark GraphSlam::createNewLandmark(Observation& obs) {
    Landmark lm;
    lm.mean = observation2Landmark(obs, _pose);
    lm.covariance = 1*Eigen::Matrix2d::Identity();
    lm.numObserved = _observationIncrement;
    lm.firstObservedHeading = _pose.theta;
    lm.color = as_cone_colors::UNKNOWN_CONE;
    lm.index = _graphIndex;
    lm.tracking = obs.index;
    lm.state = LmState::InView;
    lm.observed = true;
    lm.observations.push_back(observation2Landmark(obs, _pose));
    return lm;
}

// checks if a landmark that was supposed to be seen was indeed seen or missed
// based on the angle that a landmark was seen and is currently being seen, detects loop closure
void GraphSlam::updateStatistics() {
    size_t lmSeen = 0, lmMissed = 0;
    static size_t lmReturned = 0;

    // check whether a landmark was seen or not
    // for (auto& lm: _landmarks){
    for (int i = 0; i < _landmarks.size(); i++) {

        //ROS_WARN("LM ID: %d", _landmarks[i].index);

        // all the landmarks that are not in view are set to OffView
        if (!getInViewLm(_landmarks[i])){
            _landmarks[i].state = LmState::OffView;
            continue;
        };
                

        // if observed, increment count
        if (_landmarks[i].observed){
            _landmarks[i].numObserved += _observationIncrement;
            lmSeen++;
        }

        // if not observed, decrease count
        else {
            _landmarks[i].numObserved -= _observationIncrement;
            lmMissed++;
        }

        // landmark seen again after previously being leaving view
        if (_landmarks[i].observed && _landmarks[i].state == LmState::OffView){
            // if landmark is observed within 20m and with a similiar angle as 
            // the first time, it has "returned" 
            if (_landmarks[i].localObs.d < _loopClosureRange && std::abs(_pose.theta - _landmarks[i].firstObservedHeading) 
                                                    <= _startingHeadingDeviation){
                auto it = std::find(_landmarks.begin(), _landmarks.end(), _landmarks[i]);
                if (it != _landmarks.end())
                    it->state = LmState::Returned;
                lmReturned++;
            }
        }

        _landmarks[i].observed = false;
   
        // delete landmark
        // good landmarks aka cones are seen many times. noise is often seen once or twice
        // so we can delete these landmarks based on this counter
        Observation obs = landmark2Observation(_landmarks[i], _pose);

        // if (_landmarks[i].state == LmState::OffView && _landmarks[i].numObserved <= 300){
        //     graphDeleteLm(_landmarks[i]);
        // }
    }

    for (int i = 0; i < _landmarks.size(); i++) {
        if (_landmarks[i].state == LmState::OffView && _landmarks[i].numObserved <= _deleteLandmarkThreshold){
            graphDeleteLm(_landmarks[i]);
        }
    }
    
    // loop closure logic
    // essentially, if the number of returned landmarks are X% of the landmarks we should be
    // seeing right now, a loop closure is detected. loop closure factor usually 50%

    if (static_cast<double>(lmReturned) > 
                (lmSeen + lmMissed) * _loopClosureFactor){
        _loopClosure = true;
    }
}
float sign(float x) {
    if (x > 0.0) return 1.0;
    if (x < 0.0) return -1.0;
    return 0.0;   
}

float compute_yaw(float forward, float left) {

    if(forward != 0)
        return normalize_angle( atan2(forward,-left) + OFFSET );
    else if ( (forward == 0) && (-left != 0) )
        return normalize_angle( M_PI + sign(left)*OFFSET );

    else
        return 0;
}

float normalize_angle(float angle) {
  while(angle > M_PI)
        angle -= 2.0 * M_PI;
  while(angle < -M_PI)
        angle += 2.0 * M_PI;

    return angle;
}


// saves current pose as odometry data and calculates the TF between map and car
void GraphSlam::computeOdometry() {
    
    _odometry.header.frame_id = "map";
    _odometry.header.stamp = _coneDetections.header.stamp;

    _odometry.pose.pose.position.x = _pose.pos.x();
    _odometry.pose.pose.position.y = _pose.pos.y();
    _odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(_pose.theta);
    publishTf(_pose);
}

// TF specific transformations and stuff
void GraphSlam::publishTf(const Pose &pose) {
    // Position
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose.pos.x(), pose.pos.y(), 0.0));

    // Orientation
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, pose.theta);
    transform.setRotation(q);

    // Send TF
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_footprint"));
}

void GraphSlam::transformConeDetections() {
    _transform = tf_buffer_.lookupTransform("cog",  _coneDetections.header.frame_id , ros::Time(0));

    for (auto& cone: _coneDetections.cone_detections)
        cone.position.x += _transform.transform.translation.x;
} 

// updates landmarks in _landmarks with their optimized position, after loop closure
void GraphSlam::saveOptimizedLandmarks() {
    Vector2d newLm;
    for (Landmark &lm : _landmarks) {
        newLm = _graphTypes.getLandmarkLocationOptimized(lm.index);
        lm.mean = newLm;
    }
}
