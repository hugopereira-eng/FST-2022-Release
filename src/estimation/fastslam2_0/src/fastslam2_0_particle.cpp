#include <fastslam2_0/fastslam2_0_particle.hpp>

using namespace Eigen;


std::default_random_engine Particle::rng;

Particle::Particle(int observationIncrement, float loopClosureFactor, Matrix2f observationNoise, Matrix3f controlNoise, float newLandmarkThreshold, float startingHeadingDeviation): 
    _observationIncrement(observationIncrement), 
    _loopClosureFactor(loopClosureFactor),
    _observationNoise(observationNoise),
    _controlNoise(controlNoise),
    _newLandmarkThreshold(newLandmarkThreshold),
    _startingHeadingDeviation(startingHeadingDeviation)
{

    _startingHeadingDeviation = (_startingHeadingDeviation / 180.0f * M_PI);
}

void Particle::updatePose(Control &u, float deltaTime) {
    _pose = MotionModel::updatePose(_pose, u, deltaTime);
    // std::cout << "Motion Model Pose X: " << _pose.s.x() << " Y: " << _pose.s.y() << " Yaw: " << _pose.s.z() << std::endl;

}

void Particle::updateLandmarks(std::vector<Observation> observations, SLAM_PHASE slamPhase) { 
    // ROS_WARN_STREAM("BEGIN UPDATE");
    // std::cout << "Motion Model Pose X: " << _pose.s.x() << " Y: " << _pose.s.y() << " Yaw: " << _pose.s.z() << std::endl;
    std::vector<DataAssociation> dataAssociation;
    std::vector<bool> observedLandmarks(_landmarks.size(), false);
    Pose initialPose = _pose;
    for (Observation &observation: observations) {
        
        float prob = expf(-70);
        int index = -1;
        if(!_landmarks.empty()) { 
            //compute data association using Maximum likelihood
            std::tie(prob, index) = computeDataAssociation(observation, observedLandmarks, initialPose);
            // std::cout << "Prob-> " << prob << " Index: " << index << std::endl;
            if (prob < _newLandmarkThreshold) {
                // new landmark, lets remember it
                index = -1;
            }
        }

        DataAssociation datAss;
        datAss.observation = observation;
        datAss.prob = prob;
        datAss.index = index;
        dataAssociation.push_back(datAss);    
    }
    // sorting data association vector in order for new landmarks to be incorporated last
    std::sort(dataAssociation.begin(), dataAssociation.end(), [](DataAssociation &a, DataAssociation &b) -> bool {return a.index > b.index; });
    // std::cout << "DataAssociation SIZE: " << dataAssociation.size() << std::endl;
    // std::cout << "Landmark SIZE: " << _landmarks.size() << std::endl;

    
    Vector3f poseMean = initialPose.s;
    Matrix3f poseCovariance = _controlNoise;
    Vector2f ass_z = Vector2f(0, 0);
    Matrix2f ass_G; 
    ass_G.setZero();
    MatrixXf ass_Gs(2, 3); 
    ass_Gs.setZero();
    Matrix2f ass_Q; 
    ass_Q.setZero();

    // ROS_WARN_STREAM("BEGIN UPDATE POSE EKF");
    for (DataAssociation &da: dataAssociation) {
        if(da.index > -1) {
            // Using EKF to update the robot pose
            // std::cout << "Update existing Landmark" << std::endl;
            computeJacobians(da.index, ass_z, ass_G, ass_Gs, ass_Q);
            poseCovariance = (ass_Gs.transpose() * ass_Q.inverse() * ass_Gs + poseCovariance.inverse()).inverse();
            poseMean = _pose.s + poseCovariance * ass_Gs.transpose() * ass_Q.inverse() * (da.observation.z - ass_z);
            poseMean.z() = clampAnglePi2Pi(poseMean.z());
            // std::cout << "MEAN Pose -> X: " << poseMean.x() << " Y: " <<  poseMean.y() << " Yaw: " << poseMean.z() << std::endl;
            _pose.s = multivariateNormal(poseMean, poseCovariance);
            _pose.s.z() = clampAnglePi2Pi(_pose.s.z());
            // std::cout << "SAMPLED Pose -> X: " << _pose.s.x() << " Y: " <<  _pose.s.y() << " Yaw: " << _pose.s.z() << std::endl;
        
        }
        else if (da.index == -1 && slamPhase == SLAM_PHASE_MAP_BUILDING && !_loopClosureDetected){
            // Using the latest pose to create the landmark
            // std::cout << "Creating a New Landmark" << std::endl;
            _landmarks.push_back(createLandmark(da.observation));
            
        }
    }
    // ROS_WARN_STREAM("END UPDATE POSE EKF");

    // update the landmarks EKFs
    // ROS_WARN_STREAM("BEGIN UPDATE LANDMARK EKF");

    for (DataAssociation &da: dataAssociation) {
        if(da.index > -1) {
            // std::cout << "Update existing Landmark" << std::endl;
            computeJacobians(da.index, ass_z, ass_G, ass_Gs, ass_Q);
            Matrix2f L = ass_Gs * poseCovariance * ass_Gs.transpose() + ass_Q;
            // std::cout << "DA ERROR " << da.observation.z - ass_z << std::endl;
            _weight *= static_cast<float>(exp(-0.5f * (da.observation.z - ass_z).transpose() * L.inverse() * (da.observation.z - ass_z)) /
                                            (2 * M_PI * sqrt(L.determinant())));            
            if(slamPhase == SLAM_PHASE_MAP_BUILDING && !_loopClosureDetected) {
                observedLandmarks[da.index] = true;
                Landmark &landmark = _landmarks[da.index];
                Matrix2f K = landmark.covariance * ass_G.transpose() * ass_Q.inverse();
                landmark.mean += K*(da.observation.z - ass_z);
                landmark.covariance -= K * ass_G * landmark.covariance;
                landmark.color = da.observation.color;
                landmark.numObserved += _observationIncrement;  
            }                              
        }
        else if (da.index == -1 && slamPhase == SLAM_PHASE_MAP_BUILDING && !_loopClosureDetected) {
            _weight *= _newLandmarkThreshold;
        }
    
    // float prior = static_cast<float>(exp(-0.5f * (_pose.s - initialPose.s).transpose() * _controlNoise.inverse() * (_pose.s - initialPose.s)) /
    //                                        (2 * M_PI * sqrt(_controlNoise.determinant())));
    // float prop = static_cast<float>(exp(-0.5f * (_pose.s - poseMean).transpose() * poseCovariance.inverse() * (_pose.s - poseMean)) /
    //                                        (2 * M_PI * sqrt(poseCovariance.determinant())));
    // _weight = _weight * (prior/prop);
    std::cout << "WEIGHT -> " << _weight << std::endl;
    }
    if(slamPhase == SLAM_PHASE_MAP_BUILDING && !_loopClosureDetected)
        updateStatistics(observedLandmarks);
    
    // ROS_WARN_STREAM("END UPDATE LANDMARK EKF");
    // ROS_WARN_STREAM("END UPDATE");

}

std::tuple<float, int> Particle::computeDataAssociation(Observation observation, std::vector<bool> observedLandmarks, Pose pose) {
    float landmarkprob = 0;
    int landmarkIndex = -1;
    int index = 0;

    Vector2f ass_z = Vector2f(0, 0);
    Matrix2f ass_G; 
    ass_G.setZero();
    MatrixXf ass_Gs(2, 3); 
    ass_Gs.setZero();
    Matrix2f ass_Q; 
    ass_Q.setZero();
    // ROS_WARN_STREAM("BEGIN DATA ASSOCIATION");
    for (Landmark &landmark: _landmarks) {
        
        //Morgado Tracking
        if(observation.trackingIndex == landmark.tracking && !_loopClosureDetected) {
            //  std::cout << "MORGADO TA TRACKEANDO CONES" << std::endl;
            return std::make_tuple(20.0f, index);        
        } else { //Maximum Likelihood
            // Sample a new particle pose from the proposal distribution
            if (index >= observedLandmarks.size() || observedLandmarks[index]) {//|| (_sameColor && !landmark.couldBeColor(observation.color))) {
                index++;
                continue;
            }

            computeJacobians(index, ass_z, ass_G, ass_Gs, ass_Q); 
            
            if ((ass_z.x() > observation.z.x() * 2) || ass_z.x() < observation.z.x() / 2) {
                index++;
                continue;
            }

            if (ass_z.y() < -M_PI_2 || ass_z.y() > M_PI_2) {
                index++;
                continue;
            }
            if (landmark.state == LANDMARK_LEFT_VIEW && ass_z.x() <= 20 && std::abs(pose.s.z() - landmark.firstObserverdHeading) <= _startingHeadingDeviation) {
                landmark.state = LANDMARK_RETURNED;
            }
            // std::cout << "Landmark Jacobian -> " << ass_G << std::endl;
            // std::cout << "Pose Jacobian -> " << ass_Gs << std::endl;
            // std::cout << "Innovation Covariance Matrix -> " << ass_Q << std::endl;
            // std::cout << "Landmark Predicted at: Distance: " << ass_z.x() << " Bearing: " << ass_z.y() << std::endl;
            Matrix3f poseCovariance = (ass_Gs.transpose() * ass_Q.inverse() * ass_Gs + _controlNoise.inverse()).inverse();
            Vector3f poseMean = pose.s + poseCovariance * ass_Gs.transpose() * ass_Q.inverse() * (observation.z - ass_z);
            // std::cout << "PoseMean : " << poseMean << " Pose Covariance " << poseCovariance << std::endl;
            Pose newPose;
            newPose.s = multivariateNormal(poseMean, poseCovariance);
            newPose.s.z() = clampAnglePi2Pi(newPose.s.z());
            // std::cout << "DA Pose -> X: " << newPose.s.x() << " Y: " << newPose.s.y() << " Yaw: " << newPose.s.z() << std::endl;
            
            Observation newZ = landmark2Observation(&landmark, &newPose);
            // std::cout << "Landmark Predicted at after sample: Distance: " << newZ.x() << " Bearing: " << newZ.y() << std::endl;
            float prob = static_cast<float>(exp(-0.5f * (observation.z - newZ.z).transpose() * ass_Q.inverse() * (observation.z - newZ.z)) /
                                                (2 * M_PI * sqrt(ass_Q.determinant())));  
            // std::cout << "Prob -> " << prob << " Index: " << index << std::endl;
            if(prob > landmarkprob) {
                landmarkprob = prob;
                landmarkIndex = index;
            }
        } 
       index++; 
    }
    // ROS_WARN_STREAM("END DATA ASSOCIATION");
    return std::make_tuple(landmarkprob, landmarkIndex);
}

Landmark Particle::createLandmark(Observation observation) {
    
    Landmark landmark;

    landmark.mean = observation2Landmark(&observation, &_pose);
    // std::cout << "Landmark Location -> X: " << landmark.mean.x() << " Y: " << landmark.mean.y() << std::endl;
    landmark.covariance = _observationNoise;
    // std::cout << "Landmark Covariance -> " << landmark.covariance << std::endl;
    landmark.color = observation.color;
    // std::cout << "COLOR: " << landmark.color << std::endl;
    landmark.tracking = observation.trackingIndex;
    // std::cout << "LANDMARK CREATED WITH ID: " << landmark.tracking << std::endl;
    landmark.numObserved = _observationIncrement;
    landmark.firstObserverdHeading = _pose.s.z();

    return landmark;
}

void Particle::computeJacobians(int index, Vector2f &ass_z, Matrix2f &ass_G, MatrixXf &ass_Gs, Matrix2f &ass_Q) {
    // ROS_WARN_STREAM("BEGIN JACOBIANS");
    float dx = _landmarks[index].mean.x() - _pose.s.x();
    float dy = _landmarks[index].mean.y() - _pose.s.y();
    float d2 = pow(dx, 2) + pow(dy, 2);
    float d = sqrt(d2);
    float theta = clampAnglePi2Pi(atan2f(dy, dx) - _pose.s.z());
    
    Vector2f z(d, theta);
    ass_z = z;

    Matrix2f G;
    G << dx*d, dy*d, 
        -dy, dx;
    
    ass_G = G;

    MatrixXf Gs(2, 3);
    Gs << -dx*d, -dy*d, 0,
           dy, -dx, -d2;

    ass_Gs = Gs;

    ass_Q = ass_G * _landmarks[index].covariance * ass_G.transpose() + _observationNoise;
    
    // ROS_WARN_STREAM("END JACOBIANS");
}

void Particle::updateStatistics(std::vector<bool> observedLandmarks) {
    size_t seenLandmarks = 0;
    size_t missedLandmarks = 0;
    size_t returnedLandmarks = 0;

    float nearestLeftDistance = std::numeric_limits<float>::max();
    float nearestRightdistance = std::numeric_limits<float>::max();
    Landmark *nearestLandmarkLeft = nullptr;
    Landmark *nearestLandmarkRight = nullptr;

    for(size_t i = 0; i< _landmarks.size(); i++) {
        // check if landmark was observed
        if (i < observedLandmarks.size() && observedLandmarks[i]) {
            // count number of returned landmarks
            if(_landmarks[i].state == LANDMARK_RETURNED) 
                returnedLandmarks++;
            seenLandmarks++;
            continue;
        }
        // std::cout << "Number of Landmarks " << seenLandmarks << std::endl;
       
        Observation obs = landmark2Observation(&_landmarks[i], &_pose);
        if(obs.z.x() < 11 && obs.z.y() > -M_PI_2 && obs.z.y() < M_PI_2 ) {   // landmark is in view but not observed
            _landmarks[i].numObserved--; 

            if (_landmarks[i].numObserved < 0) {
                _landmarks.erase(_landmarks.begin() + i);
                i--;
                continue;
            }

            if (_landmarks[i].state == LANDMARK_RETURNED) // if this landmark has returned before increment number of returned landmarks
                returnedLandmarks++;

            // nontheless we missed it. increase number of missed landmarks
            missedLandmarks++;
        }
        else if (obs.z.x() > 10 && (obs.z.y() < -M_PI_2 || obs.z.y() > M_PI_2)) // outside of fov and behind the car. this cone left us.
            _landmarks[i].state = LANDMARK_LEFT_VIEW;

        // check if landmark is left or right of us and behind us and closer than closest landmark
        if (obs.z.y() >= M_PI_2 && obs.z.x() < nearestLeftDistance) {
            nearestLandmarkLeft = &_landmarks[i];
            nearestLeftDistance = obs.z.x();
        } else if (obs.z.y() <= -M_PI_2 && obs.z.x() < nearestRightdistance) {
            nearestLandmarkRight = &_landmarks[i];
            nearestRightdistance = obs.z.x();
        }
    }

    // since we passed those landmarks and they are closest to us we can assume we travelled in-between them
    if (nearestLandmarkLeft) {
        if (nearestLandmarkLeft->travelIndex == -1) {
            nearestLandmarkLeft->travelIndex++;
            nearestLandmarkLeft->side = LANDMARK_SIDE_LEFT;
        }
    }
    if (nearestLandmarkRight) {
        if (nearestLandmarkRight->travelIndex == -1) {
            nearestLandmarkRight->travelIndex++;
            nearestLandmarkRight->side = LANDMARK_SIDE_RIGHT;
        }
    }

    // // TODO: decide if we want to reset the loop closure indicator just to be more certain
    if (returnedLandmarks > 0 &&
        returnedLandmarks >= (seenLandmarks + missedLandmarks) * _loopClosureFactor) {
        _loopClosureDetected = true;
        std::cout << "LOOP CLOSURE DETECTED CARALHE" << std::endl;
    }
}