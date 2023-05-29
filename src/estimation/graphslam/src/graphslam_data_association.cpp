#include "graphslam/graphslam_data_association.hpp"

float _newLmThreshold = 0.01;

DataAssociation::DataAssociation(){
    DataAssociation(DaMethod::ML, Eigen::Matrix2d::Zero());
}

DataAssociation::DataAssociation(DaMethod method, Eigen::Matrix2d observationNoise): _daMethod(method), _observationNoise(observationNoise)
{}

//Setters
void DataAssociation::setObs(std::vector<Observation>& obs) {
    _observations = obs;
}

//Getters
std::vector<Observation> const & DataAssociation::getObs() const { return _observations; }

void DataAssociation::run(Landmark& lm) {

    _lm = lm;
    
    switch(_daMethod) {
        
    case DaMethod::ML:
        maximumLikelihood();
        break;
    
    case DaMethod::JCBB:
        jcbb();
        break;
        
    case DaMethod::SCNN:
        scnn();
        break;

    case DaMethod::Tracking:
        tracking();
        break;

    default:
        maximumLikelihood();
        break;
    }
}

void DataAssociation::maximumLikelihood() {
    //Obtain landmark distance and heading in relation to the car
    double dx = _lm.localObs.d * cosf(_lm.localObs.theta);
    double dy = _lm.localObs.d * sinf(_lm.localObs.theta);
    double d = _lm.localObs.d;

    Eigen::Vector2d z(d, _lm.localObs.theta);

    Eigen::Matrix2d G;
    G << dx*d, dy*d, 
        -dy  , dx;

    Eigen::Matrix2d Q = G * _lm.covariance * G.transpose() + _observationNoise;
    
    double minDistance = std::numeric_limits<double>::max();
    int obsIndex = 0;

    Eigen::Vector2d z_measure;

    //Iterate all observations
    for(Observation& obs: _observations){

        z_measure = Eigen::Vector2d(obs.measure.d, obs.measure.theta);
        // ROS_WARN("z_measure: %lf, %lf", z_measure.x(), z_measure.y());
        // ROS_WARN("z: %lf, %lf", z.x(), z.y());
        double mahalanobisDistance = (z_measure - z).transpose() * Q.inverse() * (z_measure - z);
        // ROS_WARN("mal dis: %lf", mahalanobisDistance);
        // double prob = (exp(-0.5f * mahalanobisDistance) / (2 * M_PI * sqrt(Q.determinant())));

        if (mahalanobisDistance < minDistance) {
            minDistance = mahalanobisDistance;
            obsIndex = obs.index;
        }
    }

    // ROS_WARN("min_dist: %lf", minDistance);
    // ROS_WARN("dist_diff d: %lf, t: %lf", _observations[obsIndex].measure.d-z.x(), _observations[obsIndex].measure.theta-z.y());

    if (minDistance < _observations[obsIndex].minDistance ){
        _observations[obsIndex].associatedLm = _lm;
        _observations[obsIndex].predicted.d = z.x();
        _observations[obsIndex].predicted.theta = z.y();
        _observations[obsIndex].innovationMatrix = Q;
        _observations[obsIndex].landmarkJacobian = G;
        _observations[obsIndex].minDistance = minDistance;
    }
}

void DataAssociation::scnn(){

}

void DataAssociation::jcbb(){

}

void DataAssociation::tracking(){
    
}