#ifndef FASTSLAM_PARTICLE_HPP
#define FASTSLAM_PARTICLE_HPP

#include <eigen3/Eigen/Eigen>
#include <fstream>
#include <random>
#include "fastslam2_0/fastslam2_0_types.hpp"
#include "fastslam2_0/fastslam2_0_motion_model.hpp"

class Particle {
public: 
    //Contructor
    Particle() : Particle(10, 0.8,Eigen::Matrix2f::Identity(), Eigen::Matrix3f::Identity(), 0.02, 10.0) {};
    Particle(int observationIncrement, float loopClosureFactor, Eigen::Matrix2f observationNoise, Eigen::Matrix3f controlNoise, float newLandmarkThreshold, float startingHeadingDeviation);

    // Attributes
    Pose _pose;
    std::vector<Landmark> _landmarks;
    double _weight = 1;
    bool _loopClosureDetected = false;

    // Methods
    void updatePose(Control &u, float deltaTime);
    void updateLandmarks(std::vector<Observation> observations, SLAM_PHASE slamPhase);


private:
    // Attributes
    static std::default_random_engine rng;
    int _observationIncrement;
    float _loopClosureFactor;
    Eigen::Matrix2f _observationNoise;
    Eigen::Matrix3f _controlNoise;
    float _newLandmarkThreshold;
    float _startingHeadingDeviation;
    
    // Methods
    Landmark createLandmark(Observation observation);
    std::tuple<float, int> computeDataAssociation(Observation observation, std::vector<bool> observedLandmarks, Pose pose);
    void computeJacobians(int index, Eigen::Vector2f &ass_z, Eigen::Matrix2f &ass_G, Eigen::MatrixXf &ass_Gs, Eigen::Matrix2f &ass_Q);
    void updateStatistics(std::vector<bool> observedLandmarks);
    
};
#endif