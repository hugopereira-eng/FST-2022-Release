#ifndef FASTSLAM_PARTICLE_HPP
#define FASTSLAM_PARTICLE_HPP

#include <eigen3/Eigen/Eigen>
#include <random>
#include "fastslam/fastslam_types.hpp"
#include "fastslam/fastslam_motion_model.hpp"

class Particle {
private:
    static std::default_random_engine rng;

    int observation_increment;
    bool delete_cones_when_left_fov;
    bool associate_same_color_only;
    Eigen::Matrix2f processNoise;
    int16_t travel_idx = 0;

    Landmark createLandmark(Observation ob);
    void updateStatistics(std::vector<bool> &observed_lm);

public:
    Particle() : Particle(false, 50, true) {};
    Particle(bool delete_cones, int obs_incr, bool same_color);

    float weight = 1;
    Pose temp_pose;
    Pose pose;
    bool loop_closure_detected = false;
    std::vector<Landmark> landmarks;

    void updatePose(const Control &control, float delta);
    void updateLandmarks(std::vector<Observation> observations, SLAM_PHASE slam_phase);
    void purgeLandmarks();
    void save(std::ofstream &fout);
    void load(std::ifstream &fin);
};
#endif