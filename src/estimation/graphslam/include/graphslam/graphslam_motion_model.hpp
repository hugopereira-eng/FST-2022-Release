#ifndef GRAPHSLAM_MOTION_MODEL_HPP
#define GRAPHSLAM_MOTION_MODEL_HPP

#include <random>
#include <iostream>
#include <graphslam/graphslam_types.hpp>

class MotionModel {
public:
    // Constructor
    // why no consctructor -> ask Pedro Gcc
    //Methods
    static Pose updatePose(const Pose &oldPose, const Control &control, double deltaTime);

private:
    // Attributes
    static std::default_random_engine rng;

    static float std_x;
    static float std_y;
    static float std_theta;

    // Methods

};
#endif