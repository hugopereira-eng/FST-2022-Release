#include <random>
#include <iostream>
#include <fastslam2_0/fastslam2_0_types.hpp>

class MotionModel {
public:
    // Constructor
    // why no consctructor -> ask Pedro Gcc
    //Methods
    static Pose updatePose(const Pose &oldPose, const Control &control, float deltaTime);

private:
    // Attributes
    static std::default_random_engine rng;

    static float std_x;
    static float std_y;
    static float std_theta;

    // Methods

};