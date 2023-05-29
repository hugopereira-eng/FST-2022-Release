#ifndef FASTSLAM_MOTION_MODEL_HPP
#define FASTSLAM_MOTION_MODEL_HPP

#include <random>
#include "fastslam/fastslam_util.hpp"
#include "fastslam/fastslam_types.hpp"

class MotionModel {
private:
    static std::default_random_engine rng;

    static float std_x;
    static float std_y;
    static float std_theta;

public:
    static void setStandardDeviations(float std_x, float std_y, float std_theta);
    static Pose updatePose(const Pose& old_pose, const Control &control, float delta_time);
};
#endif