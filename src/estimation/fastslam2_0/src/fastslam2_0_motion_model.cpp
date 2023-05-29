#include <fastslam2_0/fastslam2_0_motion_model.hpp>

std::default_random_engine MotionModel::rng;
float MotionModel::std_x = 0.01;
float MotionModel::std_y = 0.01;
float MotionModel::std_theta = 0.01;

Pose MotionModel::updatePose(const Pose &oldPose, const Control &control, float deltaTime) {
    Pose newPose = oldPose;
    
    // newPose.s.x() = oldPose.s.x();
    // newPose.s.y() = oldPose.s.y();
    // newPose.s.z() = oldPose.s.z();
    
    float dx = std::hypot(control.vx, control.vy) * deltaTime;
    // float dy = control.vy * deltaTime;

    newPose.s.x() += cosf(oldPose.s.z()) * dx; // - sinf(oldPose.s.z()) * dy;
    newPose.s.y() += sinf(oldPose.s.z()) * dx; // + cosf(oldPose.s.z()) * dy;
    newPose.s.z() += control.theta * deltaTime;

    // clamp angle between -pi and pi
    newPose.s.z() = clampAnglePi2Pi(newPose.s.z());
    return newPose;
}