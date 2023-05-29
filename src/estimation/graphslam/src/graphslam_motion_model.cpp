#include <graphslam/graphslam_motion_model.hpp>

std::default_random_engine MotionModel::rng;
float MotionModel::std_x = 0.01;
float MotionModel::std_y = 0.01;
float MotionModel::std_theta = 0.01;

Pose MotionModel::updatePose(const Pose &oldPose, const Control &control, double deltaTime) {
    Pose newPose = oldPose;

    float dx = std::hypot(control.vx, control.vy) * deltaTime;
    //float dy = control.vy * deltaTime;

    newPose.pos.x() += cosf(oldPose.theta) * dx;// - sinf(oldPose.theta) * dy;
    newPose.pos.y() += sinf(oldPose.theta) * dx;// + cosf(oldPose.theta) * dy;
    newPose.theta += control.yawRate * deltaTime;

    // clamp angle between -pi and pi
    newPose.theta = clampAnglePi2Pi(newPose.theta);
    return newPose;
}