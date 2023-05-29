#ifndef NEW_PLANNER_PARTICLE_HPP
#define NEW_PLANNER_PARTICLE_HPP

#include <random>
#include <cmath>
#include <geometry_msgs/Pose.h>
#include "common_msgs/Cone.h"


class Particle {

public:
    //constructor
    Particle();

    //methods
    void sample(float stepSize, float std_dev, geometry_msgs::Pose pose);
    void weight(std::vector<common_msgs::Cone> cones);

    geometry_msgs::Pose getPose();
    float getWeight();

private:
    static std::default_random_engine rng;

    geometry_msgs::Pose _pose;
    float _weight;

};

#endif