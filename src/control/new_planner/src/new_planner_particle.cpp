#include "new_planner/new_planner_particle.hpp"

std::default_random_engine Particle::rng;

Particle::Particle() {

}

geometry_msgs::Pose Particle::getPose () {
    return this->_pose;
}

float Particle::getWeight () {
    return this->_weight;
}

void Particle::sample(float stepSize, float std_dev, geometry_msgs::Pose pose) {
    // sample particle angle
    float angle = std::normal_distribution<float>(0, std_dev)(rng);
    
    // convert deg to rad and add current orientation
    angle = angle * M_PI/180 + pose.orientation.z;

    this->_pose.position.x = pose.position.x + stepSize * std::cos(angle);
    this->_pose.position.y = pose.position.y + stepSize * std::sin(angle);
    this->_pose.orientation.z = angle;
}

void Particle::weight(std::vector<common_msgs::Cone> cones) {

    common_msgs::Cone cog = common_msgs::Cone();

    this->_weight = 0;

    for (auto &cone: cones){
        this->_weight += 1/std::pow(std::hypot(cone.position.x - this->_pose.position.x, cone.position.y - this->_pose.position.y), 3);
        
        cog.position.x += cone.position.x; cog.position.y += cone.position.y;
    }

    cog.position.x /= cones.size(); cog.position.y /= cones.size();

    this->_weight += std::pow(std::hypot(cog.position.x - this->_pose.position.x, cog.position.y - this->_pose.position.y), 1);

}