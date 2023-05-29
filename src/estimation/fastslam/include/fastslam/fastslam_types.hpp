#ifndef FASTSLAM_TYPES_HPP
#define FASTSLAM_TYPES_HPP

#include <eigen3/Eigen/Eigen>
#include <utility>
#include "as_lib/common.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

enum SLAM_PHASE {
    SLAM_PHASE_MAP_BUILDING,
    SLAM_PHASE_LOCALIZATION
};

struct Control {
    float vx = 0;
    float vy = 0;
    // float v = 0;
    float dtheta = 0;

    Control() {}
    Control(const Eigen::Vector3f &v) : vx(v.x()), vy(v.y()), dtheta(v.z()) {};
};

struct Pose {
    Eigen::Vector2f pos = Eigen::Vector2f(0, 0);
    float theta = 0;
};

struct Observation {
    Eigen::Vector2f z = Eigen::Vector2f(0, 0);
    int color;

    Observation() {}
    Observation(float x, float y, int c) : z(x, y), color(c) {}
};

enum LANDMARK_STATE {
    LANDMARK_IN_VIEW,
    LANDMARK_LEFT_VIEW,
    LANDMARK_RETURNED
};

enum LANDMARK_SIDE {
    LANDMARK_SIDE_UNKNOWN,
    LANDMARK_SIDE_LEFT,
    LANDMARK_SIDE_RIGHT
};

struct Landmark {
    Eigen::Vector2f mu;    // landmark position mean
    Eigen::Matrix2f sigma; // landmark position covariance

    int color;

    Eigen::Vector4i colors = Eigen::Vector4i(0, 0, 0, 0);

    int num_observed = 0;
    int16_t travel_idx = -1;
    uint8_t side = LANDMARK_SIDE_UNKNOWN;
    uint8_t state = LANDMARK_IN_VIEW;

    float first_observed_heading = 0;

    /**
     * 
     * Everytime a landmark is seen, this function is run to update color statistics.
     * The way our pipeline works shouldn't really need all this, since we'll never have the same cone classed as
     * differently colored, only colored or not colored. If it's color was seen at least once, it stops being saved
     * as a non colored cone.
     * 
     * TODO: add support for both orange cones, instead of just one as it is right now
     * 
     */ 

    void updateColor(int color) {
        if (color == ORANGE_CONE) {
            colors.y()++;
        }else if(color == YELLOW_CONE){
            colors.z()++;
        }else if(color == BLUE_CONE){
            colors.w()++;
        }
        
        std::vector<int> allColors = {UNKNOWN_CONE, ORANGE_CONE, YELLOW_CONE, BLUE_CONE};

        if (colors.y() == 0 && colors.z() == 0 && colors.w() == 0){
            this->color = allColors[0];
        }else{
            int max_idx = 1;
            for (int i = 2; i < allColors.size(); i++) {
                if (colors(i) > colors(max_idx))
                    max_idx = i;
            }
            this->color =allColors[max_idx];
        }
    }

    /**
     * 
     *  NOT USED IF @param associate_same_color_only IS SET TO FALSE
     * 
     */

    bool couldBeColor(int color) {
        // assume this cone can be any color if we have less than 20 observations
        // if (colors.sum() <  20)
        //     return true;
        if (this->color == UNKNOWN_CONE)
            return true; 
        if (color == ORANGE_CONE) {
            return colors.y() > 0;
        }else if (color == YELLOW_CONE){
            return colors.z() > 0;
        }else if (color == BLUE_CONE){
            return colors.w() > 0;
        }
        return false;
    }
};

static void convertXMLRPCVector(std::vector<Eigen::Vector2d> &cones, XmlRpc::XmlRpcValue &cone_array) {
    if (cone_array.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < cone_array.size(); i++) {         
            XmlRpc::XmlRpcValue cone = cone_array[i];
            Eigen::Vector2d coord;
            coord.x() = cone[0];
            coord.y() = cone[1];
            cones.push_back(coord);
        }
    }
}

static void convert2Landmark (std::vector<Eigen::Vector2d> cones, std::vector<Landmark> &landmarks, int color, int side, int numObserved, Eigen::Matrix2f observationNoise) {
    std::transform(cones.begin(),cones.end(), std::back_inserter(landmarks),
            [&](Eigen::Vector2d cone)-> Landmark {
                Landmark landmark;
                landmark.mu = cone.cast<float>();
                landmark.sigma = observationNoise;
                landmark.num_observed = numObserved;
                landmark.color = color;
                landmark.side = side;
                return landmark;
            });
}

static void convert2CenterPoint(std::vector<Eigen::Vector2d> centerPoints, nav_msgs::Path &centerLine) {
    std::transform(centerPoints.begin(),centerPoints.end(), std::back_inserter(centerLine.poses),
            [&](Eigen::Vector2d centerPoint)-> geometry_msgs::PoseStamped {
                geometry_msgs::PoseStamped point;
                point.pose.position.x = centerPoint[0];
                point.pose.position.y = centerPoint[1];
                point.pose.position.z = 0;
                point.pose.orientation = tf::createQuaternionMsgFromYaw(0);
                return point;
            });
}
#endif
