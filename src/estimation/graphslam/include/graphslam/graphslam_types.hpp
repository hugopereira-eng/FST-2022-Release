#ifndef GRAPHSLAM_TYPES_HPP
#define GRAPHSLAM_TYPES_HPP

#include <common_msgs/Cone.h>   
#include <eigen3/Eigen/Eigen>
#include <utility>
#include <cmath>
#include <ros/ros.h>
#include "as_lib/common.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

typedef enum SlamPhase {
    Slam,
    Local
} SlamPhase;

typedef enum LmState {
    InView,
    OffView,
    Returned
} LmState;

typedef enum LmSide {
    Unknown,
    Left,
    Right
} LmSide;

enum DaMethod {
    ML,
    JCBB,
    SCNN,
    Tracking
};
struct Control {
    double vx;
    double vy; 
    double yawRate;

    Control(): vx(0.0), vy(0.0), yawRate(0.0) {}
    Control(const double vx, const double vy, const double yawRate): vx(vx), vy(vy), yawRate(yawRate) {};
};

struct z {
    double d;
    double theta;

    z(): d(0.0), theta(0.0) {}
    z(const double d, const double theta): d(d), theta(theta) {}
};

struct Pose {
   Eigen::Vector2d pos;
   double theta;
   
   Pose(): pos(0.0, 0.0), theta(0.0) {}
   Pose(const double x, const double y, const double theta): pos(x,y), theta(theta) {}
};

struct Landmark {
    Eigen::Vector2d mean;       // landmark position mean
    Eigen::Matrix2d covariance; // landmark position covariance

    z localObs;                 //Landmark seen from the car
    
    int color;
    int index;
    int tracking;
    bool observed = false;
    int numObserved = 0;
    int travelIndex = -1;
    LmSide side = LmSide::Unknown;
    LmState state = LmState::InView;
    std::vector<Eigen::Vector2d> observations;
    double firstObservedHeading = 0;

    bool operator==(Landmark lm) const {
        if (lm.index == index) {
            return true;
        } else {
            return false;
        }
    }

};

struct Observation {
    z measure;
    as_cone_colors color;
    ros::Time timestamp;
    int index;
    Landmark associatedLm;
    double prob = 0.0;
    Eigen::Matrix2d landmarkJacobian = Eigen::Matrix2d::Zero();
    Eigen::Matrix2d innovationMatrix = Eigen::Matrix2d::Zero();
    z predicted;

    double minDistance = std::numeric_limits<double>::max();

    Observation() {}
    Observation(double d, double theta, as_cone_colors c , ros::Time stamp):measure(d, theta), color(c), timestamp(stamp) {}
};

// struct DataAssociation {
//     double prob = 0;
//     int index = 0;
//     Observation observation;
// };

static inline double clampAnglePi2Pi(double angle) {
    return static_cast<double>(angle - 2 * M_PI * floor((angle + M_PI) / (2 * M_PI)));
}

inline common_msgs::Cone observation2Cone(Observation obs) {
    
    common_msgs::Cone cone;
    cone.position.x = cosf(obs.measure.theta) * obs.measure.d;
    cone.position.y = sinf(obs.measure.theta) * obs.measure.d;
    cone.position.z = 0;

    cone.color = static_cast<int>(obs.color);

    return cone;
}
inline Observation cone2Observation(common_msgs::Cone cone, ros::Time stamp) {
    
    double d = hypot(cone.position.x, cone.position.y); // + _coneRadius;     might add cone radius later
    double theta = atan2f(cone.position.y, cone.position.x);
    return Observation(d, theta, static_cast<as_cone_colors>(cone.color), stamp);
}

inline Eigen::Vector2d observation2Landmark(Observation& obs, Pose& pose) {
     return Eigen::Vector2d(pose.pos.x() + obs.measure.d * cosf(obs.measure.theta + pose.theta),
                            pose.pos.y() + obs.measure.d * sinf(obs.measure.theta + pose.theta));
}

inline Observation landmark2Observation(Landmark& lm, Pose& pose) {
    Eigen::Vector2d distance;
    distance.x() = lm.mean.x() - pose.pos.x();
    distance.y() = lm.mean.y() - pose.pos.y();
    Observation obs;
    obs.measure.d = distance.norm();
    obs.measure.theta = clampAnglePi2Pi(atan2f(distance.y(), distance.x()) - pose.theta);
    
    return obs;
}

static void convert2Landmark (std::vector<Eigen::Vector2d> cones, std::vector<Landmark> &landmarks, int color, LmSide side, Eigen::Matrix2d observationNoise, int numObserved) {
    std::transform(cones.begin(),cones.end(), std::back_inserter(landmarks),
            [&](Eigen::Vector2d cone)-> Landmark {
                Landmark lm;
                lm.mean = cone;
                lm.covariance = observationNoise;
                lm.numObserved = numObserved;
                lm.color = color;
                lm.side = side;
                return lm;
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

#endif
