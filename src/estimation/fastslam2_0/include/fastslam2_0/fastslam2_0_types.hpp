#ifndef FASTSLAM_TYPES_HPP
#define FASTSLAM_TYPES_HPP

#include <eigen3/Eigen/Eigen>
#include <common_msgs/Cone.h>   
#include <utility>
#include <cmath>
#include <ros/ros.h>
#include <as_lib/common.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>


enum SLAM_PHASE {
    SLAM_PHASE_MAP_BUILDING,
    SLAM_PHASE_LOCALIZATION
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

struct Control {
    float vx = 0;
    float vy = 0; 
    float theta = 0;

    Control() {}
    Control(const Eigen::Vector3f &v) : vx(v.x()), vy(v.y()), theta(v.z()) {};
};

struct Pose {
   Eigen::Vector3f s = Eigen::Vector3f(0, 0, 0);
};

struct Observation {
    Eigen::Vector2f z = Eigen::Vector2f(0, 0);
    int color;
    int trackingIndex;
    ros::Time timestamp;

    Observation() {}
    Observation(float x, float y, int c, int i, ros::Time stamp) : z(x, y), color(c), trackingIndex(i), timestamp(stamp) {}
};

struct Landmark {
    Eigen::Vector2f mean;    // landmark position mean
    Eigen::Matrix2f covariance; // landmark position covariance
    
    int color;
    Eigen::Vector4i colors = Eigen::Vector4i(0, 0, 0, 0);

    int tracking;
    int numObserved = 0;
    int16_t travelIndex = -1;
    uint8_t side = LANDMARK_SIDE_UNKNOWN;
    uint8_t state = LANDMARK_IN_VIEW;

    float firstObserverdHeading = 0;

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
            this->color = allColors[0];                                 //  unknown collor classification
        }else {
            int max_idx = 1;                                            // observed at least once
            for (int i = 2; i < allColors.size(); i++) {
                if (colors(i) > colors(max_idx))
                    max_idx = i;
            }
            this->color = allColors[max_idx];                           // finds the most probable color based on previous observatiosn
        }
    }

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

struct DataAssociation {
    float prob = 0;
    int index = 0;
    Observation observation;
};

static inline float clampAnglePi2Pi(float angle) {
    return static_cast<float>(angle - 2 * M_PI * floor((angle + M_PI) / (2 * M_PI)));
}

inline common_msgs::Cone observation2Cone(Observation observation, Landmark* landmark) {
    
    common_msgs::Cone cone;
    cone.position.x = cosf(observation.z.y()) * observation.z.x();
    cone.position.y = sinf(observation.z.y()) * observation.z.x();
    cone.position.z = 0;
    
    if (landmark->color == UNKNOWN_CONE) {
        if (landmark->side == LANDMARK_SIDE_LEFT) 
            cone.color = BLUE_CONE;
        else if (landmark->side == LANDMARK_SIDE_RIGHT)
            cone.color = YELLOW_CONE;
        else 
            cone.color = landmark->color;
    } else 
        cone.color = landmark->color;

    return cone;
}
inline Observation cone2Observation(common_msgs::Cone cone, ros::Time stamp) {
    
    float d = hypot(cone.position.x, cone.position.y); // + _coneRadius;     might add cone radius later
    float bearing = atan2f(cone.position.y, cone.position.x);
    return Observation(d, bearing, cone.color, cone.id, stamp);
}

inline Eigen::Vector2f observation2Landmark(Observation* observation, Pose* pose) {
     return Eigen::Vector2f(pose->s.x() + observation->z.x() * cosf(observation->z.y() + pose->s.z()),
                            pose->s.y() + observation->z.x() * sinf(observation->z.y() + pose->s.z()));
}

inline Observation landmark2Observation(Landmark* landmark, Pose* pose) {
    Eigen::Vector2f distance;
    distance.x() = landmark->mean.x() - pose->s.x();
    distance.y() = landmark->mean.y() - pose->s.y();
    Observation obs;
    obs.z.x() = distance.norm();
    obs.z.y() = clampAnglePi2Pi(atan2f(distance.y(), distance.x()) - pose->s.z());
    
    return obs;
}

inline uint32_t getColor(int &color) {

    uint8_t r = 0, g = 0, b = 0;  

    if (color == YELLOW_CONE){
        r = 255; g = 255;
    }else if (color == BLUE_CONE){
        b = 255;
    }else if (color == ORANGE_CONE){
        r = 255;
    }else if (color == UNKNOWN_CONE){
        r = 200; g = 200; b = 200;
    }

    return ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
}

static inline Eigen::Vector3f multivariateNormal(Eigen::Vector3f mean, Eigen::Matrix3f covariance) {
    int n = mean.rows();
    // std::cout <<"Mean: " << mean << " || " << "Covariance: " << covariance << std::endl;
    // Generate x from the N(0, I) distribution
    Eigen::Vector3f x(n);
    Eigen::Vector3f sum(n);
    sum.setZero();

    x.setRandom();
    // std::cout << "x1: " <<  x << std::endl;
    x = 0.5 * (x + Eigen::Vector3f::Ones(n));
    // std::cout << "x2: " <<  x << std::endl;
    sum += x;
    // std::cout << "SUM1: " << sum << std::endl;
    sum -= 0.5f * Eigen::Vector3f::Ones(n);
    // std::cout << "SUM2: " << sum << std::endl;
    x = sum / sqrtf(1.0/12.0);
    // std::cout << "x3: " <<  x << std::endl;
    // Find the eigen vectors of the covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance);
    Eigen::Matrix3f eigenvectors = eigen_solver.eigenvectors().real();

    // Find the eigenvalues of the covariance matrix
    Eigen::Matrix3f eigenvalues = eigen_solver.eigenvalues().real().asDiagonal();
    // std::cout << "Eigen Values: " << eigenvalues << std::endl;
    // Find the transformation matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(eigenvalues);
    Eigen::Matrix3f sqrt_eigenvalues = es.operatorSqrt();
    Eigen::Matrix3f Q = eigenvectors * sqrt_eigenvalues;
    // std::cout << "X: " << x << " Q: " << Q << std::endl;
    return static_cast<Eigen::Vector3f>(Q * x + mean);
}

static void convert2Landmark (std::vector<Eigen::Vector2d> cones, std::vector<Landmark> &landmarks, int color, int side, int numObserved, Eigen::Matrix2f observationNoise) {
    std::transform(cones.begin(),cones.end(), std::back_inserter(landmarks),
            [&](Eigen::Vector2d cone)-> Landmark {
                Landmark landmark;
                landmark.mean = cone.cast<float>();
                landmark.covariance = observationNoise;
                landmark.numObserved = numObserved;
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