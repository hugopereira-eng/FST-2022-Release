#ifndef STATE_ESTIMATION_EKF_HPP
#define STATE_ESTIMATION_EKF_HPP

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/unsupported/Eigen/src/MatrixFunctions/MatrixExponential.h>
#include "state_estimation/state_estimation_params.hpp"
#include "state_estimation/state_estimation_lugre.hpp"

struct StateMatrices {
    Eigen::Matrix<double,15,15> A;
    Eigen::Matrix<double,15,2>  B;
    Eigen::Matrix<double,7,15>  C;
};
class KalmanFilter {
    public:
        KalmanFilter();
        void loadParameters(Params params);
        void loadCovarianceMatrices(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::MatrixXd R);
        void update();

        //Getters
        Eigen::Matrix<double,15,1> const & getStateVector() const;
        Eigen::Matrix<double,15,15> const & getStateCovariance() const;

        //Setters
        void updateWheelSpeedsRL(const double w_rl);
        void updateWheelSpeedsRR(const double w_rr);
        void updateInertia(const double accx, const double accy, const double wz);
        void updateSteeringAngle(const double steering_fl, const double steering_fr);
        void updateMotorTorque(const double u_rl, const double u_rr);
        void updateGpsVelocity(const double vx, const double vy);

        //Timestamp used to estimate pos
        ros::Time _currentTimeStamp;

    private:
        //Methods
        void computeInitialGuess();
        void initializeContStateMatrices();
        StateMatrices discretizeModel(const Eigen::Matrix<double,15,15> &A_c, const Eigen::Matrix<double,15,2> &B_c);

        //Attributes
        Params _params;
        Lugre _lugre;
        LinModelMatrix _continousStateMatrices;
        Eigen::Matrix<double,15,1> _x;
        Eigen::Matrix<double,15,15> _P;
        Eigen::Matrix<double,15,15> _Q;
        Eigen::Matrix<double,7,7> _R;
        Eigen::Matrix<double,2,1> _u;
        Eigen::Matrix<double,7,1> _y;
        Eigen::Matrix<double,4,1> _st;

        Eigen::Matrix<double,7,15> _C;
        const double _ts = 0.01;

};

#endif