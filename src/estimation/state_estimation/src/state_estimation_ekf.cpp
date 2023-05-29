#include "state_estimation/state_estimation_ekf.hpp"

KalmanFilter::KalmanFilter() {
    computeInitialGuess();
    initializeContStateMatrices();
}

void KalmanFilter::loadParameters(Params params) {
    _params = params;
}

void KalmanFilter::loadCovarianceMatrices(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::MatrixXd R) {
    _P = P;
    _Q = Q;
    _R = R;
}

void KalmanFilter::computeInitialGuess() {
    _x.setZero();
    _Q.setZero();
    _R.setZero();
    _u.setZero();
    _y.setZero();
    _st.setZero();
}

void KalmanFilter::initializeContStateMatrices() {
    _continousStateMatrices.A.setZero();
    _continousStateMatrices.B.setZero();
    _continousStateMatrices.C.setZero();
    _C.setZero();
}

//Setters
void KalmanFilter::updateWheelSpeedsRL(const double w_rl) {
    _y(0) = w_rl;
}

void KalmanFilter::updateWheelSpeedsRR(const double w_rr) {
    _y(1) = w_rr;
}

void KalmanFilter::updateInertia(const double accx, const double accy, const double wz) {
    _y(2) = accx;
    _y(3) = accy;
    _y(6) = -wz;
}

void KalmanFilter::updateSteeringAngle(const double steering_fl, const double steering_fr){
    _st(0) = steering_fl;
    _st(1) = steering_fr;
    _st(2) = 0;
    _st(3) = 0;
}

void KalmanFilter::updateMotorTorque(const double u_rl, const double u_rr){
    _u(0) = u_rr;
    _u(1) = u_rl;
}

void KalmanFilter::updateGpsVelocity(const double vx, const double vy) {
    _y(4) = vx;
    _y(5) = vy;
}

//Getters
Eigen::Matrix<double,15,1> const & KalmanFilter::getStateVector() const { return _x; }
Eigen::Matrix<double,15,15> const & KalmanFilter::getStateCovariance() const { return _P; }

void KalmanFilter::update() {

    Eigen::Matrix<double,15,15> A, A_d;
    Eigen::Matrix<double,15,2> B, B_d;

    if (_C.isZero()) {
        _continousStateMatrices = _lugre.fullCarDynamics(_params.tireArray, _st, Eigen::Matrix<double,4,1>(_x(2),_x(5),_x(8),_x(11)), _x(12), _x(13), _x(14), _params.Fn, _params.inercia.Iz, _params.inercia.m);
        _C = _continousStateMatrices.C.block<7,15>(2,0);
    }

    //Update the current state estimation
    Eigen::MatrixXd S = _C*_P*_C.transpose() + _R;
    Eigen::MatrixXd K = _P*_C.transpose()*S.inverse();

    Eigen::Matrix<double,15,1> xk = _x + K*(_y - _C*_x);  //current state given measurements y
    _continousStateMatrices = _lugre.fullCarDynamics(_params.tireArray, _st, Eigen::Matrix<double,4,1>(xk(2),xk(5),xk(8),xk(11)), xk(12), xk(13), xk(14), _params.Fn, _params.inercia.Iz, _params.inercia.m);

    A = _continousStateMatrices.A;
    B = _continousStateMatrices.B.block<15,2>(0,2);
    _C = _continousStateMatrices.C.block<7,15>(2,0);

    //Descretize the model
    StateMatrices discreteStateMatrices = discretizeModel(A, B);
    A_d = discreteStateMatrices.A;
    B_d = discreteStateMatrices.B;
    
    _P = (Eigen::Matrix<double,15,15>::Identity() - K*_C)*_P;

    //Predict the next state
    _x = A_d*xk + B_d*_u;
    _currentTimeStamp = ros::Time::now();
    _P = A_d*_P*A_d.transpose() + _Q;
}

StateMatrices KalmanFilter::discretizeModel(const Eigen::Matrix<double,15,15> &A_c, const Eigen::Matrix<double,15,2> &B_c) {
    constexpr int NA = 15;  //contStateMatrices.A.cols();
    constexpr int NB = 2;   //contStateMatrices.B.cols();
    constexpr int NC = 7;   //contStateMatrices.C.cols();
    
    Eigen::Matrix<double,NA,NA> A_d;
    Eigen::Matrix<double,NA,NB>  B_d;
    Eigen::Matrix<double,NC,NA>  C_d = Eigen::Matrix<double,NC,NA>::Zero();

    Eigen::Matrix<double,NA+NB,NA+NB> temp = Eigen::Matrix<double,NA+NB,NA+NB>::Zero();

    temp.block<NA,NA>(0,0) = A_c;
    temp.block<NA,NB>(0,NA) = B_c;
    temp = temp*_ts;

    const Eigen::Matrix<double,NA+NB,NA+NB> temp_res = temp.exp();

    A_d = temp_res.block<NA,NA>(0,0);
    B_d = temp_res.block<NA,NB>(0,NA);
    
    return {A_d, B_d, C_d};
}