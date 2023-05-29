#include "state_estimation/state_estimation_lugre.hpp"

using namespace Eigen;

LinModelMatrix Lugre::fullCarDynamics(std::vector<Tire> &tireArray, const Matrix<double,4,1> &steerAngle, const Matrix<double,4,1> &wheelSpeedArray, const double vx, const double vy, const double wz, const Vector4d &FnArray, const double Iz, const double m) {

    Matrix<double,12,12> A11;
    Matrix<double,12,3>  A12;
    Matrix<double,3,12>  A21;
    Matrix<double,3,3>   A22;
    Matrix<double,15,15> A;
    Matrix<double,15,4>  B;
    Matrix<double,4,12> C11;
    Matrix<double,4,3>  C12;
    Matrix<double,2,12> C21;
    Matrix<double,2,3>  C22;
    Matrix<double,9,15>  C;

    //Front-Left
    const LinModelMatrixTire contStateMatricesFL = tireDynamics(tireArray[0], steerAngle(0), wheelSpeedArray(0), vx, vy, wz, FnArray(0), Iz, m);
    //Front-Right
    const LinModelMatrixTire contStateMatricesFR = tireDynamics(tireArray[1], steerAngle(1), wheelSpeedArray(1), vx, vy, wz, FnArray(1), Iz, m);
    //Rear-Left
    const LinModelMatrixTire contStateMatricesRL = tireDynamics(tireArray[2], steerAngle(2), wheelSpeedArray(2), vx, vy, wz, FnArray(2), Iz, m);
    //Rear-Right
    const LinModelMatrixTire contStateMatricesRR = tireDynamics(tireArray[3], steerAngle(3), wheelSpeedArray(3), vx, vy, wz, FnArray(3), Iz, m);

    //Construct matrix A
    A11 << contStateMatricesFL.A.block<3,3>(0,0), Matrix3d::Zero(), Matrix3d::Zero(), Matrix3d::Zero(),
           Matrix3d::Zero(), contStateMatricesFR.A.block<3,3>(0,0), Matrix3d::Zero(), Matrix3d::Zero(),
           Matrix3d::Zero(), Matrix3d::Zero(), contStateMatricesRL.A.block<3,3>(0,0), Matrix3d::Zero(),
           Matrix3d::Zero(), Matrix3d::Zero(), Matrix3d::Zero(), contStateMatricesRR.A.block<3,3>(0,0);
    A12 << contStateMatricesFL.A.block<3,3>(0,3),
           contStateMatricesFR.A.block<3,3>(0,3),
           contStateMatricesRL.A.block<3,3>(0,3),
           contStateMatricesRR.A.block<3,3>(0,3);
    A21 << contStateMatricesFL.A.block<3,3>(3,0), contStateMatricesFR.A.block<3,3>(3,0), contStateMatricesRL.A.block<3,3>(3,0), contStateMatricesRR.A.block<3,3>(3,0);
    A22 << contStateMatricesFL.A.block<3,3>(3,3) + contStateMatricesFR.A.block<3,3>(3,3) + contStateMatricesRL.A.block<3,3>(3,3) + contStateMatricesRR.A.block<3,3>(3,3);
    A << A11, A12, A21, A22;

    //Construct matrix B
    B << contStateMatricesFL.B.block<3,1>(0,0), Matrix<double,3,1>::Zero(), Matrix<double,3,1>::Zero(), Matrix<double,3,1>::Zero(),
         Matrix<double,3,1>::Zero(), contStateMatricesFR.B.block<3,1>(0,0), Matrix<double,3,1>::Zero(), Matrix<double,3,1>::Zero(),
         Matrix<double,3,1>::Zero(), Matrix<double,3,1>::Zero(), contStateMatricesRL.B.block<3,1>(0,0), Matrix<double,3,1>::Zero(),
         Matrix<double,3,1>::Zero(), Matrix<double,3,1>::Zero(), Matrix<double,3,1>::Zero(), contStateMatricesRR.B.block<3,1>(0,0),
         Matrix<double,4,4>::Zero();

    //Construct matrix C
    C11 << Matrix<double,1,3>(0,0,1), Matrix<double,1,3>::Zero(), Matrix<double,1,3>::Zero(), Matrix<double,1,3>::Zero(),
           Matrix<double,1,3>::Zero(), Matrix<double,1,3>(0,0,1), Matrix<double,1,3>::Zero(), Matrix<double,1,3>::Zero(),
           Matrix<double,1,3>::Zero(), Matrix<double,1,3>::Zero(), Matrix<double,1,3>(0,0,1), Matrix<double,1,3>::Zero(),
           Matrix<double,1,3>::Zero(), Matrix<double,1,3>::Zero(), Matrix<double,1,3>::Zero(), Matrix<double,1,3>(0,0,1);
    C12 = Matrix<double,4,3>::Zero();
    C21 << contStateMatricesFL.C.block<2,3>(1,0), contStateMatricesFR.C.block<2,3>(1,0), contStateMatricesRL.C.block<2,3>(1,0), contStateMatricesRR.C.block<2,3>(1,0);
    C22 << contStateMatricesFL.C.block<2,3>(1,3) + contStateMatricesFR.C.block<2,3>(1,3) + contStateMatricesRL.C.block<2,3>(1,3) + contStateMatricesRR.C.block<2,3>(1,3);
    C << C11, C12, C21, C22, Matrix<double,3,15>::Zero();
    C(6,12) = C(7,13) = C(8,14) = 1;

    return{A, B, C};
}

LinModelMatrixTire Lugre::tireDynamics(Tire& tire, const double delta, const double w, const double vx, const double vy, const double wz, double Fn, const double Iz, const double m) {

    double rl, L, vr_x, vr_y;
    double J = tire.J;
    Matrix<double,2,1> v_tire;
    Matrix<double,3,1> velArr(vx, vy, wz);

    const double rx = tire.p(0);
    const double ry = tire.p(1);

    //Damping Constants
    const double sigma_x = tire.sigma(1,0)+tire.sigma(2,0);   //Normally 0 since testing in the range of
    const double sigma_y = tire.sigma(1,1)+tire.sigma(2,1);   //is not duable due to high slip ratios needed

    Fn = fmax(Fn, 1.0);

    //Calculate Tire Parameters
    rl = tire.r - Fn/tire.kz;   //loaded radius metres
    L = 2.0*sqrtf(fmax(pow(tire.r,2)-pow(rl,2), _eps));

    //Calculate ground velocity under tyre
    v_tire = velocityTransform(delta, rx, ry, vx, vy, wz);

    //Relative Velocities Calculation
    vr_x = w*rl-v_tire(0);
    vr_y = -v_tire(1);
    Matrix<double,2,1> vr(vr_x, vr_y);

    //Rate of bristle restitution
    const double Ox = computeOx(tire, rl, w, vr, L);
    const double Oy = computeOy(tire, rl, w, vr, L);

    //Normalized stiffness
    const double Osig_x = tire.sigma(0,0) - tire.sigma(1,0)*Ox;
    const double Osig_y = tire.sigma(0,1) - tire.sigma(1,1)*Oy;
   
    const double c = cosf(delta);
    const double s = sinf(delta);

    Matrix<double,6,6> A;
    Matrix<double,3,3> A11,A12,A21,A22;
    Matrix<double,6,1> B;
    Matrix<double,4,6> C;

    //Dynamic of internal tire states
    //                zx   zy                           w       
    A11 <<           -Ox,   0,                         rl,
                       0, -Oy,                          0,  
       -(Fn*Osig_x*rl)/J,   0,  -(Fn*pow(rl,2)*sigma_x)/J;

    //Dynamic of external tire states
    //                     vx                   vy                                wz 
    A12 <<                 -c,                  -s,                      c*ry - rx*s,
                            s,                  -c,                    - c*rx - ry*s,
          (Fn*c*rl*sigma_x)/J, (Fn*rl*s*sigma_x)/J, -(Fn*rl*sigma_x*(c*ry - rx*s))/J;

    //Influence of the tyre states on the car body dynamics
    //                          zx                        zy                             w
    A21 <<         (Fn*Osig_x*c)/m,         -(Fn*Osig_y*s)/m,          (Fn*rl*sigma_x*c)/m,
                   (Fn*Osig_x*s)/m,          (Fn*Osig_y*c)/m,          (Fn*rl*sigma_x*s)/m,
          Fn*Osig_x/Iz*(rx*s-ry*c), Fn*Osig_y/Iz*(rx*c+ry*s), Fn*rl*sigma_x/Iz*(rx*s-ry*c);
    
    //Dynamics of the car states
    //                                                                     vx                                                                 vy                                                                    w   
    A22 <<                          -Fn/m*(pow(c,2)*sigma_x+pow(s,2)*sigma_y),                                        Fn/m*c*s*(sigma_y-sigma_x), Fn/m*sigma_x*(ry*pow(c,2)-rx*c*s)+Fn/m*sigma_y*(ry*pow(s,2)+rx*c*s),
                                                   Fn/m*c*s*(sigma_y-sigma_x),                         -Fn/m*(pow(c,2)*sigma_y+pow(s,2)*sigma_x),               Fn/m*sigma_x*s*(ry*c-rx*s)-Fn/m*sigma_y*c*(rx*c+ry*s),
            Fn/Iz*(sigma_x*(ry*pow(c,2)-rx*c*s)+sigma_y*(ry*pow(s,2)+rx*c*s)), Fn/Iz*(sigma_x*(ry*c*s-rx*pow(s,2))-sigma_y*(rx*pow(c,2)+ry*c*s)),      -Fn/Iz*(sigma_x*pow((ry*c-rx*s),2)+sigma_y*pow((ry*s+rx*c),2));

    A << A11, A12, A21, A22;

    //Input Matrix
    B << 0, 0, 1/J, 0, 0, 0;

    //Output matrix - y = [w, ax, ay, wz]
    //[y] = [C]*[x]
    C <<     0,      0,      1,      0,      0,      0,
        A(3,0), A(3,1), A(3,2), A(3,3), A(3,4), A(3,5),
        A(4,0), A(4,1), A(4,2), A(4,3), A(4,4), A(4,5),
             0,      0,      0,      0,      0,      1;
    
    return {A, B, C};
}

Matrix<double,2,1> Lugre::velocityTransform(const double delta, const double rx, const double ry, const double vx_car, const double vy_car, const double wz) {
    Matrix<double,2,1> v_car(vx_car, vy_car);
    Matrix<double,2,2> rot;
    Matrix<double,2,1> yaw_component(-ry*wz, rx*wz);

    rot << cosf(delta), -sinf(delta),
           sinf(delta), cosf(delta);

    return rot.transpose()*(v_car+yaw_component);
}

double Lugre::computeOx(const Tire &tire, const double re, const double w, const Matrix<double,2,1> &vr, const double L) {

    const double sig0_x = tire.sigma(0,0);

    //Calculate sliding friction
    const double g = computeG(tire,vr);

    //Steady state constant calculation
    const double kx = computeKi(w,vr,L,re,g,sig0_x);

    double Ox = vr.norm()*sig0_x/g + (kx/L)*abs(w*re);
    return Ox;
}

double Lugre::computeOy(const Tire &tire, const double re, const double w, const Matrix<double,2,1> &vr, const double L) {

    const double sig0_y = tire.sigma(0,1);

    //Calculate sliding friction
    const double g = computeG(tire,vr);

    //Steady state constant calculation
    const double ky = computeKi(w,vr,L,re,g,sig0_y);

    double Oy = vr.norm()*sig0_y/g + (ky/L)*abs(w*re);
    return Oy;
}

double Lugre::computeKi(const double w, const Matrix<double,2,1> &vr, const double L, const double re, const double g, const double sig0_i) {
    
    double zi = abs(w*re)*g/(vr.norm()*sig0_i);
    if(isnan(zi)) {
        zi = _eps;
    }

    const double aux = L/zi+sqrtf(_eps);
    double ki = (1-exp(-aux))/(1-(1-exp(-aux))/aux);
    return ki;
}

double Lugre::computeG(const Tire &tire, const Matrix<double,2,1> &vr) {
    Matrix<double,2,2> mk = tire.mu_k;
    Matrix<double,2,2> ms = tire.mu_s;
    const double gamma = tire.gamma;
    const double vs = tire.vs;
    double g;

    if(vr.norm() < 1e-10) {
        JacobiSVD<Matrix2d> svd(ms);
        g = svd.singularValues().maxCoeff();
    }
    else {
        Matrix<double,2,2> _aux1 = mk.array().pow(2);
        Matrix<double,2,1> _aux2 = _aux1*vr;
        Matrix<double,2,1> _aux3 = mk*vr;
        Matrix<double,2,2> _aux4 = ms.array().pow(2);
        Matrix<double,2,1> _aux5 = _aux4*vr;
        Matrix<double,2,1> _aux6 = ms*vr;

        g = _aux2.norm()/_aux3.norm()+(_aux5.norm()/_aux6.norm()-_aux2.norm()/_aux3.norm())*expf(-pow((vr.norm()/vs), gamma));
    }
    return g;
}