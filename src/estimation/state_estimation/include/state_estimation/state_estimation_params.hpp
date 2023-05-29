#ifndef STATE_ESTIMATION_PARAMS_HPP
#define STATE_ESTIMATION_PARAMS_HPP

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <iostream>

struct Inertia {
        double m;   //Car Weight
        double g;   //Gravity force
        double Iz;  //Inertial force I_zz

        //DEBUG
        void print() {
            std::cout << "m: " << m << std::endl;
            std::cout << "g: " << g << std::endl;
            std::cout << "Iz: " << Iz << std::endl;
            std::cout << "\n";
        }
    };

struct Kinematics {
    double track;
    double a;
    double b;

    //DEBUG
    void print() {
        std::cout << "track: " << track << std::endl;
        std::cout << "a: " << a << std::endl;
        std::cout << "b: " << b << std::endl;
        std::cout << "\n";
    }
};

struct Engine {
    double gr;

    //DEBUG
    void print() {
        std::cout << "mask: " << gr << std::endl;
        std::cout << "\n";
    }
};

struct Tire {
    Eigen::Matrix<double,3,2> sigma;
    double J;
    Eigen::Matrix<double,2,2> mu_k;
    Eigen::Matrix<double,2,2> mu_s;
    double gamma;
    double vs;
    double kx;
    double ky;
    double kz;
    double r;
    Eigen::Matrix<double,2,1> p;

    //DEBUG
    void print() {
        std::cout << "sigma: " << sigma << std::endl;
        std::cout << "J: " << J << std::endl;
        std::cout << "mu_k: " << mu_k << std::endl;
        std::cout << "mu_s: " << mu_s << std::endl;
        std::cout << "gamma: " << gamma << std::endl;
        std::cout << "vs: " << vs << std::endl;
        std::cout << "kx: " << kx << std::endl;
        std::cout << "ky: " << ky << std::endl;
        std::cout << "kz: " << kz << std::endl;
        std::cout << "r: " << r << std::endl;
        std::cout << "p: " << p << std::endl;
        std::cout << "\n";
    }
};

struct Params {

    Inertia inercia;
    Kinematics kinematics;
    Engine engine;

    //Array of tires
    std::vector<Tire> tireArray;
    Eigen::Vector4d Fn;
};

static inline float clampAnglePi2Pi(float angle) {
    return static_cast<float>(angle - 2 * M_PI * floor((angle + M_PI) / (2 * M_PI)));
}

static void convertXMLRPCVector(Eigen::MatrixXd &m, XmlRpc::XmlRpcValue &cov, const int matrixSize)
{
     if (cov.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        
        m.resize(matrixSize, matrixSize);

        for (int i = 0; i < matrixSize; i++) {
            for (int j = 0; j < matrixSize; j++) {
                // These matrices can cause problems if all the types
                // aren't specified with decimal points. Handle that
                // using string streams.
                std::ostringstream ostr;
                ostr << cov[matrixSize * i + j];
                std::istringstream istr(ostr.str());
                istr >> m(i, j);
            }
        }
     }
}

#endif