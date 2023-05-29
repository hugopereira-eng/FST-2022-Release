#ifndef GRAPHSLAM_DATA_ASSOCIATION_HPP
#define GRAPHSLAM_DATA_ASSOCIATION_HPP

#include "graphslam/graphslam_types.hpp"

class DataAssociation {

    public:
        //Constructor
        DataAssociation();
        DataAssociation(DaMethod method, Eigen::Matrix2d observationNoise);

        //Setters
        void setObs(std::vector<Observation>& obs);

        //Getters
        std::vector<Observation> const & getObs() const;

        void run(Landmark &lm);
    
    private:
        void maximumLikelihood();
        void scnn();
        void jcbb();
        void tracking();

        //Atributes
        DaMethod _daMethod;
        Landmark _lm;
        Pose _pose;
        std::vector<Observation> _observations;
        Eigen::Matrix2d _observationNoise;
};

#endif