#ifndef RANDOM_H
#define RANDOM_H

#include <random>
#include <eigen3/Eigen/Core>

inline Eigen::VectorXd sampleStdNormal(Eigen::Index n, std::mt19937& gen){
    Eigen::VectorXd ret(n);
    std::normal_distribution<double> dist(0.0, 1.0);
    for(Eigen::Index i = 0; i < n; i++){
        ret(i) = dist(gen);
    }
    return ret;
}

#endif