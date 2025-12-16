#ifndef MOCAP_FILTERS_NUMERICALDIFFERENTIATION_HPP
#define MOCAP_FILTERS_NUMERICALDIFFERENTIATION_HPP

#include <Eigen/Dense>

namespace mocap_filters {

class NumericalDifferentiation {
public:
    explicit NumericalDifferentiation(double dt);

    void propagate(const Eigen::Vector3d& x_meas);

    Eigen::Vector3d get_velocity_estimate() const;

private:
    double dt_;
    bool initialized_;
    Eigen::Vector3d x_prev_;
    Eigen::Vector3d v_est_;
};

}

#endif