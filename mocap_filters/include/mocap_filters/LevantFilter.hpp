#ifndef MOCAP_FILTERS_LEVANTFILTER_HPP
#define MOCAP_FILTERS_LEVANTFILTER_HPP

#include <Eigen/Dense>

namespace mocap_filters {

class LevantFilter {
public:
    LevantFilter(double C, double dt);

    void propagate_filter(const Eigen::Vector3d& x_meas);

    Eigen::Vector3d get_position_estimate() const;
    Eigen::Vector3d get_velocity_estimate() const;

private:
    double dt_;
    double alpha_;
    double lambda_;
    Eigen::Vector3d x_hat_;
    Eigen::Vector3d rho_;
    Eigen::Vector3d rho_dot_;
};

}

#endif