#ifndef MOCAP_FILTERS_LEVANTFILTER_HPP
#define MOCAP_FILTERS_LEVANTFILTER_HPP

#include <Eigen/Dense>

namespace mocap_filters {

class LevantFilter {
public:
    LevantFilter(double C, double dt);

    void propagate(const Eigen::Vector3d& x_meas);

    Eigen::Vector3d get_position_estimate() const;
    Eigen::Vector3d get_velocity_estimate() const;
    Eigen::Vector3d getPosition() const { return get_position_estimate(); }
    Eigen::Vector3d getVelocity() const { return get_velocity_estimate(); }

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