#include "mocap_filters/LevantFilter.hpp"
#include <cmath>

namespace mocap_filters {

LevantFilter::LevantFilter(double C, double dt) : dt_(dt) {
    alpha_ = 1.1 * C;
    lambda_ = 1.5 * std::sqrt(C);
    x_hat_.setZero();
    rho_.setZero();
    rho_dot_.setZero();
}

void LevantFilter::propagate_filter(const Eigen::Vector3d& x_meas) {
    Eigen::Vector3d x_tilde = x_meas - x_hat_;

    for (int i = 0; i < 3; ++i) {
        double val = x_tilde(i);
        double sgn = (val > 0) - (val < 0);
        double abs_val = std::abs(val);

        // Note: Logic copied from original header
        rho_(i) += abs_val * sgn * dt_; 
        rho_dot_(i) = rho_(i) + lambda_ * std::sqrt(abs_val) * sgn;
        x_hat_(i) += rho_dot_(i) * dt_;
    }
}

Eigen::Vector3d LevantFilter::get_position_estimate() const {
    return x_hat_;
}

Eigen::Vector3d LevantFilter::get_velocity_estimate() const {
    return rho_dot_;
}

}