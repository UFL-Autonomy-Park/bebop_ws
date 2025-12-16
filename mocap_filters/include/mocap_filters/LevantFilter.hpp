#ifndef MOCAP_FILTERS_LEVANTFILTER_HPP
#define MOCAP_FILTERS_LEVANTFILTER_HPP

#include <Eigen/Dense>
#include <cmath>

namespace mocap_filters {

class LevantFilter {
public:
    LevantFilter(double C, double dt) : dt_(dt) {
        alpha_ = 1.1 * C;
        lambda_ = 1.5 * std::sqrt(C);
        x_hat_.setZero();
        rho_.setZero();
        rho_dot_.setZero();
    }

    // Hard coded the number of states to 3
    void propagate(const Eigen::Vector3d& x_meas) {
        Eigen::Vector3d x_tilde = x_meas - x_hat_;
        for (int i = 0; i < 3; ++i) {
            double sgn_x_tilde = (x_tilde(i) > 0) - (x_tilde(i) < 0);
            double abs_x_tilde = std::abs(x_tilde(i));
            rho_(i) += abs_x_tilde * sgn_x_tilde * dt_;
            rho_dot_(i) = rho_(i) + lambda_ * std::sqrt(abs_x_tilde) * sgn_x_tilde;
            x_hat_(i) += rho_dot_(i) * dt_; // Euler solver because we have no choice
        }
    }

    Eigen::Vector3d get_position_estimate() const { return x_hat_; }
    Eigen::Vector3d get_velocity_estimate() const { return rho_dot_; }

private:
    double dt_, alpha_, lambda_;
    Eigen::Vector3d x_hat_, rho_, rho_dot_;
};

} // namespace mocap_filters

#endif