#include "mocap_filters/NumericalDifferentiation.hpp"

namespace mocap_filters {

NumericalDifferentiation::NumericalDifferentiation(double dt) 
    : dt_(dt), initialized_(false) {
    x_prev_.setZero();
    v_est_.setZero();
}

void NumericalDifferentiation::propagate(const Eigen::Vector3d& x_meas) {
    if (!initialized_) {
        x_prev_ = x_meas;
        v_est_.setZero();
        initialized_ = true;
        return;
    }

    v_est_ = (x_meas - x_prev_) / dt_;
    x_prev_ = x_meas;
}

Eigen::Vector3d NumericalDifferentiation::get_velocity_estimate() const {
    return v_est_;
}

}