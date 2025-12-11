#pragma once
#include <Eigen/Dense>
#include <cmath>

class LevantFilter {
public:
    LevantFilter(double C, double dt) : dt_(dt) {
        // C must be > max acceleration
        alpha_ = 1.1 * C;
        lambda_ = 1.5 * std::sqrt(C);
        x_hat_.setZero();
        u1_.setZero();
    }

    void propagate(const Eigen::Vector3d& f_t) {
        Eigen::Vector3d e = f_t - x_hat_;
        
        for (int i = 0; i < 3; ++i) {
            double s = (e(i) > 0) - (e(i) < 0);
            double abs_e = std::abs(e(i));

            u1_(i) += alpha_ * s * dt_;
            v_out_(i) = u1_(i) + lambda_ * std::sqrt(abs_e) * s;
            x_hat_(i) += v_out_(i) * dt_;
        }
    }

    Eigen::Vector3d getPosition() const { return x_hat_; }
    Eigen::Vector3d getVelocity() const { return v_out_; }

private:
    double dt_, alpha_, lambda_;
    Eigen::Vector3d x_hat_;
    Eigen::Vector3d u1_;
    Eigen::Vector3d v_out_;
};