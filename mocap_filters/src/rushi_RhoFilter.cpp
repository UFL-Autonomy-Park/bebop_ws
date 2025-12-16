#include "mocap_filters/rushi_RhoFilter.hpp"
#include <unsupported/Eigen/MatrixFunctions>
#include <unsupported/Eigen/KroneckerProduct>
#include <cmath>

namespace mocap_filters {

rushi_RhoFilter::rushi_RhoFilter(
    double sampling_time,
    int state_space_dim,
    double alpha,
    double beta,
    double k
) : 
    state_space_dim(state_space_dim)
{   
    Eigen::Matrix3d M;
    Eigen::Matrix3d M_inv;
    Eigen::Vector3d B;
    Eigen::Matrix3d I_3 = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd I_n = Eigen::MatrixXd::Identity(state_space_dim, state_space_dim);

    double M_12 = -std::pow(alpha,2);
    double M_32 = -(k + alpha + beta);
    double M_13 = k * alpha;
    double M_23 = alpha + k;
    double M_33 = -(beta + k);

    M << 0,   1,    0,
        M_12,  0,   M_32,
        M_13,  M_23,  M_33;
    
    double b_12 = std::pow(alpha,2) + (k + alpha + beta ) * (alpha + k);
    double b_13 = (beta + k) * (alpha + k) - k * alpha;

    B << 0, b_12, b_13;

    M_inv = M.colPivHouseholderQr().inverse();
    Eigen::Matrix3d a_d = (M * sampling_time).exp();
    Eigen::Vector3d b_d = M_inv * (a_d - I_3) * B;

    A_d = Eigen::kroneckerProduct(a_d, I_n);
    B_d = Eigen::kroneckerProduct(b_d, I_n);

    int zeta_dim = 3 * state_space_dim;
    zeta = Eigen::MatrixXd::Zero(zeta_dim, 1);
    next_zeta = Eigen::MatrixXd::Zero(zeta_dim, 1);
    next_position_estimate = zeta.block(0, 0, state_space_dim, 1);
    next_velocity_estimate = next_zeta.block(state_space_dim, 0, state_space_dim, 1);
}

void rushi_RhoFilter::propagate_filter(const Eigen::MatrixXd& current_position) 
{
    next_zeta.noalias() = A_d * zeta;
    next_zeta.noalias() += B_d * current_position;
    
    zeta = next_zeta;
    next_position_estimate = zeta.block(0, 0, state_space_dim, 1); 
    next_velocity_estimate = zeta.block(state_space_dim, 0, state_space_dim, 1);
}

const Eigen::MatrixXd& rushi_RhoFilter::get_velocity_estimate() const
{
    return next_velocity_estimate;
}

const Eigen::MatrixXd& rushi_RhoFilter::get_position_estimate() const
{
    return next_position_estimate;
}

} // namespace