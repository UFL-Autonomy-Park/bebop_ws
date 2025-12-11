#include "mocap_filters/RhoFilter.hpp"
#include <unsupported/Eigen/MatrixFunctions>
#include <unsupported/Eigen/KroneckerProduct>
#include <cmath>

namespace mocap_filters {

RhoFilter::RhoFilter(
    double sampling_time,
    int state_space_dim,
    double alpha,
    double k1,
    double k2,
    double k3
) : 
    state_space_dim(state_space_dim),
    alpha(alpha)
{   
    Eigen::Matrix4d m;
    Eigen::Matrix4d m_inv;
    Eigen::Vector4d n;
    Eigen::Vector4d e;
    Eigen::RowVector4d s;
    Eigen::Matrix4d I_4 = Eigen::Matrix4d::Identity();
    Eigen::MatrixXd I_n = Eigen::MatrixXd::Identity(state_space_dim, state_space_dim);

    double m_12 = -(3 - std::pow(k1, 2) + (2*k1 + k2 + k3)*(k1 + k2));
    double m_13 = -(k1 + k2);
    double m_14 = -(1 + (k1 + k2)*(k3 - k1));
    double m_32 = -((2*k1 + k2 + k3)*(k1 + k2) + 1);
    double m_33 = -(2*k1 + k2);
    double m_42 = -(2*k1 + k2 + k3);
    double m_43 = -(k1 + k2)*(k3 - k1);

    m << 0,      1,      0,      0,
         m_12,   0,      m_32,   m_42,
         m_13,   0,      m_33,   -1,
         m_14,   0,      m_43,   -k3;

    m_inv = m.colPivHouseholderQr().inverse();

    double n_12 = 3 - std::pow(k1, 2) + (2*k1 + k2 + k3)*(k1 + k2);
    double n_13 = k1 + k2;
    double n_14 = 1 + (k1 + k2)*(k3 - k1);

    n << 0, n_12, n_13, n_14;
    e << 0, 1, 0, 0;
    s << -1, 0, -1, 0;

    Eigen::Matrix4d a_d = (m * sampling_time).exp();
    Eigen::Vector4d b_q = m_inv * (a_d - I_4) * n;
    Eigen::Vector4d b_s = m_inv * (a_d - I_4) * e;

    A_d = Eigen::kroneckerProduct(a_d, I_n);
    B_q = Eigen::kroneckerProduct(b_q, I_n);
    B_s = Eigen::kroneckerProduct(b_s, I_n);
    S = Eigen::kroneckerProduct(s, I_n);

    int zeta_dim = 4 * state_space_dim;
    zeta = Eigen::MatrixXd::Zero(zeta_dim, 1);
    next_zeta = Eigen::MatrixXd::Zero(zeta_dim, 1);
    sgn_term = Eigen::MatrixXd::Zero(state_space_dim, 1);
    next_position_estimate = zeta.block(0, 0, state_space_dim, 1);
    next_velocity_estimate = next_zeta.block(state_space_dim, 0, state_space_dim, 1);
}

void RhoFilter::propagate_filter(const Eigen::MatrixXd& current_state) 
{
    sgn_term.noalias() = S * zeta;
    sgn_term += current_state;

    next_zeta.noalias() = A_d * zeta;
    next_zeta.noalias() += B_q * current_state;
    next_zeta.noalias() += alpha * B_s * sgn_term.cwiseSign();
    
    zeta = next_zeta;
    next_position_estimate = zeta.block(state_space_dim, 0, state_space_dim, 1); 
    next_velocity_estimate = zeta.block(state_space_dim, 0, state_space_dim, 1);
}

const Eigen::MatrixXd& RhoFilter::get_velocity_estimate() const
{
    return next_velocity_estimate;
}

const Eigen::MatrixXd& RhoFilter::get_position_estimate() const
{
    return next_position_estimate;
}

} // namespace mocap_filters