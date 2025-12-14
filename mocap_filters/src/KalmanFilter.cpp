#include "mocap_filters/KalmanFilter.hpp"

namespace mocap_filters {

KalmanFilter::KalmanFilter() :
    attitude(Eigen::Quaterniond::Identity()),
    position(Eigen::Vector3d::Zero()),
    angular_vel(Eigen::Vector3d::Zero()),
    linear_vel(Eigen::Vector3d::Zero()),
    state_cov(Matrix12d::Identity()),
    input_cov(Matrix12d::Identity()),
    measurement_cov(Matrix6d::Identity()),
    filter_status(INIT_POSE),
    last_time_stamp(0.0),
    msg_interval(0.1) 
{
    proc_jacob = Matrix12d::Identity();
    meas_jacob = Matrix6_12d::Zero();
    meas_jacob.leftCols<6>() = Matrix6d::Identity();
}

bool KalmanFilter::init(const Matrix12d& u_cov, const Matrix6d& m_cov, double freq) {
    input_cov = u_cov;
    measurement_cov = m_cov;
    msg_interval = 1.0 / freq;
    return true;
}

bool KalmanFilter::prepareInitialCondition(double curr_time, const Eigen::Quaterniond& q, const Eigen::Vector3d& p) {
    if (filter_status == INIT_POSE) {
        last_time_stamp = curr_time;
        attitude = q;
        position = p;
        filter_status = INIT_TWIST;
        return true;
    } else if (filter_status == INIT_TWIST) {
        double dt = curr_time - last_time_stamp;
        dt = dt * 0.9 + msg_interval * 0.1;
        
        Eigen::Quaterniond dq = q * attitude.inverse();
        Eigen::AngleAxisd daa(dq);
        
        last_time_stamp += dt;
        angular_vel = daa.axis() * daa.angle() / dt;
        linear_vel = (p - position) / dt;
        attitude = q;
        position = p;
        filter_status = READY;
        return true;
    }
    return false;
}

void KalmanFilter::prediction(double curr_time) {
    double dt = curr_time - last_time_stamp;
    dt = dt * 0.9 + msg_interval * 0.1;

    Eigen::Vector3d dw = angular_vel * dt;
    double dangle = dw.norm();
    Eigen::Quaterniond dq = (dangle > 1e-6) ? Eigen::Quaterniond(Eigen::AngleAxisd(dangle, dw / dangle)) : Eigen::Quaterniond::Identity();

    last_time_stamp += dt;
    attitude = dq * attitude;
    position += linear_vel * dt;

    // Jacobian updates
    proc_jacob(0, 1) = dw(2); proc_jacob(0, 2) = -dw(1);
    proc_jacob(1, 0) = -dw(2); proc_jacob(1, 2) = dw(0);
    proc_jacob(2, 0) = dw(1); proc_jacob(2, 1) = -dw(0);
    proc_jacob.topRightCorner<6,6>() = Eigen::Matrix<double, 6, 6>::Identity() * dt;

    state_cov = proc_jacob * state_cov * proc_jacob.transpose() + input_cov;
}

void KalmanFilter::update(const Eigen::Quaterniond& q, const Eigen::Vector3d& p) {
    Eigen::Quaterniond re_q = q * attitude.inverse();
    Eigen::AngleAxisd re_aa(re_q);
    if (std::abs(re_aa.angle()) > M_PI) {
        re_aa.angle() = 2 * M_PI - re_aa.angle();
        re_aa.axis() = -re_aa.axis();
    }
    
    Vector12d dx;
    Vector6d re;
    re << re_aa.axis() * re_aa.angle(), (p - position);

    Matrix6d S = measurement_cov + state_cov.topLeftCorner<6, 6>();
    Matrix12_6d K = state_cov.leftCols<6>() * S.inverse();
    dx = K * re;

    Eigen::Vector3d dq_v = dx.head<3>() / 2.0;
    Eigen::Quaterniond dq_corr(1.0, dq_v(0), dq_v(1), dq_v(2));
    dq_corr.normalize();

    attitude = dq_corr * attitude;
    position += dx.segment<3>(3);
    angular_vel += dx.segment<3>(6);
    linear_vel += dx.segment<3>(9);
    state_cov = (Matrix12d::Identity() - K * meas_jacob) * state_cov;
}

bool isReady() const { 
    return filter_status == READY;
}

} // namespace