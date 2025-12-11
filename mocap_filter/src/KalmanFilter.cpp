#include <mocap_filter/KalmanFilter.h>
#include <iostream>

using namespace std;
using namespace Eigen;

namespace mocap {

KalmanFilter::KalmanFilter():
    attitude(Quaterniond::Identity()),
    position(Vector3d::Zero()),
    angular_vel(Vector3d::Zero()),
    linear_vel(Vector3d::Zero()),
    state_cov(Matrix12d::Identity()),
    input_cov(Matrix12d::Identity()),
    measurement_cov(Matrix6d::Identity()),
    filter_status(INIT_POSE),
    last_time_stamp(0.0),
    msg_interval(0.1) {
    // Constructor
    proc_jacob = Matrix12d::Identity();
    proc_noise_jacob = Matrix12d::Identity();
    meas_jacob = Matrix6_12d::Zero();
    meas_jacob.leftCols<6>() = Matrix6d::Identity();
    meas_noise_jacob = Matrix6d::Identity();
    }

bool KalmanFilter::init(const Matrix12d& u_cov, const Matrix6d& m_cov, const int& freq) {
    input_cov = u_cov;
    measurement_cov = m_cov;
    msg_interval = 1.0 / static_cast<double>(freq);
    return true;
}

bool KalmanFilter::prepareInitialCondition(
    const double& curr_time_stamp,
    const Eigen::Quaterniond& m_attitude,
    const Eigen::Vector3d& m_position
) {
    switch (filter_status) {
        case INIT_POSE: {
            last_time_stamp = curr_time_stamp;
            attitude = m_attitude;
            position = m_position;
            filter_status = INIT_TWIST;
            state_cov = Matrix12d::Identity();
            return true;
        }
        case INIT_TWIST: {
            Quaterniond dq = m_attitude * attitude.inverse();
            AngleAxisd daa(dq);
            Vector3d dr = m_position - position;
            double dt = curr_time_stamp - last_time_stamp;
            dt = dt * 0.9 + msg_interval * 0.1; // Smooth dt

            last_time_stamp += dt;
            attitude = m_attitude;
            position = m_position;
            angular_vel = daa.axis() * daa.angle() / dt;
            linear_vel = dr / dt;
            state_cov = Matrix12d::Identity();
            filter_status = READY;
            return true;
        }
        case READY: return false;
        default: return false;
    }
}

bool KalmanFilter::isReady() {
    return filter_status == READY;
}

void KalmanFilter::reset() {
    filter_status = INIT_POSE;
}

void KalmanFilter::prediction(const double& curr_time_stamp) {
    double dt = curr_time_stamp - last_time_stamp;
    dt = dt * 0.9 + msg_interval * 0.1; // Smooth dt
    
    Vector3d dw = angular_vel * dt;
    Vector3d dr = linear_vel * dt;
    double dangle = dw.norm();

    // Prevent division by zero
    Quaterniond dq;
    if (dangle > 1e-6) {
        dq = Quaterniond(AngleAxisd(dangle, dw / dangle));
    } else {
        dq = Quaterniond::Identity();
    }

    last_time_stamp += dt;
    attitude = dq * attitude;
    position += dr;
    
    // Update Jacobians
    proc_jacob(0,  1) =  dw(2);
    proc_jacob(0,  2) = -dw(1);
    proc_jacob(1,  0) = -dw(2);
    proc_jacob(1,  2) =  dw(0);
    proc_jacob(2,  0) =  dw(1);
    proc_jacob(2,  1) = -dw(0);
    proc_jacob(0,  6) =  dt;
    proc_jacob(1,  7) =  dt;
    proc_jacob(2,  8) =  dt;
    proc_jacob(3,  9) =  dt;
    proc_jacob(4, 10) =  dt;
    proc_jacob(5, 11) =  dt;

    // Covariance prediction
    state_cov = proc_jacob * state_cov * proc_jacob.transpose() + input_cov;
}

void KalmanFilter::update(
    const Eigen::Quaterniond& m_attitude,
    const Eigen::Vector3d& m_position  
) {
    Quaterniond re_q = m_attitude * attitude.inverse();
    AngleAxisd re_aa(re_q);
    if (std::abs(re_aa.angle()) > std::abs(2*M_PI-re_aa.angle())) {
        re_aa.angle() = 2*M_PI - re_aa.angle();
        re_aa.axis() = -re_aa.axis();
    }
    Quaterniond re_qs(re_aa);
    Vector3d re_th(re_qs.x()*2.0, re_qs.y()*2.0, re_qs.z()*2.0);
    Vector3d re_r = m_position - position;
    Vector6d re;
    re.head<3>() = re_th;
    re.tail<3>() = re_r;

    // Compute the covariance of the residual
    Matrix6d S = measurement_cov + state_cov.topLeftCorner<6, 6>();

    // Compute the Kalman gain
    Matrix12_6d K = state_cov.leftCols<6>()*S.inverse();

    // Compute the correction of the state
    Vector12d dx = K * re;

    // Update the state based on the correction
    Vector3d dq_vec3 = dx.head<3>() / 2.0;
    Vector4d dq_vec4 = 1.0/sqrt(1.0+dq_vec3.squaredNorm())*
    Vector4d(dq_vec3(0), dq_vec3(1), dq_vec3(2), 1.0);
    Quaterniond dq(dq_vec4(3), dq_vec4(0), dq_vec4(1), dq_vec4(2));

    attitude = dq * attitude;
    position += dx.segment<3>(3);
    angular_vel += dx.segment<3>(6);
    linear_vel += dx.segment<3>(9);

    // Update the uncertainty of the state/error
    state_cov = (Matrix12d::Identity()-K*meas_jacob) * state_cov;
} 

}