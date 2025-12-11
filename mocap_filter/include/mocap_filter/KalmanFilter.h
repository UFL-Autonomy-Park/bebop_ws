#ifndef MOCAP_KALMAN_FILTER_H
#define MOCAP_KALMAN_FILTER_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace mocap {

class KalmanFilter {
    public:

    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
    typedef Eigen::Matrix<double, 12, 12> Matrix12d;
    typedef Eigen::Matrix<double, 6, 12> Matrix6_12d;
    typedef Eigen::Matrix<double, 12, 6> Matrix12_6d;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<double, 12, 1> Vector12d;

    enum Status { 
        INIT_POSE, 
        INIT_TWIST, 
        READY 
    };

    KalmanFilter();
    ~KalmanFilter() {}

    // Initialize filter parameters
    bool init(const Matrix12d& u_cov, const Matrix6d& m_cov, const int& freq);

    // Initial conditions
    bool prepareInitialCondition(
        const double& curr_time_stamp,
        const Eigen::Quaterniond& m_attitude,
        const Eigen::Vector3d& m_position
    );

    // Core math
    void prediction(const double& curr_time_stamp);
    void update(
        const Eigen::Quaterniond& m_attitude,
        const Eigen::Vector3d& m_position
    );

    // Utilities
    bool isReady();
    void reset();

    // Public variables
    Eigen::Quaterniond attitude;
    Eigen::Vector3d position;
    Eigen::Vector3d angular_vel;
    Eigen::Vector3d linear_vel;
    Matrix12d state_cov;

private:
    // Filter parameters
    Matrix12d input_cov;
    Matrix6d measurement_cov;
    Status filter_status;
    double last_time_stamp;
    double msg_interval;

    // Jacobians
    Matrix12d proc_jacob; // Process Jacobian
    Matrix6_12d meas_jacob; // Measurement Jacobian
    Matrix12d proc_noise_jacob; // Process Noise Jacobian
    Matrix6d meas_noise_jacob; // Measurement Noise Jacobian
};

} // 

#endif