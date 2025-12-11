#ifndef MOCAP_FILTERS_KALMANFILTER_HPP
#define MOCAP_FILTERS_KALMANFILTER_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace mocap_filters {

class KalmanFilter {
public:
    using Matrix12d = Eigen::Matrix<double, 12, 12>;
    using Matrix6d = Eigen::Matrix<double, 6, 6>;
    using Matrix6_12d = Eigen::Matrix<double, 6, 12>;
    using Matrix12_6d = Eigen::Matrix<double, 12, 6>;
    using Vector12d = Eigen::Matrix<double, 12, 1>;
    using Vector6d = Eigen::Matrix<double, 6, 1>;

    enum Status { INIT_POSE, INIT_TWIST, READY };

    KalmanFilter();
    ~KalmanFilter() = default;

    bool init(const Matrix12d& u_cov, const Matrix6d& m_cov, double freq);
    bool prepareInitialCondition(double curr_time, const Eigen::Quaterniond& q, const Eigen::Vector3d& p);
    void prediction(double curr_time);
    void update(const Eigen::Quaterniond& q, const Eigen::Vector3d& p);
    bool isReady() const { return filter_status == READY; }

    Eigen::Quaterniond attitude;
    Eigen::Vector3d position;
    Eigen::Vector3d angular_vel;
    Eigen::Vector3d linear_vel;
    Matrix12d state_cov;

private:
    Matrix12d input_cov;
    Matrix6d measurement_cov;
    Status filter_status;
    double last_time_stamp;
    double msg_interval;

    Matrix12d proc_jacob;
    Matrix6_12d meas_jacob;
};

} // namespace mocap_filters

#endif