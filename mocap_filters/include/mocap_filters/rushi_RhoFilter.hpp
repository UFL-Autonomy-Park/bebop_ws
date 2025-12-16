#ifndef MOCAP_FILTERS_RUSHI_RHOFILTER_HPP
#define MOCAP_FILTERS_RUSHI_RHOFILTER_HPP

#include <Eigen/Dense>

namespace mocap_filters {

class rushi_RhoFilter {
public:
    rushi_RhoFilter(
        double sampling_time,
        int state_space_dim,
        double alpha,
        double beta,
        double k
    );

    rushi_RhoFilter(const rushi_RhoFilter&) = delete;
    rushi_RhoFilter& operator=(const rushi_RhoFilter&) = delete;

    void propagate_filter(const Eigen::VectorXd& current_position);
    const Eigen::MatrixXd& get_velocity_estimate() const;
    const Eigen::MatrixXd& get_position_estimate() const;

private:
    int state_space_dim;
    double alpha;

    Eigen::MatrixXd A_d;
    Eigen::MatrixXd B_d;

    Eigen::MatrixXd zeta;
    Eigen::MatrixXd next_zeta;
    Eigen::MatrixXd next_velocity_estimate;
    Eigen::MatrixXd next_position_estimate;
};

#endif

}