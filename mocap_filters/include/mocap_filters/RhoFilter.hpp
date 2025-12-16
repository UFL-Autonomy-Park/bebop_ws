#ifndef MOCAP_FILTERS_RHOFILTER_HPP
#define MOCAP_FILTERS_RHOFILTER_HPP

#include <Eigen/Dense>

namespace mocap_filters {

class RhoFilter {
public:
    RhoFilter(
        double sampling_time,
        int state_space_dim,
        double alpha,
        double k1,
        double k2,
        double k3
    );

    RhoFilter(const RhoFilter&) = delete;
    RhoFilter& operator=(const RhoFilter&) = delete;

    void propagate_filter(const Eigen::VectorXd& current_position);
    
    const Eigen::MatrixXd& get_velocity_estimate() const;
    const Eigen::MatrixXd& get_position_estimate() const;

private:
    int state_space_dim;
    double alpha;

    Eigen::MatrixXd A_d;
    Eigen::MatrixXd B_q;
    Eigen::MatrixXd B_s;
    Eigen::MatrixXd S;

    Eigen::MatrixXd zeta;  
    Eigen::MatrixXd next_zeta;
    Eigen::MatrixXd next_velocity_estimate;
    Eigen::MatrixXd next_position_estimate;
    Eigen::MatrixXd sgn_term;
};

}

#endif