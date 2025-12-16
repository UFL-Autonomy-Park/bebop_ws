#include "NCRLabViz.hpp"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NCRLabViz>());
    rclcpp::shutdown();
    return 0;
}
