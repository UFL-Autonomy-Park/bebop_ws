#include "BebopClient.hpp"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BebopClient>());
    rclcpp::shutdown();
    
    return 0;
}
