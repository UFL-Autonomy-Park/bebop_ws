#include "BebopServer.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

BebopServer::BebopServer() : Node("bebop_server_node"), teleop_id_(-1), selector_state_(0.0) {
    RCLCPP_INFO(this->get_logger(), "Initializing Bebop Server Node");

    // Load Joy axis and button config
    this->declare_parameter("x_axis", -1);
    this->declare_parameter("y_axis", -1);
    this->declare_parameter("z_axis", -1);
    this->declare_parameter("yaw_axis", -1);

    this->declare_parameter("mode_button", -1);
    this->declare_parameter("takeoff_button", -1);
    this->declare_parameter("land_button", -1);

    this->declare_parameter("x_vel_max", 1.0);
    this->declare_parameter("y_vel_max", 1.0);
    this->declare_parameter("z_vel_max", 1.0);
    this->declare_parameter("yaw_vel_max", 1.0);

    this->get_parameter("x_axis", axes_.x.axis);
    this->get_parameter("y_axis", axes_.y.axis);
    this->get_parameter("z_axis", axes_.z.axis);
    this->get_parameter("yaw_axis", axes_.yaw.axis);

    this->get_parameter("mode_button", buttons_.mode.button);
    this->get_parameter("takeoff_button", buttons_.takeoff.button);
    this->get_parameter("land_button", buttons_.land.button);

    this->get_parameter("x_vel_max", axes_.x.factor);
    this->get_parameter("y_vel_max", axes_.y.factor);
    this->get_parameter("z_vel_max", axes_.z.factor);
    this->get_parameter("yaw_vel_max", axes_.yaw.factor);

    RCLCPP_INFO(this->get_logger(), "Loaded controller parameters:\nX: %d, Y: %d, Z: %d, Yaw: %d, Mode: %d, Takeoff: %d, Land: %d", 
        axes_.x.axis, axes_.y.axis, axes_.z.axis, axes_.yaw.axis, buttons_.mode.button, buttons_.takeoff.button, buttons_.land.button);

    // Teleop command publisher
    teleop_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("~/cmd_teleop", 10);

    // Scan for connected bebops at 1 hz
    scan_timer_ = this->create_wall_timer(1s, std::bind(&BebopServer::scan_bebops, this));

    // Publish teleop commands at 50Hz
    teleop_timer_ = this->create_wall_timer(20ms, std::bind(&BebopServer::publish_teleop, this));

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&BebopServer::joy_callback, this, _1));
}

void BebopServer::scan_bebops() {
    // Get list of nodes and their namespaces

    // Look for advertised tol service to see if a given client is active
    auto services_and_types = this->get_service_names_and_types();
    for (const auto& service : services_and_types) {
        std::string service_handle = service.first;
        
        if (service_handle.length() > 1 && service_handle[0] == '/') {

            // Only check services that are properly namespaced with bebopX
            size_t second_slash = service_handle.find('/', 1);
            if (second_slash != std::string::npos) {
                // Get the namespace
                std::string ns = service_handle.substr(1, second_slash-1);

                // Use regex to validate namespace formatting
                std::regex pattern("^bebop(\\d+)$");
                std::smatch matches;
                if (std::regex_match(ns, matches, pattern)) {

                    // Get the associated bebop number
                    int id = std::stoi(matches[1]);

                    // Return if this bebop is already connected
                    if (bebops_.count(id) > 0) {
                        continue;
                    }

                    // Check for tol service
                    std::string service_name = service_handle.substr(second_slash+1, service_handle.length()-1);
                    if (service_name == std::string("tol")) {

                        // Initialize and add bebop to map
                        Bebop bebop;
                        bebop.flying_state = landed;
                        bebop.control_mode = teleop;
                        bebops_[id] = bebop;

                        // Create and add flying state subscriber to map
                        std::string state_topic = std::string("/").append(ns).append("/state");
                        auto state_sub = this->create_subscription<std_msgs::msg::Int32>(
                            state_topic, 10,
                            [this, id](const std_msgs::msg::Int32::SharedPtr msg) {
                                if (bebops_[id].flying_state != (FlyingState)msg->data) {
                                    bebops_[id].flying_state = (FlyingState)msg->data;

                                    std::string state_str;
                                    switch (bebops_[id].flying_state) {
                                    case landed:
                                        state_str = "landed";
                                        break;
                                    case taking_off:
                                        state_str = "taking_off";
                                        break;
                                    case flying:
                                        state_str = "flying";
                                        break;
                                    case landing:
                                        state_str = "landing";
                                        break;
                                    default:
                                        state_str = "undefined";
                                    }

                                    RCLCPP_INFO(this->get_logger(), "Bebop %d state changed to %s", id, state_str.c_str());
                                }
                            }
                        );
                        state_subs_[id] = state_sub;

                        // Create and add control mode subscriber to map
                        std::string mode_topic = std::string("/").append(ns).append("/mode");
                        auto mode_sub = this->create_subscription<std_msgs::msg::Int32>(
                            mode_topic, 10,
                            [this, id](const std_msgs::msg::Int32::SharedPtr msg) {
                                if (bebops_[id].control_mode != (ControlMode)msg->data) {
                                    bebops_[id].control_mode = (ControlMode)msg->data;

                                    std::string mode_str;
                                    switch (bebops_[id].flying_state) {
                                    case teleop:
                                        mode_str = "teleop";
                                        break;
                                    case autonomous:
                                        mode_str = "autonomous";
                                        break;
                                    default:
                                        mode_str = "undefined";
                                    }

                                    RCLCPP_INFO(this->get_logger(), "Bebop %d mode changed to %s", id, mode_str.c_str());
                                }
                            }
                        );
                        mode_subs_[id] = mode_sub;

                        // Create and add TOL client
                        std::string tol_handle = std::string("/").append(ns).append("/tol");
                        auto tol_client = this->create_client<std_srvs::srv::SetBool>(tol_handle);
                        tol_clients_[id] = tol_client;

                        // Create and add mode client
                        std::string mode_handle = std::string("/").append(ns).append("/set_mode");
                        auto mode_client = this->create_client<std_srvs::srv::SetBool>(mode_handle);
                        mode_clients_[id] = mode_client;

                        RCLCPP_INFO(this->get_logger(), "Bebop %d connected", id);
                    }
                }
            }
        }
    }

    // auto nodes_names_with_namespaces = this->get_node_names_and_namespaces();
    // auto nodes = this->get_node_names_and_namespaces();
    // RCLCPP_INFO(this->get_logger(), "Active nodes:");
    // for (const auto& node : nodes) {
    //     RCLCPP_INFO(this->get_logger(), "  - %s", node.c_str());
    // }
}

double BebopServer::get_joy_axis(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const Axis &axis) {
    if (axis.axis < 0 || std::abs(axis.axis) > (int)joy_msg->axes.size()-1) {
        RCLCPP_ERROR(this->get_logger(), "Axis %d out of range, joy has %d axes", axis.axis, (int)joy_msg->axes.size());
        return -1;
    }

    double output = joy_msg->axes[std::abs(axis.axis)] * axis.factor + axis.offset;

    return output;
}

int BebopServer::get_joy_button(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const Button &button) {
    if (button.button < 0 || button.button > (int)joy_msg->buttons.size()-1) {
        RCLCPP_ERROR(this->get_logger(), "Button %d out of range, joy has %d buttons", button.button, (int)joy_msg->buttons.size());
        return -1;
    }

    return joy_msg->buttons[button.button];
}

void BebopServer::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {

    // Do nothing if no bebops are connected
    if (bebops_.size() == 0) {
        return;
    }

    int mode_button_state = get_joy_button(joy_msg, buttons_.mode);
    if (mode_button_state != button_state_.mode.state) {
        if (mode_button_state == 1) {
            RCLCPP_INFO(this->get_logger(), "Mode button pressed");
            
            // Toggle between teleop and autonomous
            auto mode_request = std::make_shared<std_srvs::srv::SetBool::Request>();
            mode_request->data = bebops_[teleop_id_].control_mode == teleop ? true : false;
            auto mode_result = mode_clients_[teleop_id_]->async_send_request(mode_request, std::bind(&BebopServer::mode_response_callback, this, _1));
        }

        button_state_.mode.state = mode_button_state;
    }

    // Check for button presses
    int takeoff_button_state = get_joy_button(joy_msg, buttons_.takeoff);
    if (takeoff_button_state != button_state_.takeoff.state) {
        if (takeoff_button_state == 1) {
            RCLCPP_INFO(this->get_logger(), "Takeoff button pressed");
            takeoff_all_bebops();
        }

        button_state_.takeoff.state = takeoff_button_state;
    }

    int land_button_state = get_joy_button(joy_msg, buttons_.land);
    if (land_button_state != button_state_.land.state) {
        if (land_button_state == 1) {
            RCLCPP_INFO(this->get_logger(), "Land button pressed");
            land_all_bebops();
        }

        button_state_.land.state = land_button_state;
    }

    // Select bebops via D-pad
    double selector_state = joy_msg->axes[7];
    if (selector_state != selector_state_) {

        if (teleop_id_ > -1) {
            bool id_changed = false;
            if (selector_state == 1.0) {
                auto it = bebops_.find(teleop_id_);
                it++;
                if (it != bebops_.end()) {
                    teleop_id_ = it->first;
                } else {
                    teleop_id_ = (bebops_.begin())->first;
                }

                id_changed = true;
            } else if (selector_state == -1.0) {
                auto it = bebops_.find(teleop_id_);
                if (it != bebops_.begin()) {
                    it--;
                    teleop_id_ = it->first;
                } else {
                    teleop_id_ = (std::prev(bebops_.end()))->first;
                }

                id_changed = true;
            }

            if (id_changed) {
                RCLCPP_INFO(this->get_logger(), "Bebop %d selected", teleop_id_);
            }

        } else {
            // Teleop ID not yet initialized, so just select the first bebop (lowest id)
            teleop_id_ = (bebops_.begin())->first;
            RCLCPP_INFO(this->get_logger(), "Bebop %d selected", teleop_id_);
        }

        selector_state_ = selector_state;
    }

    // Read stick values
    teleop_cmd_.linear.x = get_joy_axis(joy_msg, axes_.x);
    teleop_cmd_.linear.y = get_joy_axis(joy_msg, axes_.y);
    teleop_cmd_.linear.z = get_joy_axis(joy_msg, axes_.z);
    teleop_cmd_.angular.z = get_joy_axis(joy_msg, axes_.yaw);
}

void BebopServer::publish_teleop() {
    rclcpp::Time now = this->get_clock()->now();
    teleop_pub_->publish(teleop_cmd_);
}

void BebopServer::takeoff_all_bebops() {
    // Send true request for takeoff
    auto takeoff_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    takeoff_request->data = true;
    for (const auto& pair : bebops_) {
        auto tol_result = tol_clients_[pair.first]->async_send_request(takeoff_request, std::bind(&BebopServer::tol_response_callback, this, _1));
    }
}

void BebopServer::land_all_bebops() {
    // Send false request for landing
    auto landing_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    landing_request->data = false;
    for (const auto& pair : bebops_) {
        auto tol_result = tol_clients_[pair.first]->async_send_request(landing_request, std::bind(&BebopServer::tol_response_callback, this, _1));
    }
}

void BebopServer::tol_response_callback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
    auto response = future.get();

    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "TOL request failed! Response = %s", response->message.c_str());
    }
}

void BebopServer::mode_response_callback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
    auto response = future.get();

    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Set mode request failed! Response = %s", response->message.c_str());
    }
}

