#ifndef STUDICA_CONTROL__GAMEPAD_COMPONENT_H_
#define STUDICA_CONTROL__GAMEPAD_COMPONENT_H_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace studica_control {

class GamepadController : public rclcpp::Node {
public:
    explicit GamepadController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    GamepadController(const std::string &name, const std::string &cmd_vel_topic);
    ~GamepadController();

    static std::shared_ptr<rclcpp::Node> initialize(rclcpp::Node *control);

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void publish_twist();

    // Subscriptions and Publishers
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    
    // Timer for continuous publishing
    rclcpp::TimerBase::SharedPtr timer_;

    // Control parameters
    double linear_scale_;
    double angular_scale_;
    double deadzone_;
    
    // Current movement state
    double linear_x_;
    double linear_y_;
    double angular_z_;
    
    // Button/axis mappings (configurable) - semantic names for game_controller_node
    std::string axis_linear_x_;
    std::string axis_linear_y_;
    std::string axis_angular_z_;
    std::string button_turbo_;
    
    bool turbo_mode_;
    double turbo_multiplier_;
};

} // namespace studica_control

#endif // STUDICA_CONTROL__GAMEPAD_COMPONENT_H_
