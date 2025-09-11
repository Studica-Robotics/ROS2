#include "studica_control/gamepad_component.h"

namespace studica_control {

std::shared_ptr<rclcpp::Node> GamepadController::initialize(rclcpp::Node *control) {
    control->declare_parameter<std::string>("gamepad.name", "gamepad_controller");
    control->declare_parameter<std::string>("gamepad.cmd_vel_topic", "cmd_vel");
    
    std::string name = control->get_parameter("gamepad.name").as_string();
    std::string cmd_vel_topic = control->get_parameter("gamepad.cmd_vel_topic").as_string();
    
    auto gamepad = std::make_shared<GamepadController>(name, cmd_vel_topic);
    return gamepad;
}

GamepadController::GamepadController(const rclcpp::NodeOptions &options) 
    : Node("gamepad_controller", options) {}

GamepadController::GamepadController(const std::string &name, const std::string &cmd_vel_topic) 
    : rclcpp::Node(name), linear_x_(0.0), linear_y_(0.0), angular_z_(0.0), turbo_mode_(false) {
    
    // Declare parameters with defaults
    this->declare_parameter<double>("linear_scale", 0.7);
    this->declare_parameter<double>("angular_scale", 1.0);
    this->declare_parameter<double>("deadzone", 0.1);
    this->declare_parameter<double>("turbo_multiplier", 1.5);
    // PS4 Controller semantic mappings (game_controller_node)
    this->declare_parameter<std::string>("axis_linear_x", "left_stick_y");      // Left stick vertical
    this->declare_parameter<std::string>("axis_linear_y", "left_stick_x");      // Left stick horizontal  
    this->declare_parameter<std::string>("axis_angular_z", "right_stick_x");    // Right stick horizontal
    this->declare_parameter<std::string>("button_turbo", "right_bumper");       // R1 button
    
    // Get parameters
    linear_scale_ = this->get_parameter("linear_scale").as_double();
    angular_scale_ = this->get_parameter("angular_scale").as_double();
    deadzone_ = this->get_parameter("deadzone").as_double();
    turbo_multiplier_ = this->get_parameter("turbo_multiplier").as_double();
    axis_linear_x_ = this->get_parameter("axis_linear_x").as_string();
    axis_linear_y_ = this->get_parameter("axis_linear_y").as_string();
    axis_angular_z_ = this->get_parameter("axis_angular_z").as_string();
    button_turbo_ = this->get_parameter("button_turbo").as_string();
    
    // Create subscription to joy topic
    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&GamepadController::joy_callback, this, std::placeholders::_1));
    
    // Create publisher for cmd_vel
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    
    // Create timer for continuous publishing (10Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&GamepadController::publish_twist, this));
    
    RCLCPP_INFO(this->get_logger(), "Gamepad controller initialized. Publishing to: %s", cmd_vel_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "PS4 Controls: %s/%s = movement, %s = rotation, %s = turbo", 
                axis_linear_x_.c_str(), axis_linear_y_.c_str(), axis_angular_z_.c_str(), button_turbo_.c_str());
}

GamepadController::~GamepadController() {}

void GamepadController::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Helper function to get axis value by name from Joy message
    auto get_axis_value = [&](const std::string& axis_name) -> double {
        // For game_controller_node, we need to map semantic names to indices
        // This is a simplified mapping for PS4 controller
        if (axis_name == "left_stick_x") return msg->axes.size() > 0 ? msg->axes[0] : 0.0;
        if (axis_name == "left_stick_y") return msg->axes.size() > 1 ? msg->axes[1] : 0.0;
        if (axis_name == "right_stick_x") return msg->axes.size() > 2 ? msg->axes[2] : 0.0;
        if (axis_name == "right_stick_y") return msg->axes.size() > 3 ? msg->axes[3] : 0.0;
        return 0.0;
    };
    
    auto get_button_value = [&](const std::string& button_name) -> bool {
        // Simplified mapping for PS4 controller buttons
        if (button_name == "right_bumper") return msg->buttons.size() > 5 ? msg->buttons[5] : false;
        if (button_name == "left_bumper") return msg->buttons.size() > 4 ? msg->buttons[4] : false;
        return false;
    };
    
    // Get raw axis values using semantic names
    double raw_linear_x = get_axis_value(axis_linear_x_);
    double raw_linear_y = get_axis_value(axis_linear_y_);
    double raw_angular_z = get_axis_value(axis_angular_z_);
    
    // Apply deadzone
    auto apply_deadzone = [this](double value) {
        return (std::abs(value) < deadzone_) ? 0.0 : value;
    };
    
    linear_x_ = apply_deadzone(raw_linear_x);
    linear_y_ = apply_deadzone(raw_linear_y);
    angular_z_ = apply_deadzone(raw_angular_z);
    
    // Check turbo button using semantic name
    turbo_mode_ = get_button_value(button_turbo_);
    
    // Debug output (throttled)
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Joy input - Linear: [%.2f, %.2f], Angular: %.2f, Turbo: %s",
                          linear_x_, linear_y_, angular_z_, turbo_mode_ ? "ON" : "OFF");
}

void GamepadController::publish_twist() {
    geometry_msgs::msg::Twist twist_msg;
    
    // Calculate scales
    double current_linear_scale = linear_scale_;
    double current_angular_scale = angular_scale_;
    
    if (turbo_mode_) {
        current_linear_scale *= turbo_multiplier_;
        current_angular_scale *= turbo_multiplier_;
    }
    
    // Set twist values
    twist_msg.linear.x = linear_x_ * current_linear_scale;
    twist_msg.linear.y = linear_y_ * current_linear_scale;
    twist_msg.linear.z = 0.0;
    
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = angular_z_ * current_angular_scale;
    
    // Publish the message
    cmd_vel_publisher_->publish(twist_msg);
    
    // Debug output for non-zero commands
    if (std::abs(linear_x_) > 0.01 || std::abs(linear_y_) > 0.01 || std::abs(angular_z_) > 0.01) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                              "Publishing cmd_vel - Linear: [%.2f, %.2f, 0], Angular: [0, 0, %.2f]",
                              twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z);
    }
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::GamepadController)
