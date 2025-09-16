#include "studica_control/gamepad_component.h"

namespace studica_control {

std::shared_ptr<rclcpp::Node> GamepadController::initialize(rclcpp::Node * /*control*/) {
    // The GamepadController is now a composable node and will be initialized by the component manager.
    // The constructor will handle parameter declaration and setup.
    return std::make_shared<GamepadController>(rclcpp::NodeOptions());
}

GamepadController::GamepadController(const rclcpp::NodeOptions &options) 
    : Node("gamepad_controller", options), linear_x_(0.0), linear_y_(0.0), angular_z_(0.0), turbo_mode_(false) {
    
    // Declare parameters with default values
    this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    this->declare_parameter<double>("linear_scale", 0.7);
    this->declare_parameter<double>("angular_scale", 1.0);
    this->declare_parameter<double>("deadzone", 0.1);
    this->declare_parameter<double>("turbo_multiplier", 1.5);
    this->declare_parameter<int>("axis_linear_x", 1);
    this->declare_parameter<int>("axis_linear_y", 0);
    this->declare_parameter<int>("axis_angular_z", 2);
    this->declare_parameter<int>("button_turbo", 5);
    
    // Get parameters
    linear_scale_ = this->get_parameter("linear_scale").as_double();
    angular_scale_ = this->get_parameter("angular_scale").as_double();
    deadzone_ = this->get_parameter("deadzone").as_double();
    turbo_multiplier_ = this->get_parameter("turbo_multiplier").as_double();
    axis_linear_x_ = this->get_parameter("axis_linear_x").as_int();
    axis_linear_y_ = this->get_parameter("axis_linear_y").as_int();
    axis_angular_z_ = this->get_parameter("axis_angular_z").as_int();
    button_turbo_ = this->get_parameter("button_turbo").as_int();
    
    // Create subscription to joy topic
    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&GamepadController::joy_callback, this, std::placeholders::_1));
    
    // Get topic name from parameters
    std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();

    // Create publisher for cmd_vel
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    
    // Create timer for continuous publishing (10Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&GamepadController::publish_twist, this));
    
    RCLCPP_INFO(this->get_logger(), "Gamepad controller initialized. Publishing to: %s", cmd_vel_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "PS4 Controls: axis[%d/%d] = movement, axis[%d] = rotation, button[%d] = turbo", 
                axis_linear_x_, axis_linear_y_, axis_angular_z_, button_turbo_);
}

GamepadController::~GamepadController() {}

void GamepadController::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Check if we have enough axes and buttons
    if (msg->axes.size() <= static_cast<size_t>(std::max({axis_linear_x_, axis_linear_y_, axis_angular_z_})) ||
        msg->buttons.size() <= static_cast<size_t>(button_turbo_)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "PS4 controller doesn't have enough axes/buttons");
        return;
    }
    
    // Get raw axis values using PS4 controller indices
    double raw_linear_x = msg->axes[axis_linear_x_];
    double raw_linear_y = msg->axes[axis_linear_y_];
    double raw_angular_z = msg->axes[axis_angular_z_];
    
    // Apply deadzone
    auto apply_deadzone = [this](double value) {
        return (std::abs(value) < deadzone_) ? 0.0 : value;
    };
    
    linear_x_ = apply_deadzone(raw_linear_x);
    linear_y_ = apply_deadzone(raw_linear_y);
    angular_z_ = apply_deadzone(raw_angular_z);
    
    // Check turbo button
    turbo_mode_ = (msg->buttons[button_turbo_] == 1);
    
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
