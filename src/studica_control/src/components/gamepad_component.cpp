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
    this->declare_parameter<int>("axis_linear_x", -1);
    this->declare_parameter<int>("axis_linear_y", -1);
    this->declare_parameter<int>("axis_angular_z", -1);
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

    // Subscribe to gamepad button mapping topic [/gamepad_buttons] with Transient Local QoS
    // to ensure we receive the last published mapping even if we subscribe late.
    rclcpp::QoS buttons_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    gamepad_button_subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/gamepad_buttons", buttons_qos,
        std::bind(&GamepadController::gamepad_button_callback, this, std::placeholders::_1));
    
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
    if (axis_linear_x_ < 0 || axis_linear_y_ < 0 || axis_angular_z_ < 0) {
        RCLCPP_WARN(this->get_logger(),
                    "Axis mappings are unset (=-1). Waiting for /gamepad_buttons to initialize [x,y,z].");
    }
    RCLCPP_INFO(this->get_logger(), "Listening for axis remap on /gamepad_buttons [Int32MultiArray: x,y,z]");
}

GamepadController::~GamepadController() {}

void GamepadController::gamepad_button_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    // Expecting an array of exactly 3 integers: [x_axis_index, y_axis_index, z_axis_index]
    if (msg->data.size() < 3) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Received /gamepad_buttons with %zu elements; expected 3 [x,y,z]",
                             msg->data.size());
        return;
    }

    int new_axis_x = msg->data[0];
    int new_axis_y = msg->data[1];
    int new_axis_z = msg->data[2];

    if (new_axis_x < 0 || new_axis_y < 0 || new_axis_z < 0) {
        RCLCPP_WARN(this->get_logger(),
                    "Ignoring /gamepad_buttons remap with negative index: [%d,%d,%d]",
                    new_axis_x, new_axis_y, new_axis_z);
        return;
    }

    // Update parameters so changes are visible externally (dynamic)
    this->set_parameter(rclcpp::Parameter("axis_linear_x", new_axis_x));
    this->set_parameter(rclcpp::Parameter("axis_linear_y", new_axis_y));
    this->set_parameter(rclcpp::Parameter("axis_angular_z", new_axis_z));

    // Update local cached values used by callbacks
    axis_linear_x_ = new_axis_x;
    axis_linear_y_ = new_axis_y;
    axis_angular_z_ = new_axis_z;

    RCLCPP_INFO(this->get_logger(),
                "Updated gamepad axis mapping via /gamepad_buttons -> X:%d Y:%d Z:%d",
                axis_linear_x_, axis_linear_y_, axis_angular_z_);
}

void GamepadController::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Ensure axis mapping has been provided
    if (axis_linear_x_ < 0 || axis_linear_y_ < 0 || axis_angular_z_ < 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Axis mappings not set. Awaiting /gamepad_buttons [x,y,z].");
        return;
    }

    // Check if we have enough axes and buttons
    const size_t axes_sz = msg->axes.size();
    if (static_cast<size_t>(axis_linear_x_) >= axes_sz ||
        static_cast<size_t>(axis_linear_y_) >= axes_sz ||
        static_cast<size_t>(axis_angular_z_) >= axes_sz ||
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
