/*
 * gamepad_component.cpp
 *
 * ros2 component that reads joystick input and publishes velocity commands.
 * subscribes to the /joy topic (from a joy node), applies scaling and a
 * deadzone, and publishes geometry_msgs/Twist to a configurable cmd_vel topic.
 *
 * axis mapping defaults to -1 (unset). set them by publishing an
 * Int32MultiArray of [x_axis, y_axis, z_axis] to /gamepad_buttons, or
 * configure them via params.yaml in the gamepad_controller section.
 *
 * to use this component:
 *   1. enable gamepad in params.yaml under control_server
 *   2. run a joy node: ros2 run joy joy_node
 *   3. publish axis mapping to /gamepad_buttons if needed
 *
 * topic (subscribes): /joy (sensor_msgs/Joy) — raw joystick input
 * topic (subscribes): /gamepad_buttons (std_msgs/Int32MultiArray) — axis remap
 * topic (publishes):  <cmd_vel_topic> (geometry_msgs/Twist) — velocity output
 */

#include "studica_control/gamepad_component.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace studica_control {


// loads params.yaml into the node's options so the gamepad_controller
// section is picked up under the correct namespace
std::shared_ptr<rclcpp::Node> GamepadController::initialize(rclcpp::Node * /*control*/) {
    std::string params_file = ament_index_cpp::get_package_share_directory("studica_control")
                              + "/config/params.yaml";
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", params_file});
    return std::make_shared<GamepadController>(options);
}


// main constructor — declares parameters, sets up subscriptions, publisher,
// and a 10hz timer that continuously publishes the current velocity state
GamepadController::GamepadController(const rclcpp::NodeOptions &options)
    : Node("gamepad_controller", options),
      linear_x_(0.0), linear_y_(0.0), angular_z_(0.0), turbo_mode_(false) {

    // declare all parameters with safe defaults
    this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    this->declare_parameter<double>("linear_scale", -1);
    this->declare_parameter<double>("angular_scale", -1);
    this->declare_parameter<double>("deadzone", -1);
    this->declare_parameter<double>("turbo_multiplier", -1);
    this->declare_parameter<int>("axis_linear_x", -1);
    this->declare_parameter<int>("axis_linear_y", -1);
    this->declare_parameter<int>("axis_angular_z", -1);
    this->declare_parameter<int>("button_turbo", -1);

    linear_scale_    = this->get_parameter("linear_scale").as_double();
    angular_scale_   = this->get_parameter("angular_scale").as_double();
    deadzone_        = this->get_parameter("deadzone").as_double();
    turbo_multiplier_ = this->get_parameter("turbo_multiplier").as_double();
    axis_linear_x_   = this->get_parameter("axis_linear_x").as_int();
    axis_linear_y_   = this->get_parameter("axis_linear_y").as_int();
    axis_angular_z_  = this->get_parameter("axis_angular_z").as_int();
    button_turbo_    = this->get_parameter("button_turbo").as_int();

    // subscribe to raw joystick input
    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        std::bind(&GamepadController::joy_callback, this, std::placeholders::_1));

    // subscribe to axis remapping topic with transient local qos so we get
    // the last published mapping even if we start after it was sent
    rclcpp::QoS buttons_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    gamepad_button_subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/gamepad_buttons", buttons_qos,
        std::bind(&GamepadController::gamepad_button_callback, this, std::placeholders::_1));

    std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

    // publish velocity at 10hz regardless of joystick input rate
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&GamepadController::publish_twist, this));

    RCLCPP_INFO(this->get_logger(), "gamepad controller ready. publishing to: %s", cmd_vel_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "gamepad controls: axis[%d/%d] = movement, axis[%d] = rotation, button[%d] = turbo",
                axis_linear_x_, axis_linear_y_, axis_angular_z_, button_turbo_);

    if (axis_linear_x_ < 0 || axis_linear_y_ < 0 || axis_angular_z_ < 0) {
        RCLCPP_WARN(this->get_logger(),
                    "axis mappings unset (-1). publish [x,y,z] axis indices to /gamepad_buttons to configure.");
    }
}

GamepadController::~GamepadController() {}


// handles axis remapping messages. expects an array of exactly 3 integers
// representing the joystick axis indices for [x, y, z] movement.
// updates both the node parameters (visible externally) and local values.
void GamepadController::gamepad_button_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 3) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "received /gamepad_buttons with %zu elements — expected 3 [x,y,z]",
                             msg->data.size());
        return;
    }

    int new_x = msg->data[0];
    int new_y = msg->data[1];
    int new_z = msg->data[2];

    if (new_x < 0 || new_y < 0 || new_z < 0) {
        RCLCPP_WARN(this->get_logger(),
                    "ignoring /gamepad_buttons remap with negative index: [%d,%d,%d]",
                    new_x, new_y, new_z);
        return;
    }

    this->set_parameter(rclcpp::Parameter("axis_linear_x", new_x));
    this->set_parameter(rclcpp::Parameter("axis_linear_y", new_y));
    this->set_parameter(rclcpp::Parameter("axis_angular_z", new_z));

    axis_linear_x_  = new_x;
    axis_linear_y_  = new_y;
    axis_angular_z_ = new_z;

    RCLCPP_INFO(this->get_logger(), "axis mapping updated -> x:%d y:%d z:%d",
                axis_linear_x_, axis_linear_y_, axis_angular_z_);
}


// reads raw axis values from the joystick message, applies deadzone filtering,
// and checks the turbo button state. values are stored for use by publish_twist.
void GamepadController::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (axis_linear_x_ < 0 || axis_linear_y_ < 0 || axis_angular_z_ < 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "axis mappings not set — awaiting /gamepad_buttons [x,y,z]");
        return;
    }

    const size_t axes_sz = msg->axes.size();
    if (static_cast<size_t>(axis_linear_x_) >= axes_sz ||
        static_cast<size_t>(axis_linear_y_) >= axes_sz ||
        static_cast<size_t>(axis_angular_z_) >= axes_sz ||
        msg->buttons.size() <= static_cast<size_t>(button_turbo_)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "controller does not have enough axes or buttons for the configured mapping");
        return;
    }

    // apply deadzone — values below the threshold are treated as zero
    auto apply_deadzone = [this](double value) {
        return (std::abs(value) < deadzone_) ? 0.0 : value;
    };

    linear_x_  = apply_deadzone(msg->axes[axis_linear_x_]);
    linear_y_  = apply_deadzone(msg->axes[axis_linear_y_]);
    angular_z_ = apply_deadzone(msg->axes[axis_angular_z_]);
    turbo_mode_ = (msg->buttons[button_turbo_] == 1);

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "joy input — linear: [%.2f, %.2f], angular: %.2f, turbo: %s",
                          linear_x_, linear_y_, angular_z_, turbo_mode_ ? "on" : "off");
}


// builds a twist message from the current movement state and publishes it.
// if turbo mode is active, scales are multiplied by the turbo multiplier.
void GamepadController::publish_twist() {
    double lin_scale = linear_scale_  * (turbo_mode_ ? turbo_multiplier_ : 1.0);
    double ang_scale = angular_scale_ * (turbo_mode_ ? turbo_multiplier_ : 1.0);

    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x  = linear_x_  * lin_scale;
    twist_msg.linear.y  = linear_y_  * lin_scale;
    twist_msg.linear.z  = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = angular_z_ * ang_scale;

    cmd_vel_publisher_->publish(twist_msg);

    if (std::abs(linear_x_) > 0.01 || std::abs(linear_y_) > 0.01 || std::abs(angular_z_) > 0.01) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                              "publishing cmd_vel — linear: [%.2f, %.2f], angular: %.2f",
                              twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z);
    }
}


} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::GamepadController)
