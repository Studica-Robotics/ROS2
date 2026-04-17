/*
 * gamepad_component.h
 *
 * ros2 component that reads joystick input and publishes velocity commands.
 * acts as a bridge between a connected gamepad (via the joy node) and any
 * node that accepts geometry_msgs/Twist commands, such as a drive controller.
 *
 * you configure which joystick axes map to which movement directions, set a
 * deadzone to filter out stick drift, and optionally enable a turbo button
 * that multiplies speed when held.
 *
 * topic (subscribes): /joy (sensor_msgs/Joy)
 *   raw joystick input from the ros2 joy node. run: ros2 run joy joy_node
 *
 * topic (subscribes): /gamepad_buttons (std_msgs/Int32MultiArray)
 *   axis index mapping: [x_axis, y_axis, z_axis]. send this once to configure
 *   which physical axes control each direction. uses transient local qos so
 *   the mapping is remembered even if sent before this node starts.
 *
 * topic (publishes): <cmd_vel_topic> (geometry_msgs/Twist)
 *   velocity command published at 10hz regardless of joystick input rate.
 */

#ifndef STUDICA_CONTROL__GAMEPAD_COMPONENT_H_
#define STUDICA_CONTROL__GAMEPAD_COMPONENT_H_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

namespace studica_control {

// gamepadcontroller — joystick to velocity command bridge.
class GamepadController : public rclcpp::Node {
public:
    // loads params.yaml and creates the gamepad node under its own namespace
    static std::shared_ptr<rclcpp::Node> initialize(rclcpp::Node *control);

    // composable node constructor — used when loading as a ros2 plugin
    explicit GamepadController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    ~GamepadController();

private:
    // processes raw joystick input: applies deadzone, stores movement state
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    // handles axis remapping messages from /gamepad_buttons
    void gamepad_button_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

    // builds and publishes a twist message from the current movement state
    void publish_twist();

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr gamepad_button_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // scaling factors applied to raw axis values before publishing
    double linear_scale_;
    double angular_scale_;
    double turbo_multiplier_;
    double deadzone_;

    // current movement state (updated by joy_callback, read by publish_twist)
    double linear_x_;
    double linear_y_;
    double angular_z_;
    bool turbo_mode_;

    // joystick axis and button index for each control (set via params or /gamepad_buttons)
    int axis_linear_x_;
    int axis_linear_y_;
    int axis_angular_z_;
    int button_turbo_;
};

} // namespace studica_control

#endif // STUDICA_CONTROL__GAMEPAD_COMPONENT_H_
