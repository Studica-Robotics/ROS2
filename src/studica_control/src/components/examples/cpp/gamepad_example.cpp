/*
 * gamepad_example.cpp
 *
 * Subscribe to cmd_vel output from the gamepad controller.
 * Also demonstrates publishing to /gamepad_axis_remap to remap axes at runtime.
 *
 * Setup (3 terminals):
 *   1. ros2 launch studica_control studica_launch.py  (gamepad.enabled: true)
 *   2. ros2 run joy joy_node
 *   3. ros2 run studica_control gamepad_example
 *
 * Run 'ros2 topic echo /joy' and move sticks to find axis indices, then set
 * them in params.yaml or publish to /gamepad_axis_remap at runtime.
 *
 * Topic subscribed:  cmd_vel  (geometry_msgs/Twist)
 * Topic published:   /gamepad_axis_remap  (std_msgs/Int32MultiArray)
 *   publish [x_axis, y_axis, z_axis] indices to remap without restarting
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

class GamepadExample : public rclcpp::Node {
public:
    GamepadExample() : Node("gamepad_example") {
        sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&GamepadExample::on_twist, this, std::placeholders::_1));

        remap_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
            "/gamepad_axis_remap",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

        RCLCPP_INFO(get_logger(), "listening on /cmd_vel");
        RCLCPP_INFO(get_logger(),
            "to remap axes: publish [x,y,z] indices to /gamepad_axis_remap");
    }

    // call this from your code to remap axes without restarting the driver
    // example: remap(1, -1, 0) — forward/back on axis 1, no strafe, rotate on axis 0
    void remap(int x_axis, int y_axis, int z_axis) {
        std_msgs::msg::Int32MultiArray msg;
        msg.data = {x_axis, y_axis, z_axis};
        remap_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "axis remap -> x:%d y:%d z:%d", x_axis, y_axis, z_axis);
    }

private:
    void on_twist(const geometry_msgs::msg::Twist::SharedPtr msg) {
        RCLCPP_INFO(get_logger(),
            "cmd_vel — linear: (%.2f, %.2f)  angular: %.2f",
            msg->linear.x, msg->linear.y, msg->angular.z);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr remap_pub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GamepadExample>());
    rclcpp::shutdown();
    return 0;
}
