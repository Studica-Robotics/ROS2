/*
 * dio_component.hpp
 *
 * ros2 component for a digital input/output pin on the vmx-pi board.
 * configure as input (reads a button/sensor) or output (drives a relay, LED, etc.).
 *
 * topic (publishes): <name>/state (std_msgs/Bool)
 *   current pin state — true = high, false = low.
 *   polling: published at 10 hz.
 *   interrupt: also published immediately on every edge when interrupt_edge is set.
 *
 * topic (subscribes, output mode only): <name>/cmd (std_msgs/Bool)
 *   set the pin high (true) or low (false) directly.
 *   ignored on input-mode pins.
 *
 * service: <name>/dio_cmd (studica_control/SetData)
 *   available commands:
 *     toggle — flip output between high and low (output mode only)
 *
 * interrupt support (input mode only):
 *   set interrupt_edge: "rising", "falling", or "none" (default).
 *   on a detected edge the state topic is published immediately in addition
 *   to the regular 10 hz poll. the interrupt fires in a VMX background
 *   thread — ros2 publishers are thread-safe so this is handled automatically.
 *
 *   use case example — emergency stop button:
 *     configure the e-stop pin as input with interrupt_edge: "rising".
 *     subscribe to /<name>/state in your safety node and publish 0 to
 *     /cmd_vel the instant the message arrives.
 */

#ifndef DIO_COMPONENT_H
#define DIO_COMPONENT_H

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "dio.hpp"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {

class DIO : public rclcpp::Node {
public:
    static std::vector<std::shared_ptr<rclcpp::Node>> initialize(rclcpp::Node *control,
                                                                   std::shared_ptr<VMXPi> vmx);

    explicit DIO(const rclcpp::NodeOptions &options);

    DIO(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex pin,
        studica_driver::PinMode pin_mode, const std::string &interrupt_edge = "none",
        int debounce_ms = 0);

    ~DIO();

private:
    std::shared_ptr<studica_driver::DIO> dio_;
    std::shared_ptr<VMXPi> vmx_;
    VMXChannelIndex pin_;
    studica_driver::PinMode pin_mode_;

    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cmd_sub_;  // output mode only
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                      std::shared_ptr<studica_control::srv::SetData::Response> response);
    void publish_dio_state();
};

} // namespace studica_control

#endif // DIO_COMPONENT_H
