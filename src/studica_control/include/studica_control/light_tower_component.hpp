/*
 * light_tower_component.hpp
 *
 * ROS2 component for a 5-output LED light tower (red/green/yellow/buzzer + continuous).
 * Enforces mutual exclusion — activating a new colour/buzzer automatically clears
 * the current one.
 *
 * service: <name>/set  (studica_control/SetData)
 *   params:
 *     "off"              — everything off
 *     "<color>"          — solid on  (colors: red, green, yellow, buzzer)
 *     "<color>:blink"    — software blink at default_blink_hz (from params.yaml)
 *     "<color>:blink_hw" — hardware blink (continuous pin LOW, hardware drives rate)
 *     "<color>:<hz>"     — software blink at a specific hz (e.g. "red:2.5")
 *
 * topic (publishes): <name>/state (std_msgs/String)
 *   current state string, e.g. "off", "red:solid", "green:blink:1.0", "yellow:blink_hw"
 *   published on every state change and at 1 hz as a heartbeat.
 */

#ifndef LIGHT_TOWER_COMPONENT_HPP
#define LIGHT_TOWER_COMPONENT_HPP

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "light_tower.hpp"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {

class LightTower : public rclcpp::Node {
public:
    static std::shared_ptr<rclcpp::Node> initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);

    explicit LightTower(const rclcpp::NodeOptions &options);

    LightTower(std::shared_ptr<VMXPi> vmx,
               const std::string &name,
               int pin_continuous,
               int pin_red,
               int pin_green,
               int pin_yellow,
               int pin_buzzer,
               double default_blink_hz);

    ~LightTower();

private:
    std::shared_ptr<studica_driver::LightTower> tower_;

    std::string active_color_{"none"};
    double      blink_hz_{0.0};
    bool        blink_state_{false};
    double      default_blink_hz_{1.0};

    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr       state_pub_;
    rclcpp::TimerBase::SharedPtr                              heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr                              blink_timer_;

    void cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request>  request,
                            std::shared_ptr<studica_control::srv::SetData::Response> response);

    void        set_output(const std::string &color, double blink_hz);
    void        set_pin(const std::string &color, bool value);
    void        clear_active();
    void        cancel_blink();
    std::string state_string() const;
    void        publish_state();
};

} // namespace studica_control

#endif // LIGHT_TOWER_COMPONENT_HPP
