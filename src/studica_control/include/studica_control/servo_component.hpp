/*
 * servo_component.h
 *
 * ros2 component for a servo motor connected to a vmx-pi pwm output channel.
 * three servo types are supported:
 *   standard   — position servo, -150 to 150 degrees
 *   continuous — spinning servo, -100 to 100 (speed)
 *   linear     — linear actuator, 0 to 100 (percent extension)
 *
 * topic (subscribes): <name>/cmd (std_msgs/Float64)
 *   set the target value directly. units depend on servo type:
 *     standard:   degrees (-150 to 150)
 *     continuous: speed   (-100 to 100)
 *     linear:     percent (0 to 100)
 *
 * topic (publishes): <name>/state (std_msgs/Float64)
 *   last commanded value, published at 20hz.
 *
 * service: <name>/set_servo (studica_control/SetData)
 *   initparams.speed — target value (same units as the cmd topic)
 */

#ifndef SERVO_COMPONENT_H
#define SERVO_COMPONENT_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "servo.hpp"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {

class Servo : public rclcpp::Node {
public:
    static std::vector<std::shared_ptr<rclcpp::Node>> initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);

    explicit Servo(const rclcpp::NodeOptions &options);

    Servo(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex port,
          studica_driver::ServoType type, int min, int max);

    ~Servo();

private:
    std::shared_ptr<studica_driver::Servo> servo_;
    std::shared_ptr<VMXPi> vmx_;
    VMXChannelIndex port_;
    studica_driver::ServoType type_;

    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void set_value(double value);
    void cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                      std::shared_ptr<studica_control::srv::SetData::Response> response);
    void publish_state();
};

} // namespace studica_control

#endif // SERVO_COMPONENT_H
