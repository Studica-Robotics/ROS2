/*
 * servo_component.h
 *
 * ros2 component for a servo motor connected to a vmx-pi pwm output channel.
 * servos are small motors that can be told to move to a specific position or spin
 * at a set speed. three servo types are supported, each with a different use case:
 *   standard   — position servo, accepts an angle from -150 to 150 degrees
 *   continuous — spinning servo, accepts a speed from -100 to 100
 *   linear     — linear actuator, accepts a position from 0 to 100 percent
 *
 * topic (publishes): <topic> (std_msgs/Float32)
 *   last commanded angle or speed value, published at 20hz.
 *
 * service: <name>/set_servo_angle (studica_control/SetData)
 *   pass the target value as a plain number in the params field.
 *   example: params = "90" moves a standard servo to 90 degrees.
 */

#ifndef SERVO_COMPONENT_H
#define SERVO_COMPONENT_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "servo.h"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {

// servo — servo motor node. one instance per servo.
class Servo : public rclcpp::Node {
public:
    // reads params.yaml and creates one node per servo entry in the list
    static std::vector<std::shared_ptr<rclcpp::Node>> initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);

    // composable node constructor — used when loading as a ros2 plugin
    explicit Servo(const rclcpp::NodeOptions &options);

    // main constructor — connects to the servo and sets up topics/services
    Servo(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex port,
          studica_driver::ServoType type, int min, int max, const std::string &topic);

    ~Servo();

private:
    std::shared_ptr<studica_driver::Servo> servo_;
    std::shared_ptr<VMXPi> vmx_;
    VMXChannelIndex port_;
    studica_driver::ServoType type_;  // standard, continuous, or linear

    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // parses the target value from params and moves the servo
    void cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                      std::shared_ptr<studica_control::srv::SetData::Response> response);

    // publishes the last commanded angle or speed
    void publish_angle();

    void DisplayVMXError(VMXErrorCode vmxerr);
};

} // namespace studica_control

#endif // SERVO_COMPONENT_H
