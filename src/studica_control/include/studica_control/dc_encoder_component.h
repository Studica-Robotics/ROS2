/*
 * dc_encoder_component.h
 *
 * ros2 component for a duty cycle (pwm-based) absolute encoder.
 * unlike a quadrature encoder that counts steps, a duty cycle encoder
 * reports an absolute position by encoding the angle as a pulse width.
 * this means it always knows where it is, even after a power cycle.
 * useful for joints or mechanisms where absolute position matters.
 *
 * topic (publishes): <topic> (studica_control/DutyCycleEncoderMsg)
 *   absolute position and frequency data, published at 20hz.
 *     output     — position as a value between 0.0 and 1.0
 *     frequency  — pulse frequency in hz
 *
 * service: dc_encoder_cmd (studica_control/SetData)
 *   params field sets the command. available commands:
 *     get_output    — return the position output (0.0–1.0) as a string
 *     get_frequency — return the pulse frequency in hz as a string
 */

#ifndef DC_ENCODER_COMPONENT_H
#define DC_ENCODER_COMPONENT_H

#include "rclcpp/rclcpp.hpp"

#include "duty_cycle_encoder.h"
#include "studica_control/msg/duty_cycle_encoder_msg.hpp"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {

// dutycycleencoder — absolute position encoder node. one instance per encoder.
class DutyCycleEncoder : public rclcpp::Node {
public:
    // reads params.yaml and creates one node per dc_encoder entry in the list
    static std::vector<std::shared_ptr<rclcpp::Node>> initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);

    // composable node constructor — used when loading as a ros2 plugin
    explicit DutyCycleEncoder(const rclcpp::NodeOptions &options);

    // main constructor — connects to the encoder and sets up topics/services
    DutyCycleEncoder(std::shared_ptr<VMXPi> vmx, const std::string &name,
                     VMXChannelIndex port, const std::string &topic);

    ~DutyCycleEncoder();

private:
    std::shared_ptr<studica_driver::DutyCycleEncoder> duty_cycle_encoder_;
    std::shared_ptr<VMXPi> vmx_;
    VMXChannelIndex port_;

    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::Publisher<studica_control::msg::DutyCycleEncoderMsg>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // receives the raw service request and forwards it to cmd()
    void cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                      std::shared_ptr<studica_control::srv::SetData::Response> response);

    // parses the command string and returns the requested reading
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response);

    // reads position and frequency, then publishes them
    void publish_data();

    void DisplayVMXError(VMXErrorCode vmxerr);
};

} // namespace studica_control

#endif // DC_ENCODER_COMPONENT_H
