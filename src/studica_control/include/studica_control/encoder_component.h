/*
 * encoder_component.h
 *
 * ros2 component for a quadrature encoder connected to two vmx-pi digital input channels.
 * a quadrature encoder uses two signal wires (channel a and channel b) to measure
 * how far a shaft has turned and in which direction. supports multiple encoders.
 *
 * topic (publishes): <topic> (studica_control/EncoderMsg)
 *   tick count and direction, published at 20hz.
 *     count     — total tick count since last reset (or since startup)
 *     direction — 1 = forward, -1 = reverse
 *
 * service: encoder_cmd (studica_control/SetData)
 *   params field sets the command. available commands:
 *     get_count     — return the current tick count as a string
 *     get_direction — return the current direction as a string ("1" or "-1")
 *     reset         — reset the tick count to zero
 */

#ifndef ENCODER_COMPONENT_H
#define ENCODER_COMPONENT_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "encoder.h"
#include "studica_control/msg/encoder_msg.hpp"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {

// encoder — quadrature encoder node. one instance per encoder.
class Encoder : public rclcpp::Node {
public:
    // reads params.yaml and creates one node per encoder entry in the list
    static std::vector<std::shared_ptr<rclcpp::Node>> initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);

    // composable node constructor — used when loading as a ros2 plugin
    explicit Encoder(const rclcpp::NodeOptions &options);

    // main constructor — connects to the encoder and sets up topics/services
    Encoder(std::shared_ptr<VMXPi> vmx, const std::string &name,
            VMXChannelIndex port_a, VMXChannelIndex port_b, const std::string &topic);

    ~Encoder();

private:
    std::shared_ptr<VMXPi> vmx_;
    std::shared_ptr<studica_driver::Encoder> encoder_;
    VMXChannelIndex port_a_;  // channel a — primary counting signal
    VMXChannelIndex port_b_;  // channel b — used to determine direction

    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::Publisher<studica_control::msg::EncoderMsg>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // receives the raw service request and forwards it to cmd()
    void cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request,
                      std::shared_ptr<studica_control::srv::SetData::Response> response);

    // parses the command string and calls the appropriate encoder driver function
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response);

    // reads tick count and direction, then publishes them
    void publish_data();

    void DisplayVMXError(VMXErrorCode vmxerr);
};

} // namespace studica_control

#endif // ENCODER_COMPONENT_H
