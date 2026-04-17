/*
 * dio_component.h
 *
 * ros2 component for a digital input/output (dio) pin on the vmx-pi board.
 * a digital pin carries a simple on/off signal — high (3.3v) or low (0v).
 * you can configure it as an input (read a button or sensor) or an output
 * (drive a light, relay, or other digital device). supports multiple pins.
 *
 * topic (publishes): <topic> (std_msgs/Bool)
 *   current pin state — true = high, false = low, published at 10hz.
 *
 * service: dio_cmd (studica_control/SetData)
 *   params field sets the command. available commands:
 *     toggle — flip the output between high and low (output mode only)
 */

#ifndef DIO_COMPONENT_H
#define DIO_COMPONENT_H

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "dio.h"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {

// dio — digital input/output pin node. one instance per pin.
class DIO : public rclcpp::Node {
public:
    // reads params.yaml and creates one node per dio entry in the list
    static std::vector<std::shared_ptr<rclcpp::Node>> initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);

    // composable node constructor — used when loading as a ros2 plugin
    explicit DIO(const rclcpp::NodeOptions &options);

    // main constructor — connects to the pin and sets up topics/services
    DIO(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex pin,
        studica_driver::PinMode pin_mode, const std::string &topic);

    ~DIO();

private:
    std::shared_ptr<studica_driver::DIO> dio_;
    std::shared_ptr<VMXPi> vmx_;
    VMXChannelIndex pin_;
    studica_driver::PinMode pin_mode_;  // input or output

    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // handles incoming service commands
    void cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                      std::shared_ptr<studica_control::srv::SetData::Response> response);

    // reads pin state and publishes it
    void publish_dio_state();
};

} // namespace studica_control

#endif // DIO_COMPONENT_H
