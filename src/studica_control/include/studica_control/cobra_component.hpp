/*
 * cobra_component.h
 *
 * ros2 component for the studica cobra reflectance sensor array.
 * 4-channel analog sensor — each channel outputs a voltage between
 * 0 and vref. low voltage = dark surface, high voltage = light surface.
 * suited for line following and surface detection.
 *
 * topics (publish, 20hz):
 *   <name>/ch_0  (std_msgs/Float32) — voltage channel 0
 *   <name>/ch_1  (std_msgs/Float32) — voltage channel 1
 *   <name>/ch_2  (std_msgs/Float32) — voltage channel 2
 *   <name>/ch_3  (std_msgs/Float32) — voltage channel 3
 *
 * service: <name>/cobra_cmd (studica_control/SetData)
 *   initparams.n_encoder — channel to query (0–3)
 *   commands:
 *     get_raw     — raw ADC value for the specified channel
 *     get_voltage — voltage in volts for the specified channel
 */

#ifndef COBRA_COMPONENT_H
#define COBRA_COMPONENT_H

#include <array>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "cobra.hpp"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {

class Cobra : public rclcpp::Node {
public:
    static std::vector<std::shared_ptr<rclcpp::Node>> initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);

    explicit Cobra(const rclcpp::NodeOptions &options);

    Cobra(std::shared_ptr<VMXPi> vmx, const std::string &name, const float &vref);

    ~Cobra();

private:
    std::shared_ptr<VMXPi> vmx_;
    std::shared_ptr<studica_driver::Cobra> cobra_;
    float vref_;

    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, 4> channel_pubs_;
    rclcpp::TimerBase::SharedPtr timer_;

    void cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request,
                      std::shared_ptr<studica_control::srv::SetData::Response> response);
    void publish_data();
};

}  // namespace studica_control

#endif  // COBRA_COMPONENT_H
