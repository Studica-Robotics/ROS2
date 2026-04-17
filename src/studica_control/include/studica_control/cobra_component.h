/*
 * cobra_component.h
 *
 * ros2 component for the studica cobra reflectance sensor array.
 * the cobra is a 4-channel analog sensor that measures how much infrared
 * light is reflected off a surface. each channel outputs a voltage between
 * 0 and vref — low voltage over dark surfaces, high voltage over light ones.
 * this makes it well suited for line following and surface detection.
 *
 * the sensor communicates over i2c and uses a configurable reference voltage
 * (vref) to scale the output range.
 *
 * topic (publishes): <topic> (std_msgs/Float32MultiArray)
 *   voltage readings for all 4 channels, published at 20hz.
 *   data[0..3] = voltage in volts for channels 0–3
 *
 * service: cobra_cmd (studica_control/SetData)
 *   params field sets the command. available commands:
 *     get_raw <channel>     — return the raw adc value for one channel (0–3)
 *     get_voltage <channel> — return the voltage in volts for one channel (0–3)
 */

#ifndef COBRA_COMPONENT_H
#define COBRA_COMPONENT_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "cobra.h"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {

// cobra — reflectance sensor array node. one instance per sensor board.
class Cobra : public rclcpp::Node {
public:
    // reads params.yaml and creates one node per cobra entry in the list
    static std::vector<std::shared_ptr<rclcpp::Node>> initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);

    // composable node constructor — used when loading as a ros2 plugin
    explicit Cobra(const rclcpp::NodeOptions &options);

    // main constructor — connects to the sensor and sets up topics/services
    Cobra(std::shared_ptr<VMXPi> vmx, const std::string &name,
          const float &vref, const std::string &topic);

    ~Cobra();

private:
    std::shared_ptr<VMXPi> vmx_;
    std::shared_ptr<studica_driver::Cobra> cobra_;
    float vref_;  // reference voltage — scales the output range of each channel

    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // receives the raw service request and forwards it to cmd()
    void cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request,
                      std::shared_ptr<studica_control::srv::SetData::Response> response);

    // parses the command string and returns the requested channel reading
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response);

    // reads all 4 channels and publishes their voltages
    void publish_data();
};

}  // namespace studica_control

#endif  // COBRA_COMPONENT_H
