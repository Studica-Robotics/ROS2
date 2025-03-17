#ifndef SHARP_COMPONENT_H
#define SHARP_COMPONENT_H

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

#include "sharp.h"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {
    
class Sharp : public rclcpp::Node {
public:
    explicit Sharp(const rclcpp::NodeOptions &options);
    Sharp(const std::string &name, VMXChannelIndex port, std::shared_ptr<VMXPi> vmx);
    ~Sharp();

private:
    std::shared_ptr<studica_driver::Sharp> sharp_;
    std::shared_ptr<VMXPi> vmx_;
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string name_;
    VMXChannelIndex port_;
    void cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void publish_range();
};

} // namespace studica_control

#endif // SHARP_COMPONENT_H
