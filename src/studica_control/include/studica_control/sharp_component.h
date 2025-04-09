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
    static std::vector<std::shared_ptr<rclcpp::Node>> initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);
    explicit Sharp(const rclcpp::NodeOptions &options);
    Sharp(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex port, const std::string &topic);
    ~Sharp();

private:
    std::shared_ptr<studica_driver::Sharp> sharp_;
    std::shared_ptr<VMXPi> vmx_;
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    VMXChannelIndex port_;
    void cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void publish_range();
};

} // namespace studica_control

#endif // SHARP_COMPONENT_H
