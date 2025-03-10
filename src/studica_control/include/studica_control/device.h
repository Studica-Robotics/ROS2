#ifndef DEVICE_H
#define DEVICE_H

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "studica_control/srv/set_data.hpp" 

class Device : public rclcpp::Node {
public:
    Device(const std::string &name) : rclcpp::Node(name) {}
    virtual void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) = 0;
};

#endif // DEVICE_H
