#ifndef DEVICE_H
#define DEVICE_H

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include "studica_control/srv/set_data.hpp" 

class Device : public rclcpp::Node {
public:
    // Constructor initializes the Node with a name
    Device(const std::string &name) : rclcpp::Node(name) {}

    // Pure virtual function to be overridden by derived classes
    virtual void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) = 0;
};

#endif // DEVICE_H
