#ifndef IMU_COMPONENT_H
#define IMU_COMPONENT_H

#include <stdio.h>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "imu.h"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {

class Imu : public rclcpp::Node {
    
public:
    explicit Imu(const rclcpp::NodeOptions &options); 
    Imu(std::shared_ptr<VMXPi> vmx);
    ~Imu();
    std::shared_ptr<studica_driver::Imu> imu_;

private:
    void cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                      std::shared_ptr<studica_control::srv::SetData::Response> response);

    std::shared_ptr<VMXPi> vmx_;
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;

    void DisplayVMXError(VMXErrorCode vmxerr);
};

} // namespace studica_control

#endif // IMU_COMPONENT_H
