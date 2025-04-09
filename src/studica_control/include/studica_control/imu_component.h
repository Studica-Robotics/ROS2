#ifndef IMU_COMPONENT_H
#define IMU_COMPONENT_H

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "imu.h"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {

class Imu : public rclcpp::Node {
    
public:
    static std::shared_ptr<rclcpp::Node> initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);
    explicit Imu(const rclcpp::NodeOptions &options); 
    Imu(std::shared_ptr<VMXPi> vmx, const std::string &name, const std::string &topic);
    ~Imu();
    
private:
    std::shared_ptr<studica_driver::Imu> imu_;
    std::shared_ptr<VMXPi> vmx_;
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
        std::shared_ptr<studica_control::srv::SetData::Response> response);
    void publish_data();
    void DisplayVMXError(VMXErrorCode vmxerr);
};

} // namespace studica_control

#endif // IMU_COMPONENT_H
