#ifndef IMU_DRIVER_HPP_
#define IMU_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "VMXPi.h"
# include <studica_control/srv/set_data.hpp>
#include <std_msgs/msg/string.hpp>

class ImuDriver : public rclcpp::Node {
public:
    ImuDriver();
    int cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response);

private:
    void publish_imu_data();
    void Spin();
    void publish_message();

    std::shared_ptr<VMXPi> vmx;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    int count_;
};

#endif // IMU_DRIVER_HPP_
