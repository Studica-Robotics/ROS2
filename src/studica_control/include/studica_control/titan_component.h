#ifndef TITAN_COMPONENT_H
#define TITAN_COMPONENT_H

#include <string>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "diff_drive_odometry.h"
#include "studica_control/srv/set_data.hpp"
#include "titan.h"
#include "VMXManager.h"
#include "VMXPi.h"  


namespace studica_control {

class Titan : public rclcpp::Node {
public:
    Titan(std::shared_ptr<VMXPi> vmx, const std::string &name, const uint8_t &canID, const uint16_t &motorFreq, const float &distPerTick, const float &speed);
    explicit Titan(const rclcpp::NodeOptions & options);
    ~Titan();
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void publish_odometry();

private:
    std::shared_ptr<VMXPi> vmx_;
    std::shared_ptr<studica_driver::Titan> titan_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<Odometry> odom_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;

    double last_left_encoder_ = 0.0;
    double last_right_encoder_ = 0.0;

    double left_command_ = 0.0;
    double right_command_ = 0.0;

    double wheel_radius_ = 0.05;
    double wheel_separation_ = 0.25;

    std::string name_;
    uint8_t canID_;
    uint16_t motorFreq_;
    float distPerTick_;
    float speed_;
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
};
}  // namespace studica_control

#endif  // TITAN_COMPONENT_H
