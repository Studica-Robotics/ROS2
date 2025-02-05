#ifndef MECANUM_DRIVE_COMPONENT_H
#define MECANUM_DRIVE_COMPONENT_H

#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "mecanum_drive_odometry.h"
#include "studica_control/srv/set_data.hpp"
#include "titan.h"
#include "VMXManager.h"
#include "VMXPi.h"  


namespace studica_control {

class MecanumDrive : public rclcpp::Node {
public:
    MecanumDrive(std::shared_ptr<VMXPi> vmx, const std::string &name, const uint8_t &canID, const uint16_t &motorFreq, const float &ticksPerRotation, const float &wheel_radius, const float &wheelbase, const float &width);
    explicit MecanumDrive(const rclcpp::NodeOptions & options);
    ~MecanumDrive();
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void publish_odometry();

private:
    std::shared_ptr<VMXPi> vmx_;
    std::shared_ptr<studica_driver::Titan> titan_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<MecanumOdometry> odom_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    double length_x_;
    double length_y_;

    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;

    std::string name_;
    uint8_t canID_;
    uint16_t motorFreq_;
    float ticksPerRotation_;
    float distPerTick_;
    float wheel_radius_;
    float wheelbase_;
    float width_;
   
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
};

}  // namespace studica_control

#endif  // TITAN_COMPONENT_H
