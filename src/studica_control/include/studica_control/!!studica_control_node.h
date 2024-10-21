#ifndef STUDICA_CONTROL_NODE_H
#define STUDICA_CONTROL_NODE_H

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <rclcpp/rclcpp.hpp>
# include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/imu.hpp>

// SRV FILES 
# include <studica_control/srv/set_data.hpp> // <- add srv, action, msg here
# include <studica_control/srv/control_imu.hpp>
#endif

#include <rclcpp/rclcpp.hpp>
#include "VMXPi.h"
namespace studica_control
{

class StudicaControlServer : public rclcpp::Node {
public:
    // Constructor
    StudicaControlServer();
    void set_executor(const std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>& exec);
    void set_hal(const std::shared_ptr<VMXPi>& vmx);

private:
    // Service callbacks
    // void handle_read_data(const std::shared_ptr<studica_control::srv::Trigger::Request> request,
    //                       std::shared_ptr<studica_control::srv::Trigger::Response> response);

    // void handle_set_data(const std::shared_ptr<studica_control::srv::SetBool::Request> request,
    //                      std::shared_ptr<studica_control::srv::SetBool::Response> response);

    void handle_imu_service(const std::shared_ptr<studica_control::srv::ControlImu::Request> request,
                            std::shared_ptr<studica_control::srv::ControlImu::Response> response);

    // void handle_motor_service(const std::shared_ptr<studica_control::srv::ControlMotor::Request> request,
    //                            std::shared_ptr<studica_control::srv::ControlMotor::Response> response);

    // Publisher and timer
    void publish_imu_data();
    // void set_motor_speed(double speed);

    // Services and publishers
    rclcpp::Service<studica_control::srv::ControlImu>::SharedPtr imu_service_;
    // rclcpp::Service<studica_control::srv::ControlMotor>::SharedPtr motor_service_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}

#endif // STUDICA_CONTROL_NODE_H
