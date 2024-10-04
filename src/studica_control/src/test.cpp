#include <rclcpp/rclcpp.hpp>
#include <studica_control/srv/set_data.hpp>  // Replace with your service header
#include <chrono>
#include <memory>
#include "studica_control/imu_driver_node.h"

#ifndef IMU_DRIVER_HPP_
#define IMU_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "VMXPi.h"
# include <studica_control/srv/set_data.hpp>

class ImuDriver : public rclcpp::Node {
public:
    ImuDriver();
    int cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response);

private:
    void publish_imu_data();
    void Spin();

    std::shared_ptr<VMXPi> vmx;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // IMU_DRIVER_HPP_

using namespace std::chrono_literals;
using SetData = studica_control::srv::SetData;  // Alias for service type

class ServiceClientNode : public rclcpp::Node
{
public:
    ServiceClientNode() : Node("service_client_node")
    {
        // Create the service client
        client_ = this->create_client<SetData>("/manage_dynamic_publisher");

        // Wait for the service to be available
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for the service to be available...");
        }

        // Prepare the service request with an example data value
        auto request = std::make_shared<SetData::Request>();
        request->command = "start";  // Example: sending integer 1
        request->name = "tester_publisher";
        // Call the service asynchronously
        auto future = client_->async_send_request(request);

        // Wait for the response
        try {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Service response: Success - %s", response->message.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Service response: Failure - %s", response->message.c_str());
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }

private:
    rclcpp::Client<SetData>::SharedPtr client_;  // Service client
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);  // Initialize ROS2
    auto node = std::make_shared<ServiceClientNode>();  // Create the service client node
    rclcpp::spin(node);  // Spin the node
    rclcpp::shutdown();  // Shutdown ROS2
    return 0;
}

// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     auto imu_node = std::make_shared<ImuDriver>();
//     // auto publisher_node = std::make_shared<ImuDriver>(name);

//     rclcpp::spin(imu_node);
//     rclcpp::shutdown();
//     return 0;
// }
