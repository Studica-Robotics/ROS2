/*
 * imu_example.cpp
 *
 * Subscribe to IMU orientation, angular velocity, and linear acceleration.
 * Demonstrates the zero_yaw service command after 5 seconds.
 *
 * Run:      ros2 run studica_control imu_example
 * Requires: studica_launch.py running, imu enabled in params.yaml
 *           imu.topic: imu
 *
 * Topic subscribed: imu  (sensor_msgs/Imu)
 * Service:          /imu/get_imu_data  (studica_control/SetData)
 *   Commands: zero_yaw, <anything else> returns pitch/yaw/roll string
 */

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "studica_control/srv/set_data.hpp"

using namespace std::chrono_literals;

class ImuExample : public rclcpp::Node {
public:
    ImuExample() : Node("imu_example"), zeroed_(false) {
        sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu", 10,
            std::bind(&ImuExample::on_imu, this, std::placeholders::_1));

        client_ = create_client<studica_control::srv::SetData>("/imu/get_imu_data");

        // zero yaw once after 5 seconds
        zero_timer_ = create_wall_timer(5s, std::bind(&ImuExample::zero_yaw, this));

        RCLCPP_INFO(get_logger(), "listening on imu topic — will zero yaw in 5 s");
    }

private:
    void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
        const auto &q  = msg->orientation;
        const auto &av = msg->angular_velocity;
        const auto &la = msg->linear_acceleration;
        RCLCPP_INFO(get_logger(),
            "orientation (x,y,z,w): %.3f %.3f %.3f %.3f  "
            "gyro: %.3f %.3f %.3f  accel: %.3f %.3f %.3f",
            q.x, q.y, q.z, q.w,
            av.x, av.y, av.z,
            la.x, la.y, la.z);
    }

    void zero_yaw() {
        zero_timer_->cancel();  // fire once
        if (!client_->wait_for_service(2s)) {
            RCLCPP_ERROR(get_logger(), "get_imu_data service not available");
            return;
        }
        auto req = std::make_shared<studica_control::srv::SetData::Request>();
        req->params = "zero_yaw";
        client_->async_send_request(req,
            [this](rclcpp::Client<studica_control::srv::SetData>::SharedFuture f) {
                RCLCPP_INFO(get_logger(), "zero_yaw: %s", f.get()->message.c_str());
            });
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
    rclcpp::Client<studica_control::srv::SetData>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr zero_timer_;
    bool zeroed_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuExample>());
    rclcpp::shutdown();
    return 0;
}
