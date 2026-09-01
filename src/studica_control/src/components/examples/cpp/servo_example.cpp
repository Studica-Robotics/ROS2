/*
 * servo_example.cpp
 *
 * Command a servo and read back its state.
 * Moves through 90°, -90°, and 0° with 3-second pauses between each.
 *
 * Run:      ros2 run studica_control servo_example
 * Requires: studica_launch.py running, servo enabled in params.yaml
 *           sensors: ["servo"], servo.type: standard
 *
 * Topic published:   /servo/cmd    (std_msgs/Float64)  target degrees -150 to 150
 * Topic subscribed:  /servo/state  (std_msgs/Float64)  last commanded value at 20 Hz
 * Service:           /servo/set_servo  (studica_control/SetData)
 *   initparams.speed — target value (same units as cmd topic)
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "studica_control/srv/set_data.hpp"

using namespace std::chrono_literals;

static const std::string SENSOR = "servo";

class ServoExample : public rclcpp::Node {
public:
    ServoExample() : Node("servo_example"), step_(0), targets_({90.0, -90.0, 0.0}) {
        cmd_pub_ = create_publisher<std_msgs::msg::Float64>("/" + SENSOR + "/cmd", 10);

        state_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/" + SENSOR + "/state", 10,
            [this](std_msgs::msg::Float64::SharedPtr msg) {
                RCLCPP_INFO(get_logger(), "servo state: %.1f", msg->data);
            });

        client_ = create_client<studica_control::srv::SetData>("/" + SENSOR + "/set_servo");

        // move to each target with 3-second intervals
        timer_ = create_wall_timer(3s, std::bind(&ServoExample::next_position, this));
        RCLCPP_INFO(get_logger(), "servo example ready (sensor: %s)", SENSOR.c_str());
    }

private:
    void set_angle(double degrees) {
        std_msgs::msg::Float64 msg;
        msg.data = degrees;
        cmd_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "commanding %.1f degrees", degrees);
    }

    void set_angle_via_service(double degrees) {
        if (!client_->wait_for_service(2s)) {
            RCLCPP_ERROR(get_logger(), "set_servo service not available");
            return;
        }
        auto req = std::make_shared<studica_control::srv::SetData::Request>();
        req->initparams.speed = static_cast<float>(degrees);
        client_->async_send_request(req,
            [this, degrees](rclcpp::Client<studica_control::srv::SetData>::SharedFuture f) {
                RCLCPP_INFO(get_logger(), "set_servo(%.1f): %s", degrees, f.get()->message.c_str());
            });
    }

    void next_position() {
        if (step_ >= static_cast<int>(targets_.size())) {
            timer_->cancel();
            RCLCPP_INFO(get_logger(), "sequence complete");
            return;
        }
        double target = targets_[step_++];

        // alternate between topic and service to demonstrate both
        if (step_ % 2 == 0) {
            set_angle_via_service(target);
        } else {
            set_angle(target);
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr state_sub_;
    rclcpp::Client<studica_control::srv::SetData>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    int step_;
    std::vector<double> targets_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServoExample>());
    rclcpp::shutdown();
    return 0;
}
