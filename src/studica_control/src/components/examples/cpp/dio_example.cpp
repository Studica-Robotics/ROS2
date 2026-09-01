/*
 * dio_example.cpp
 *
 * Read digital pin state and drive an output pin.
 * Publishes alternating high/low to /dio/cmd every 2 seconds and prints the
 * resulting /dio/state feedback.
 *
 * Run:      ros2 run studica_control dio_example
 * Requires: studica_launch.py running, dio enabled in params.yaml
 *           sensors: ["dio"], dio.type: output
 *           (set type: input to observe an input pin without driving it)
 *
 * Topic published:   /dio/cmd    (std_msgs/Bool)  set pin high/low (output only)
 * Topic subscribed:  /dio/state  (std_msgs/Bool)  current pin state at 10 Hz
 * Service:           /dio/dio_cmd  (studica_control/SetData)
 *   Commands: toggle
 */

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "studica_control/srv/set_data.hpp"

using namespace std::chrono_literals;

static const std::string SENSOR = "dio";

class DioExample : public rclcpp::Node {
public:
    DioExample() : Node("dio_example"), pin_state_(false) {
        cmd_pub_ = create_publisher<std_msgs::msg::Bool>("/" + SENSOR + "/cmd", 10);

        state_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/" + SENSOR + "/state", 10,
            [this](std_msgs::msg::Bool::SharedPtr msg) {
                RCLCPP_INFO(get_logger(), "pin state: %s", msg->data ? "HIGH" : "LOW");
            });

        client_ = create_client<studica_control::srv::SetData>("/" + SENSOR + "/dio_cmd");

        // drive the pin at 0.5 Hz using cmd topic
        drive_timer_ = create_wall_timer(2s, std::bind(&DioExample::toggle_output, this));

        RCLCPP_INFO(get_logger(), "DIO example ready (sensor: %s)", SENSOR.c_str());
    }

private:
    void toggle_output() {
        pin_state_ = !pin_state_;
        std_msgs::msg::Bool msg;
        msg.data = pin_state_;
        cmd_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "set pin %s via topic", pin_state_ ? "HIGH" : "LOW");
    }

    void toggle_via_service() {
        if (!client_->wait_for_service(1s)) {
            RCLCPP_WARN(get_logger(), "dio_cmd service not available");
            return;
        }
        auto req = std::make_shared<studica_control::srv::SetData::Request>();
        req->params = "toggle";
        client_->async_send_request(req,
            [this](rclcpp::Client<studica_control::srv::SetData>::SharedFuture f) {
                RCLCPP_INFO(get_logger(), "toggle: %s", f.get()->message.c_str());
            });
    }

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_sub_;
    rclcpp::Client<studica_control::srv::SetData>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr drive_timer_;
    bool pin_state_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DioExample>());
    rclcpp::shutdown();
    return 0;
}
