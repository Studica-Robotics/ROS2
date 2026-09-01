/*
 * dc_encoder_example.cpp
 *
 * Subscribe to duty cycle (absolute) encoder position data.
 * Demonstrates the get_absolute_position service command.
 *
 * Run:      ros2 run studica_control dc_encoder_example
 * Requires: studica_launch.py running, duty_cycle enabled in params.yaml
 *           sensors: ["duty_cycle"], duty_cycle.topic: duty_cycle_encoder
 *
 * Topic subscribed: <topic>  (studica_control/DutyCycleEncoderMsg)
 *   absolute_angle  — angle within one revolution (degrees)
 *   rollover_count  — number of complete revolutions
 *   total_rotation  — total rotation across all revolutions (degrees)
 * Service: /duty_cycle/duty_cycle_encoder_cmd  (studica_control/SetData)
 *   Commands: get_absolute_position, get_rollover_count, get_total_rotation
 */

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "studica_control/msg/duty_cycle_encoder_msg.hpp"
#include "studica_control/srv/set_data.hpp"

using namespace std::chrono_literals;

static const std::string SENSOR = "duty_cycle";

class DcEncoderExample : public rclcpp::Node {
public:
    DcEncoderExample() : Node("dc_encoder_example") {
        // topic name is set in params.yaml under duty_cycle.<sensor>.topic
        sub_ = create_subscription<studica_control::msg::DutyCycleEncoderMsg>(
            "duty_cycle_encoder", 10,
            std::bind(&DcEncoderExample::on_data, this, std::placeholders::_1));

        client_ = create_client<studica_control::srv::SetData>(
            "/" + SENSOR + "/duty_cycle_encoder_cmd");

        poll_timer_ = create_wall_timer(5s, std::bind(&DcEncoderExample::poll_service, this));

        RCLCPP_INFO(get_logger(), "listening on duty_cycle_encoder topic");
    }

private:
    void on_data(const studica_control::msg::DutyCycleEncoderMsg::SharedPtr msg) {
        RCLCPP_INFO(get_logger(),
            "angle: %.2f deg  rollovers: %d  total: %.2f deg",
            msg->absolute_angle, msg->rollover_count, msg->total_rotation);
    }

    void poll_service() {
        if (!client_->wait_for_service(1s)) {
            RCLCPP_WARN(get_logger(), "duty_cycle_encoder_cmd service not available");
            return;
        }
        auto req = std::make_shared<studica_control::srv::SetData::Request>();
        req->params = "get_absolute_position";
        client_->async_send_request(req,
            [this](rclcpp::Client<studica_control::srv::SetData>::SharedFuture f) {
                RCLCPP_INFO(get_logger(), "service get_absolute_position: %s",
                            f.get()->message.c_str());
            });
    }

    rclcpp::Subscription<studica_control::msg::DutyCycleEncoderMsg>::SharedPtr sub_;
    rclcpp::Client<studica_control::srv::SetData>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr poll_timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DcEncoderExample>());
    rclcpp::shutdown();
    return 0;
}
