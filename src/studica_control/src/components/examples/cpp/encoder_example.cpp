/*
 * encoder_example.cpp
 *
 * Subscribe to quadrature encoder count and direction.
 * Demonstrates the get_count and get_direction service commands.
 *
 * Run:      ros2 run studica_control encoder_example
 * Requires: studica_launch.py running, encoder enabled in params.yaml
 *           sensors: ["encoder"], encoder.topic: encoder
 *
 * Topic subscribed: <topic>  (studica_control/EncoderMsg)
 *   encoder_count     — accumulated tick count
 *   encoder_direction — "forward" or "reverse"
 * Service: /encoder/encoder_cmd  (studica_control/SetData)
 *   Commands: get_count, get_direction
 */

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "studica_control/msg/encoder_msg.hpp"
#include "studica_control/srv/set_data.hpp"

using namespace std::chrono_literals;

static const std::string SENSOR = "encoder";

class EncoderExample : public rclcpp::Node {
public:
    EncoderExample() : Node("encoder_example") {
        // topic name is set in params.yaml under encoder.<sensor>.topic
        sub_ = create_subscription<studica_control::msg::EncoderMsg>(
            SENSOR, 10,
            std::bind(&EncoderExample::on_encoder, this, std::placeholders::_1));

        client_ = create_client<studica_control::srv::SetData>("/" + SENSOR + "/encoder_cmd");

        // poll the service every 5 seconds as a cross-check
        poll_timer_ = create_wall_timer(5s, std::bind(&EncoderExample::poll_service, this));

        RCLCPP_INFO(get_logger(), "listening on %s topic", SENSOR.c_str());
    }

private:
    void on_encoder(const studica_control::msg::EncoderMsg::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "count: %d  direction: %s",
                    msg->encoder_count, msg->encoder_direction.c_str());
    }

    void poll_service() {
        if (!client_->wait_for_service(1s)) {
            RCLCPP_WARN(get_logger(), "encoder_cmd service not available");
            return;
        }
        auto req = std::make_shared<studica_control::srv::SetData::Request>();
        req->params = "get_count";
        client_->async_send_request(req,
            [this](rclcpp::Client<studica_control::srv::SetData>::SharedFuture f) {
                RCLCPP_INFO(get_logger(), "service get_count: %s", f.get()->message.c_str());
            });
    }

    rclcpp::Subscription<studica_control::msg::EncoderMsg>::SharedPtr sub_;
    rclcpp::Client<studica_control::srv::SetData>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr poll_timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EncoderExample>());
    rclcpp::shutdown();
    return 0;
}
