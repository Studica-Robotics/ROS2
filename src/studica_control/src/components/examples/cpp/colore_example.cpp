/*
 * colore_example.cpp
 *
 * Subscribe to the color the Colore sensor sees, and (if matching is enabled)
 * which taught color it is. Also shows the set_brightness service command.
 *
 * Run:      ros2 run studica_control colore_example
 * Requires: studica_launch.py running, colore enabled in params.yaml
 *           sensors: ["color0"]
 *           For the match output, add "match" to publish_outputs (see README).
 *
 * Topics subscribed:
 *   /color0/color        (std_msgs/ColorRGBA)          — RGB, each channel 0.0-1.0
 *   /color0/color_match  (studica_control/ColoreMatch) — nearest taught color + confidence
 * Service: /color0/colore_cmd  (studica_control/SetData)
 *   set_brightness  — white LED 0-100, value in request.initparams.n_encoder
 *   learn_color,<name>
 */

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "studica_control/msg/colore_match.hpp"
#include "studica_control/srv/set_data.hpp"

using namespace std::chrono_literals;

static const std::string SENSOR = "color0";

class ColoreExample : public rclcpp::Node {
public:
    ColoreExample() : Node("colore_example") {
        color_sub_ = create_subscription<std_msgs::msg::ColorRGBA>(
            "/" + SENSOR + "/color", 10,
            std::bind(&ColoreExample::on_color, this, std::placeholders::_1));

        match_sub_ = create_subscription<studica_control::msg::ColoreMatch>(
            "/" + SENSOR + "/color_match", 10,
            std::bind(&ColoreExample::on_match, this, std::placeholders::_1));

        client_ = create_client<studica_control::srv::SetData>("/" + SENSOR + "/colore_cmd");

        // Set the white LED brightness to 50% once at startup.
        svc_timer_ = create_wall_timer(1s, std::bind(&ColoreExample::set_brightness_once, this));

        RCLCPP_INFO(get_logger(), "colore example ready (sensor: %s)", SENSOR.c_str());
    }

private:
    void on_color(const std_msgs::msg::ColorRGBA::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "color  r=%.2f g=%.2f b=%.2f",
                    msg->r, msg->g, msg->b);
    }

    void on_match(const studica_control::msg::ColoreMatch::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "match  %s (confidence %.2f)",
                    msg->label.c_str(), msg->confidence);
    }

    void set_brightness_once() {
        svc_timer_->cancel();  // run only once
        if (!client_->wait_for_service(2s)) {
            RCLCPP_WARN(get_logger(), "colore_cmd service not available");
            return;
        }
        auto req = std::make_shared<studica_control::srv::SetData::Request>();
        req->params = "set_brightness";
        req->initparams.n_encoder = 50;  // 0-100
        client_->async_send_request(req,
            [this](rclcpp::Client<studica_control::srv::SetData>::SharedFuture f) {
                RCLCPP_INFO(get_logger(), "set_brightness: %s", f.get()->message.c_str());
            });
    }

    rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr color_sub_;
    rclcpp::Subscription<studica_control::msg::ColoreMatch>::SharedPtr match_sub_;
    rclcpp::Client<studica_control::srv::SetData>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr svc_timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColoreExample>());
    rclcpp::shutdown();
    return 0;
}
