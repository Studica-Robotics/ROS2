/*
 * cobra_example.cpp
 *
 * Subscribe to all 4 Cobra reflectance channels and print them side by side.
 * Demonstrates the get_voltage service command for a single channel.
 *
 * Run:      ros2 run studica_control cobra_example
 * Requires: studica_launch.py running, cobra enabled in params.yaml
 *           sensors: ["cobra"]
 *
 * Topics subscribed: /cobra/ch_0 … /cobra/ch_3  (std_msgs/Float32)
 *   voltage reading per channel at 20 Hz
 * Service: /cobra/cobra_cmd  (studica_control/SetData)
 *   initparams.n_encoder — channel to query (0-3)
 *   Commands: get_raw, get_voltage
 */

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "studica_control/srv/set_data.hpp"

using namespace std::chrono_literals;

static const std::string SENSOR = "cobra";

class CobraExample : public rclcpp::Node {
public:
    CobraExample() : Node("cobra_example"), ch_({NAN, NAN, NAN, NAN}) {
        for (int i = 0; i < 4; ++i) {
            ch_subs_[i] = create_subscription<std_msgs::msg::Float32>(
                "/" + SENSOR + "/ch_" + std::to_string(i), 10,
                [this, i](std_msgs::msg::Float32::SharedPtr msg) {
                    ch_[i] = msg->data;
                });
        }

        client_ = create_client<studica_control::srv::SetData>("/" + SENSOR + "/cobra_cmd");

        // print all 4 channels at 10 Hz
        print_timer_ = create_wall_timer(100ms, std::bind(&CobraExample::print_channels, this));

        // poll channel 0 voltage via service every 5 seconds
        svc_timer_ = create_wall_timer(5s, std::bind(&CobraExample::poll_service, this));

        RCLCPP_INFO(get_logger(), "cobra example ready (sensor: %s)", SENSOR.c_str());
    }

private:
    void print_channels() {
        RCLCPP_INFO(get_logger(),
            "ch0: %.3fV  ch1: %.3fV  ch2: %.3fV  ch3: %.3fV",
            ch_[0], ch_[1], ch_[2], ch_[3]);
    }

    void poll_service() {
        if (!client_->wait_for_service(1s)) {
            RCLCPP_WARN(get_logger(), "cobra_cmd service not available");
            return;
        }
        auto req = std::make_shared<studica_control::srv::SetData::Request>();
        req->params = "get_voltage";
        req->initparams.n_encoder = 0;  // query channel 0
        client_->async_send_request(req,
            [this](rclcpp::Client<studica_control::srv::SetData>::SharedFuture f) {
                RCLCPP_INFO(get_logger(), "service ch0 voltage: %s V",
                            f.get()->message.c_str());
            });
    }

    std::array<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr, 4> ch_subs_;
    rclcpp::Client<studica_control::srv::SetData>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr print_timer_;
    rclcpp::TimerBase::SharedPtr svc_timer_;
    std::array<float, 4> ch_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CobraExample>());
    rclcpp::shutdown();
    return 0;
}
