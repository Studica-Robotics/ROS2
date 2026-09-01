/*
 * light_tower_example.cpp
 *
 * Cycles through every light tower state via the /light_tower/set service and
 * prints state feedback from the /light_tower/state topic.
 *
 * Run:      ros2 run studica_control light_tower_example
 * Requires: studica_launch.py running, light_tower enabled in params.yaml
 *
 * Service:          /light_tower/set  (studica_control/SetData)
 *   Commands:
 *     "off"              — everything off
 *     "<color>"          — solid on  (red, green, yellow, buzzer)
 *     "<color>:blink"    — software blink at default_blink_hz
 *     "<color>:blink_hw" — hardware blink (continuous pin LOW)
 *     "<color>:<hz>"     — software blink at specific Hz
 *
 * Topic subscribed:  /light_tower/state  (std_msgs/String)  current state at 1 Hz
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "studica_control/srv/set_data.hpp"

using namespace std::chrono_literals;

class LightTowerExample : public rclcpp::Node {
public:
    LightTowerExample() : Node("light_tower_example"), step_(0) {
        states_ = {
            "red",
            "green",
            "yellow",
            "buzzer",
            "red:blink",
            "green:blink_hw",
            "yellow:2.5",
            "off",
        };

        state_sub_ = create_subscription<std_msgs::msg::String>(
            "/light_tower/state", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(get_logger(), "state: %s", msg->data.c_str());
            });

        client_ = create_client<studica_control::srv::SetData>("/light_tower/set");

        cycle_timer_ = create_wall_timer(3s, std::bind(&LightTowerExample::next_state, this));

        RCLCPP_INFO(get_logger(), "Light tower example ready — cycling every 3 s");
    }

private:
    void next_state() {
        if (!client_->wait_for_service(1s)) {
            RCLCPP_WARN(get_logger(), "/light_tower/set not available");
            return;
        }

        const std::string &cmd = states_[step_ % states_.size()];
        step_++;

        auto req = std::make_shared<studica_control::srv::SetData::Request>();
        req->params = cmd;
        client_->async_send_request(req,
            [this, cmd](rclcpp::Client<studica_control::srv::SetData>::SharedFuture f) {
                RCLCPP_INFO(get_logger(), "set \"%s\" → %s",
                            cmd.c_str(), f.get()->message.c_str());
            });
    }

    std::vector<std::string> states_;
    size_t step_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
    rclcpp::Client<studica_control::srv::SetData>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr cycle_timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LightTowerExample>());
    rclcpp::shutdown();
    return 0;
}
