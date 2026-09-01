/*
 * sharp_example.cpp
 *
 * Subscribe to GP2Y infrared range readings published by the sharp component.
 * Prints distance in metres; warns when the reading is out of sensor range.
 *
 * Run:      ros2 run studica_control sharp_example
 * Requires: studica_launch.py running, sharp enabled in params.yaml
 *           sensors: ["sharp"]
 *
 * Topic subscribed: /sharp/range  (sensor_msgs/Range)
 * Service:          /sharp/sharp_cmd  (studica_control/SetData)
 *   Commands: get_distance  (returns centimetres)
 */

#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

static const std::string SENSOR = "sharp";

class SharpExample : public rclcpp::Node {
public:
    SharpExample() : Node("sharp_example") {
        sub_ = create_subscription<sensor_msgs::msg::Range>(
            "/" + SENSOR + "/range", 10,
            std::bind(&SharpExample::on_range, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "listening on /%s/range", SENSOR.c_str());
    }

private:
    void on_range(const sensor_msgs::msg::Range::SharedPtr msg) {
        if (std::isinf(msg->range)) {
            RCLCPP_WARN(get_logger(), "out of range (valid: %.2f – %.2f m)",
                        msg->min_range, msg->max_range);
        } else {
            RCLCPP_INFO(get_logger(), "IR range: %.3f m", msg->range);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SharpExample>());
    rclcpp::shutdown();
    return 0;
}
