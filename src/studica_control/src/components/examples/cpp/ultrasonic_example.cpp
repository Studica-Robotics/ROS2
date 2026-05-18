/*
 * ultrasonic_example.cpp
 *
 * Subscribe to HC-SR04 range readings published by the ultrasonic component.
 * Prints distance in metres; warns when the reading is out of sensor range.
 *
 * Run:      ros2 run studica_control ultrasonic_example
 * Requires: studica_launch.py running, ultrasonic enabled in params.yaml
 *           sensors: ["ultrasonic"]
 *
 * Topic subscribed: /ultrasonic/range  (sensor_msgs/Range)
 * Service:          /ultrasonic/ultrasonic_cmd  (studica_control/SetData)
 *   Commands: get_distance, get_distance_inches, get_distance_millimeters
 */

#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

static const std::string SENSOR = "ultrasonic";

class UltrasonicExample : public rclcpp::Node {
public:
    UltrasonicExample() : Node("ultrasonic_example") {
        sub_ = create_subscription<sensor_msgs::msg::Range>(
            "/" + SENSOR + "/range", 10,
            std::bind(&UltrasonicExample::on_range, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "listening on /%s/range", SENSOR.c_str());
    }

private:
    void on_range(const sensor_msgs::msg::Range::SharedPtr msg) {
        if (std::isinf(msg->range)) {
            RCLCPP_WARN(get_logger(), "out of range (valid: %.2f – %.2f m)",
                        msg->min_range, msg->max_range);
        } else {
            RCLCPP_INFO(get_logger(), "range: %.3f m", msg->range);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UltrasonicExample>());
    rclcpp::shutdown();
    return 0;
}
