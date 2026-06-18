/*
 * parsec_example.cpp
 *
 * Subscribe to multi-zone ToF readings published by the Parsec component.
 * Prints nearest valid distance and a compact zone grid for 100 frames, then exits.
 * Polls get_min_distance via service every 5 seconds.
 *
 * Run:      ros2 run studica_control parsec_example
 * Requires: studica_launch.py running, parsec enabled in params.yaml
 *           sensors: ["parsec"]  (name must match SENSOR below)
 *
 * Topics subscribed:
 *   /parsec/zones     (studica_control/ParsecZoneMsg)
 *   /parsec/min_range (sensor_msgs/Range)
 * Service: /parsec/parsec_cmd (studica_control/SetData)
 *   Commands: get_config, get_zone_distance, get_min_distance
 */

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "studica_control/msg/parsec_zone_msg.hpp"
#include "studica_control/srv/set_data.hpp"

using namespace std::chrono_literals;

static const std::string SENSOR = "parsec";
static constexpr int kMaxPrints = 100;

class ParsecExample : public rclcpp::Node {
public:
    ParsecExample() : Node("parsec_example") {
        zones_sub_ = create_subscription<studica_control::msg::ParsecZoneMsg>(
            "/" + SENSOR + "/zones", 10,
            std::bind(&ParsecExample::on_zones, this, std::placeholders::_1));

        range_sub_ = create_subscription<sensor_msgs::msg::Range>(
            "/" + SENSOR + "/min_range", 10,
            std::bind(&ParsecExample::on_min_range, this, std::placeholders::_1));

        client_ = create_client<studica_control::srv::SetData>("/" + SENSOR + "/parsec_cmd");
        poll_timer_ = create_wall_timer(5s, std::bind(&ParsecExample::poll_service, this));

        RCLCPP_INFO(get_logger(), "listening on /%s/zones and /%s/min_range",
                    SENSOR.c_str(), SENSOR.c_str());
    }

private:
    static bool is_valid_mm(int16_t d) { return d >= 1 && d <= 4000; }

    void on_zones(const studica_control::msg::ParsecZoneMsg::SharedPtr msg) {
        const int side = (msg->zones == 64) ? 8 : 4;
        const int center = msg->zones / 2;
        const int16_t center_mm =
            (center < static_cast<int>(msg->fdist.size())) ? msg->fdist[static_cast<size_t>(center)] : -2;

        RCLCPP_INFO(get_logger(), "%3d | zones seq=%u count=%u center=%d mm",
                    print_count_, msg->seq, msg->zones, center_mm);

        for (int row = 0; row < side; ++row) {
            std::string line = "  ";
            for (int col = 0; col < side; ++col) {
                const size_t idx = static_cast<size_t>(row * side + col);
                if (idx >= msg->fdist.size()) {
                    break;
                }
                const int16_t d = msg->fdist[idx];
                char cell[16];
                if (d == -1) {
                    std::snprintf(cell, sizeof(cell), "%7s", "off");
                } else if (d == -2 || !is_valid_mm(d)) {
                    std::snprintf(cell, sizeof(cell), "%7s", "---");
                } else {
                    std::snprintf(cell, sizeof(cell), "%4d mm", d);
                }
                line += cell;
                if (col + 1 < side) {
                    line += ' ';
                }
            }
            RCLCPP_INFO(get_logger(), "%s", line.c_str());
        }

        ++print_count_;
        if (print_count_ >= kMaxPrints) {
            RCLCPP_INFO(get_logger(), "printed %d frames, exiting", kMaxPrints);
            rclcpp::shutdown();
        }
    }

    void on_min_range(const sensor_msgs::msg::Range::SharedPtr msg) {
        if (std::isinf(msg->range)) {
            RCLCPP_WARN(get_logger(), "no valid zones in frame");
        } else {
            RCLCPP_INFO(get_logger(), "nearest valid zone: %.3f m", msg->range);
        }
    }

    void poll_service() {
        if (!client_->wait_for_service(1s)) {
            RCLCPP_WARN(get_logger(), "parsec_cmd service not available");
            return;
        }
        auto req = std::make_shared<studica_control::srv::SetData::Request>();
        req->params = "get_min_distance";
        client_->async_send_request(req,
            [this](rclcpp::Client<studica_control::srv::SetData>::SharedFuture f) {
                const auto resp = f.get();
                if (resp->success) {
                    RCLCPP_INFO(get_logger(), "service get_min_distance: %s mm", resp->message.c_str());
                } else {
                    RCLCPP_WARN(get_logger(), "service get_min_distance failed: %s",
                                resp->message.c_str());
                }
            });
    }

    rclcpp::Subscription<studica_control::msg::ParsecZoneMsg>::SharedPtr zones_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;
    rclcpp::Client<studica_control::srv::SetData>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr poll_timer_;
    int print_count_{0};
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParsecExample>());
    rclcpp::shutdown();
    return 0;
}

