/*
 * titan_example.cpp
 *
 * Command motors via topics and read encoder/RPM feedback.
 * Runs motor 0 at 80% for 3 seconds, stops for 2 seconds, repeats 3 times,
 * then resets the encoder via the service.
 *
 * Run:      ros2 run studica_control titan_example
 * Requires: studica_launch.py running, titan enabled in params.yaml
 *           sensor name "titan0", m_0 encoder_mode: quadrature
 *
 * Topics published:   /titan0/m_N/cmd  (std_msgs/Float64)  duty cycle -1.0 to 1.0
 * Topics subscribed:  /titan0/m_0/encoder  (std_msgs/Float64)
 *                     /titan0/m_0/rpm      (std_msgs/Float64)
 * Service:            /titan0/titan_cmd    (studica_control/SetData)
 */

#include <array>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "studica_control/srv/set_data.hpp"

using namespace std::chrono_literals;

static const std::string SENSOR = "titan0";

class TitanExample : public rclcpp::Node {
public:
    TitanExample() : Node("titan_example"), phase_(Phase::RUNNING), loop_(0) {
        // command publishers — one per motor, always present
        for (int i = 0; i < 4; ++i) {
            cmd_pubs_[i] = create_publisher<std_msgs::msg::Float64>(
                "/" + SENSOR + "/m_" + std::to_string(i) + "/cmd", 10);
        }

        // quadrature mode feedback — comment out if using encoder_mode: absolute
        enc_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/" + SENSOR + "/m_0/encoder", 10,
            [this](std_msgs::msg::Float64::SharedPtr msg) {
                RCLCPP_INFO(get_logger(), "m_0 encoder distance: %.4f", msg->data);
            });
        rpm_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/" + SENSOR + "/m_0/rpm", 10,
            [this](std_msgs::msg::Float64::SharedPtr msg) {
                RCLCPP_INFO(get_logger(), "m_0 rpm: %.1f", msg->data);
            });

        // absolute mode feedback — uncomment if encoder_mode: absolute
        // angle_sub_ = create_subscription<std_msgs::msg::Float64>(
        //     "/" + SENSOR + "/m_0/angle", 10, [this](std_msgs::msg::Float64::SharedPtr msg) {
        //         RCLCPP_INFO(get_logger(), "m_0 angle: %.2f deg", msg->data); });

        client_ = create_client<studica_control::srv::SetData>("/" + SENSOR + "/titan_cmd");

        set_speed(0, 0.8);
        phase_start_ = now();
        RCLCPP_INFO(get_logger(), "loop 1/3 — running at 80%%");

        timer_ = create_wall_timer(100ms, std::bind(&TitanExample::tick, this));
    }

private:
    enum class Phase { RUNNING, STOPPED, DONE };

    void set_speed(int motor, double duty) {
        std_msgs::msg::Float64 msg;
        msg.data = duty;
        cmd_pubs_[motor]->publish(msg);
    }

    void call_reset_encoder(int motor) {
        if (!client_->wait_for_service(2s)) {
            RCLCPP_ERROR(get_logger(), "titan_cmd service not available");
            return;
        }
        auto req = std::make_shared<studica_control::srv::SetData::Request>();
        req->params = "reset_encoder";
        req->initparams.n_encoder = motor;
        client_->async_send_request(req,
            [this](rclcpp::Client<studica_control::srv::SetData>::SharedFuture f) {
                RCLCPP_INFO(get_logger(), "reset_encoder: %s", f.get()->message.c_str());
            });
    }

    void tick() {
        if (phase_ == Phase::DONE) return;

        double elapsed = (now() - phase_start_).seconds();

        if (phase_ == Phase::RUNNING && elapsed >= 3.0) {
            set_speed(0, 0.0);
            phase_ = Phase::STOPPED;
            phase_start_ = now();
            RCLCPP_INFO(get_logger(), "loop %d/3 — stopped", loop_ + 1);

        } else if (phase_ == Phase::STOPPED && elapsed >= 2.0) {
            ++loop_;
            if (loop_ >= 3) {
                call_reset_encoder(0);
                phase_ = Phase::DONE;
                RCLCPP_INFO(get_logger(), "done — encoder reset");
                return;
            }
            set_speed(0, 0.8);
            phase_ = Phase::RUNNING;
            phase_start_ = now();
            RCLCPP_INFO(get_logger(), "loop %d/3 — running at 80%%", loop_ + 1);
        }
    }

    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 4> cmd_pubs_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr enc_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rpm_sub_;
    rclcpp::Client<studica_control::srv::SetData>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time phase_start_;
    Phase phase_;
    int loop_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TitanExample>());
    rclcpp::shutdown();
    return 0;
}
