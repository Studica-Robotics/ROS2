/*
 * titan_example.cpp
 *
 * Command motors via topics and read encoder/RPM feedback.
 * Before the run loop: Titan2 setup via titan_cmd (PID type, autotune, target velocity).
 * Then runs motor 0 at target RPM for 3 seconds, stops for 2 seconds
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

static constexpr int32_t PID_MCV2 = 2;
static constexpr int MOTOR = 0;
static constexpr float TARGET_RPM = 10.0f;
static constexpr int AUTOTUNE_WAIT_S = 15;

class TitanExample : public rclcpp::Node {
public:
    TitanExample() : Node("titan_example"), phase_(Phase::RUNNING), setup_done_(false) {
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
                RCLCPP_INFO(get_logger(), "m_0 rpm: %.2f", msg->data);
            });

        // absolute mode feedback — uncomment if encoder_mode: absolute
        // angle_sub_ = create_subscription<std_msgs::msg::Float64>(
        //     "/" + SENSOR + "/m_0/angle", 10, [this](std_msgs::msg::Float64::SharedPtr msg) {
        //         RCLCPP_INFO(get_logger(), "m_0 angle: %.2f deg", msg->data); });

        client_ = create_client<studica_control::srv::SetData>("/" + SENSOR + "/titan_cmd");

        phase_start_ = now();
        timer_ = create_wall_timer(100ms, std::bind(&TitanExample::tick, this));
        RCLCPP_INFO(get_logger(), "Titan example ready — waiting for titan_cmd, then setup");
    }

private:
    enum class Phase { RUNNING, STOPPED, DONE };

    bool call_service(const std::string& command, int motor = 0, float speed = 0.f, int32_t int_value = 0) {
        if (!client_->wait_for_service(2s)) {
            RCLCPP_ERROR(get_logger(), "titan_cmd service not available");
            return false;
        }
        auto req = std::make_shared<studica_control::srv::SetData::Request>();
        req->params = command;
        req->initparams.n_encoder = motor;
        req->initparams.speed = speed;
        req->initparams.int_value = int_value;

        auto future = client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(shared_from_this(), future, 5s) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "%s: timed out", command.c_str());
            return false;
        }
        const auto resp = future.get();
        if (resp->success) {
            RCLCPP_INFO(get_logger(), "%s: %s", command.c_str(), resp->message.c_str());
        } else {
            RCLCPP_ERROR(get_logger(), "%s: %s", command.c_str(), resp->message.c_str());
        }
        return resp->success;
    }

    /** One-time Titan2 setup before the original RUNNING / STOPPED loop. */
    void tick_setup() {
        switch (setup_index_) {
            case 0:
                if (!client_->wait_for_service(0s)) {
                    return;
                }
                RCLCPP_INFO(get_logger(), "setup: set_pid_type MCV2 on all motors");
                if (!call_service("set_pid_type", 0, 0.f, PID_MCV2)) {
                    phase_ = Phase::DONE;
                    return;
                }
                setup_index_ = 1;
                break;
            case 1:
                RCLCPP_INFO(get_logger(), "setup: autotune all (%d s)", AUTOTUNE_WAIT_S);
                if (!call_service("autotune")) {
                    phase_ = Phase::DONE;
                    return;
                }
                setup_index_ = 2;
                phase_start_ = now();
                break;
            case 2:
                if ((now() - phase_start_).seconds() < AUTOTUNE_WAIT_S) {
                    return;
                }
                if (!call_service("set_target_velocity", MOTOR, TARGET_RPM)) {
                    phase_ = Phase::DONE;
                    return;
                }
                setup_done_ = true;
                phase_ = Phase::RUNNING;
                phase_start_ = now();
                RCLCPP_INFO(get_logger(), "running at %.1f rpm", TARGET_RPM);
                break;
            default:
                break;
        }
    }

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
        if (phase_ == Phase::DONE) {
            return;
        }

        if (!setup_done_) {
            tick_setup();
            return;
        }

        const double elapsed = (now() - phase_start_).seconds();

        if (phase_ == Phase::RUNNING && elapsed >= 3.0) {
            call_service("set_target_velocity", MOTOR, 0.f);
            set_speed(MOTOR, 0.0);
            phase_ = Phase::STOPPED;
            phase_start_ = now();
            RCLCPP_INFO(get_logger(), "stopped");

        } else if (phase_ == Phase::STOPPED && elapsed >= 2.0) {
            call_reset_encoder(MOTOR);
            phase_ = Phase::DONE;
            timer_->cancel();
            RCLCPP_INFO(get_logger(), "done — encoder reset");
        }
    }

    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 4> cmd_pubs_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr enc_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rpm_sub_;
    rclcpp::Client<studica_control::srv::SetData>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time phase_start_;
    Phase phase_;
    bool setup_done_;
    int setup_index_ = 0;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TitanExample>());
    rclcpp::shutdown();
    return 0;
}
