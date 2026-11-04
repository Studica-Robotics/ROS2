/*
 * control_panel_component.hpp
 *
 * ros2 component for an operator panel: start, stop and reset buttons with an LED for each.
 * buttons connect their pin to ground when pressed. the component owns all six pins, so
 * they must not also be listed under dio in params.yaml.
 *
 * while enabled, motors stay off until start is pressed:
 *
 *   state    | motors   | start led | stop led | reset led | next
 *   ---------+----------+-----------+----------+-----------+-----------------
 *   waiting  | disabled | blinking  | off      | off       | start -> running
 *   running  | enabled  | solid     | off      | off       | stop  -> stopped
 *   stopped  | disabled | off       | solid    | blinking  | reset -> waiting
 *
 * stop also works from waiting, and start is refused while the stop button is held.
 * this is a software stop, not an e-stop: it relies on this process and the titan
 * components running.
 *
 * topic (publishes): /control_panel/state (std_msgs/String, transient local)
 *   current state: "waiting", "running" or "stopped". user code can watch this to know
 *   when to run.
 *
 * topic (publishes): /control_panel/reset (std_msgs/Empty)
 *   published on every reset press, in any state.
 *
 * topic (publishes): /motors_enabled (std_msgs/Bool, transient local)
 *   true only while running. the titan components subscribe to this.
 *
 * topic (subscribes): /control_panel/press (std_msgs/String)
 *   software button press: "start", "stop" or "reset". shares the press lockout
 *   with the physical buttons. stop is never locked out.
 */

#ifndef CONTROL_PANEL_COMPONENT_HPP
#define CONTROL_PANEL_COMPONENT_HPP

#include <array>
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include "dio.hpp"
#include "VMXPi.h"

namespace studica_control {

class ControlPanel : public rclcpp::Node {
public:
    // pin order for the button and led arrays
    enum Channel { START = 0, STOP = 1, RESET = 2 };

    // reads params.yaml and creates the panel. returns nullptr if the pins are unusable,
    // which leaves the titans held off because nothing can start them.
    static std::shared_ptr<rclcpp::Node> initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);

    // composable node constructor — used when loading as a ros2 plugin
    explicit ControlPanel(const rclcpp::NodeOptions &options);

    // main constructor — opens the pins and starts in the waiting state
    ControlPanel(std::shared_ptr<VMXPi> vmx, const std::array<int, 3> &button_pins,
                 const std::array<int, 3> &led_pins, int press_lockout_ms, double blink_hz);

    ~ControlPanel();

    // true if every button and led pin opened
    bool pins_ready() const;

private:
    enum class State { Waiting, Running, Stopped };
    enum class Led { Off, Solid, Blink };

    std::array<std::shared_ptr<studica_driver::DIO>, 3> buttons_;
    std::array<std::shared_ptr<studica_driver::DIO>, 3> leds_;

    State state_ = State::Waiting;
    std::array<Led, 3> led_modes_ = {Led::Off, Led::Off, Led::Off};
    std::array<bool, 3> led_levels_ = {false, false, false};

    // button levels: high = released. a change only counts once two samples agree,
    // so contact bounce on release can't look like a press.
    std::array<bool, 3> last_samples_ = {true, true, true};
    std::array<bool, 3> button_levels_ = {true, true, true};
    std::array<std::chrono::steady_clock::time_point, 3> last_presses_{};
    std::chrono::milliseconds press_lockout_{300};

    std::chrono::steady_clock::time_point start_time_;
    double blink_hz_ = 2.0;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motors_enabled_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr press_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void update();
    void update_leds();
    void on_press(int button, const std::string &source);
    void set_state(State state);
    void set_led_modes(Led start, Led stop, Led reset);
};

}  // namespace studica_control

#endif  // CONTROL_PANEL_COMPONENT_HPP
