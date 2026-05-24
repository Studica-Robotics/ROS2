/*
 * power_component.hpp
 *
 * ros2 component for the vmx-pi onboard power monitor.
 * reads the system (battery) voltage and publishes it as a standard
 * sensor_msgs/BatteryState message.
 *
 * always started when studica_control is running — no enabled flag needed.
 *
 * params (power.*):
 *   battery_count  int  1 or 2 — number of 3.0 Ah NiMH packs installed.
 *                               two packs wired in parallel = 6.0 Ah, same voltage curve.
 *                               default: 1
 *
 * topic (publishes): /battery_state  (sensor_msgs/BatteryState) — 2 hz
 *   voltage          — battery voltage in volts
 *   percentage       — estimated state-of-charge (0–1) from NiMH discharge curve
 *   design_capacity  — 3.0 Ah × battery_count
 *   present          — always true (vmx cannot run without a battery)
 *   power_supply_status     — DISCHARGING
 *   power_supply_technology — NIMH
 *   current / charge / capacity — NaN (no coulomb counter available)
 */

#ifndef POWER_COMPONENT_HPP
#define POWER_COMPONENT_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include "VMXPi.h"

namespace studica_control {

// Power — battery voltage monitoring node.
// always started; only one optional param (battery_count).
class Power : public rclcpp::Node {
public:
    // reads battery_count from params and creates the power node
    static std::shared_ptr<rclcpp::Node> initialize(rclcpp::Node *control,
                                                     std::shared_ptr<VMXPi> vmx);

    // composable node constructor — used when loading as a ros2 plugin
    explicit Power(const rclcpp::NodeOptions &options);

    // main constructor
    Power(std::shared_ptr<VMXPi> vmx, int battery_count);

    ~Power();

private:
    std::shared_ptr<VMXPi> vmx_;
    float design_capacity_ah_;  // 3.0 * battery_count

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // reads voltage from the vmx and publishes a BatteryState message
    void publish_data();
};

} // namespace studica_control

#endif // POWER_COMPONENT_HPP
