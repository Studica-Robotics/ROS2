/*
 * imu_component.h
 *
 * ros2 component for the navx imu (inertial measurement unit) built into the vmx-pi.
 * an imu measures how the board is rotating and accelerating in 3d space.
 * use it to track orientation (which way the robot is facing) and motion.
 *
 * topic (publishes): <topic> (sensor_msgs/Imu)
 *   orientation, angular velocity, and linear acceleration, published at 20hz.
 *   all values follow ros2 conventions:
 *     orientation      — quaternion (x, y, z, w), relative to power-on heading
 *     angular_velocity — radians per second (converted from deg/s)
 *     linear_accel     — metres per second squared (converted from g)
 *   covariance fields are set to -1 (unknown) per ros2 convention (rep-145).
 *
 * service: imu_cmd (studica_control/SetData)
 *   params field sets the command. available commands:
 *     zero_yaw — reset the yaw (rotation around vertical axis) to zero
 */

#ifndef IMU_COMPONENT_H
#define IMU_COMPONENT_H

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "imu.h"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {

// imu — orientation and motion sensing node.
class Imu : public rclcpp::Node {
public:
    // reads params.yaml and creates the imu node
    static std::shared_ptr<rclcpp::Node> initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);

    // composable node constructor — used when loading as a ros2 plugin
    explicit Imu(const rclcpp::NodeOptions &options);

    // main constructor — connects to the imu and sets up topics/services
    Imu(std::shared_ptr<VMXPi> vmx, const std::string &name,
        const std::string &topic, const std::string &frame_id);

    ~Imu();

private:
    std::shared_ptr<studica_driver::Imu> imu_;
    std::shared_ptr<VMXPi> vmx_;
    std::string frame_id_;  // ros frame this imu is attached to (e.g. "imu_link")

    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // handles incoming service commands
    void cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                      std::shared_ptr<studica_control::srv::SetData::Response> response);

    // reads imu values and publishes them as a sensor_msgs/Imu message
    void publish_data();

    void DisplayVMXError(VMXErrorCode vmxerr);
};

} // namespace studica_control

#endif // IMU_COMPONENT_H
