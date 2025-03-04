#ifndef MECANUM_DRIVE_ODOMETRY_H
#define MECANUM_DRIVE_ODOMETRY_H

#include <cmath>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/time.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

# define PLANAR_POINT_DIM 3

namespace studica_control {

class MecanumOdometry : public rclcpp::Node {
    public:
        explicit MecanumOdometry(const rclcpp::NodeOptions & options);
        explicit MecanumOdometry();
        ~MecanumOdometry();

        void init(const rclcpp::Time &time);
        bool updateAndPublish(
            const double front_left, const double front_right,
            const double rear_left, const double rear_right, const rclcpp::Time &dt);
        void publishOdometry();

        double getX() const { return x_; }
        double getY() const { return y_; }
        double getHeading() const { return theta_; }
        void setWheelParams(const double length_x, const double length_y);

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
        rclcpp::Time timestamp_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        double length_x_;
        double length_y_;

        double x_;
        double y_;
        double theta_;
        
        double prev_front_left_;
        double prev_front_right_;
        double prev_rear_left_;
        double prev_rear_right_;
};

} // namespace studica_control

#endif // MECANUM_DRIVE_ODOMETRY_H
