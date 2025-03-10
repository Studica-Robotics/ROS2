#include "studica_control/mecanum_drive_odometry.h"

namespace studica_control {

MecanumOdometry::MecanumOdometry(const rclcpp::NodeOptions & options) : Node("mecanum_odometry", options) {}

MecanumOdometry::MecanumOdometry()
: Node("mecanum_odometry"),
  timestamp_(0.0),
  length_x_(0.0),
  length_y_(0.0),
  x_(0.0),
  y_(0.0),
  theta_(0.0),
  prev_front_left_(0.0),
  prev_front_right_(0.0),
  prev_rear_left_(0.0),
  prev_rear_right_(0.0) {}  

MecanumOdometry::~MecanumOdometry() {}

void MecanumOdometry::init(const rclcpp::Time &time) {
    timestamp_ = time;
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

bool MecanumOdometry::updateAndPublish(
    const double front_left, const double front_right, 
    const double rear_left, const double rear_right, const rclcpp::Time &time) {

    const double dt = time.seconds() - timestamp_.seconds();
    if (dt < 0.0001) return false;

    double delta_front_left = front_left - prev_front_left_;
    double delta_front_right = front_right - prev_front_right_;
    double delta_rear_left = rear_left - prev_rear_left_;
    double delta_rear_right = rear_right - prev_rear_right_;

    prev_front_left_ = front_left;
    prev_front_right_ = front_right;
    prev_rear_left_ = rear_left;
    prev_rear_right_ = rear_right;

    double delta_x = 0.25 * (delta_front_left + delta_front_right + delta_rear_left + delta_rear_right);
    double delta_y = 0.25 * (-delta_front_left + delta_front_right + delta_rear_left - delta_rear_right);
    double delta_theta = (-delta_front_left + delta_front_right - delta_rear_left + delta_rear_right) / (2.0 * (length_x_ + length_y_));

    double avg_theta = theta_ + delta_theta / 2.0;

    double cos_theta = cos(avg_theta);
    double sin_theta = sin(avg_theta);

    double delta_x_global = delta_x * cos_theta - delta_y * sin_theta;
    double delta_y_global = delta_x * sin_theta + delta_y * cos_theta;

    x_ += delta_x_global;
    y_ += delta_y_global;
    theta_ += delta_theta;

    theta_ = fmod(theta_ + M_PI, 2.0 * M_PI);
    if (theta_ < 0) {
        theta_ += 2.0 * M_PI;
    }
    theta_ -= M_PI;

    publishOdometry();

    return true;   
}

void MecanumOdometry::publishOdometry() {
    auto current_time = this->now();
    
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_publisher_->publish(odom_msg);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = current_time;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_footprint";

    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;

    tf.transform.rotation = odom_msg.pose.pose.orientation;
    
    tf_broadcaster_->sendTransform(tf);
}

void MecanumOdometry::setWheelParams(const double length_x, const double length_y) {
    length_x_ = length_x;
    length_y_ = length_y;
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::MecanumOdometry)
