#include "studica_control/diff_drive_odometry.h"

namespace studica_control {

DiffOdometry::DiffOdometry(const rclcpp::NodeOptions & options) : Node("diff_drive_", options) {}

DiffOdometry::DiffOdometry(size_t velocity_rolling_window_size)
: Node("diff_odometry"),
  timestamp_(0.0), 
  x_(0.0), y_(0.0), 
  heading_(0.0), 
  linear_(0.0), 
  angular_(0.0), 
  wheel_separation_(0.0), 
  left_wheel_prev_pos_(0.0),
  right_wheel_prev_pos_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_accumulator_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size) {}

  DiffOdometry::~DiffOdometry() {}

void DiffOdometry::init(const rclcpp::Time &time) {
    resetAccumulators();
    timestamp_ = time;

    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void DiffOdometry::publishOdometry() {
    auto current_time = this->now();
    
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    tf2::Quaternion q;
    q.setRPY(0, 0, heading_);

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = linear_;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_;

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

bool DiffOdometry::updateAndPublish(double left_pos, double right_pos, const rclcpp::Time &time) {
    const double dt = time.seconds() - timestamp_.seconds();
    if (dt < 0.0001) return false;

    const double left_wheel_cur_pos = left_pos;
    const double right_wheel_cur_pos = right_pos;

    const double left_wheel_est_vel = left_wheel_cur_pos - left_wheel_prev_pos_;
    const double right_wheel_est_vel = right_wheel_cur_pos - right_wheel_prev_pos_;

    left_wheel_prev_pos_ = left_wheel_cur_pos;
    right_wheel_prev_pos_ = right_wheel_cur_pos;

    updateFromVelocity(left_wheel_est_vel, right_wheel_est_vel, time);

    publishOdometry();

    return true;
}

bool DiffOdometry::updateFromVelocity(double left_vel, double right_vel, const rclcpp::Time &time) {
    const double dt = time.seconds() - timestamp_.seconds();
    if (dt < 0.0001) return false;

    const double linear = (left_vel + right_vel) * 0.5;
    const double angular = (right_vel - left_vel) / wheel_separation_;

    integrateExact(linear, angular);
    
    timestamp_ = time;

    linear_accumulator_.accumulate(linear / dt);
    angular_accumulator_.accumulate(angular / dt);

    linear_ = linear_accumulator_.getRollingMean();
    angular_ = angular_accumulator_.getRollingMean();

    return true;
}

void DiffOdometry::updateOpenLoop(double linear, double angular, const rclcpp::Time &time) {
    linear_ = linear;
    angular_ = angular;

    const double dt = time.seconds() - timestamp_.seconds();
    timestamp_ = time;
    integrateExact(linear * dt, angular * dt);
}

void DiffOdometry::resetOdometry() {
    x_ = 0.0;
    y_ = 0.0;
    heading_ = 0.0;
}

void DiffOdometry::setWheelParams(float wheel_separation) {
    wheel_separation_ = wheel_separation;
}

void DiffOdometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size) {
    velocity_rolling_window_size_ = velocity_rolling_window_size;
    resetAccumulators();
}

void DiffOdometry::integrateRungeKutta2(double linear, double angular) {
    const double direction = heading_ + angular * 0.5;

    x_ += linear * std::cos(direction);
    y_ += linear * std::sin(direction);
    heading_ += angular;
}

void DiffOdometry::integrateExact(double linear, double angular) {
    if (fabs(angular) < 1e-6) integrateRungeKutta2(linear, angular);
    else {
        const double prev_heading = heading_;
        const double r = linear / angular;
        heading_ += angular;
        x_ += r * (std::sin(heading_) - std::sin(prev_heading));
        y_ += -r * (std::cos(heading_) - std::cos(prev_heading));
    }
}

void DiffOdometry::resetAccumulators() {
    linear_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
    angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

} // namespace studica_control