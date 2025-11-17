#include "studica_control/diff_drive_odometry.h"

namespace studica_control {

std::shared_ptr<DiffOdometry> DiffOdometry::initialize(rclcpp::Node *control){
    control->declare_parameter<std::string>("diff_drive_odometry.name", "");
    control->declare_parameter<bool>("diff_drive_odometry.use_imu", true);
    control->declare_parameter<std::string>("diff_drive_odometry.imu_topic", "");
    control->declare_parameter<std::string>("diff_drive_odometry.topic", "unknown");
    control->declare_parameter<bool>("diff_drive_odometry.publish_tf", true);

    std::string name = control->get_parameter("diff_drive_odometry.name").as_string();
    bool use_imu = control->get_parameter("diff_drive_odometry.use_imu").as_bool();
    std::string imu_topic = control->get_parameter("diff_drive_odometry.imu_topic").as_string();
    std::string topic = control->get_parameter("diff_drive_odometry.topic").as_string();
    bool publish_tf = control->get_parameter("diff_drive_odometry.publish_tf").as_bool();

    auto diff_odom = std::make_shared<DiffOdometry>(name, use_imu, imu_topic, topic, publish_tf, 10);
    return diff_odom;
}

DiffOdometry::DiffOdometry(const rclcpp::NodeOptions & options) : Node("diff_odometry", options) {}

DiffOdometry::DiffOdometry(const std::string &name, bool use_imu, const std::string &imu_topic, const std::string &topic, bool publish_tf, size_t velocity_rolling_window_size)
: Node(name),
  timestamp_(0.0), 
  x_(0.0), y_(0.0), 
  heading_(0.0), 
  linear_(0.0), 
  angular_(0.0), 
  wheel_separation_(0.0), 
  left_wheel_prev_pos_(0.0),
  right_wheel_prev_pos_(0.0),
  use_imu_(use_imu),
  publish_tf_(publish_tf),
  imu_topic_(imu_topic),
  topic_(topic),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_accumulator_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size) {}

DiffOdometry::~DiffOdometry() {}

void DiffOdometry::init(const rclcpp::Time &time) {
    resetAccumulators();
    timestamp_ = time;

    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_, 10, std::bind(&DiffOdometry::imuCallback, this, std::placeholders::_1));
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(topic_, 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void DiffOdometry::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(imu_data_mutex_);
    imu_data_ = *msg;
    use_imu_ = true;
}

void DiffOdometry::publishOdometry() {
    auto current_time = this->now();
    
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    tf2::Quaternion q;

    if (use_imu_) {
        std::lock_guard<std::mutex> lock(imu_data_mutex_);
        odom_msg.pose.pose.orientation = imu_data_.orientation;
        odom_msg.twist.twist.angular = imu_data_.angular_velocity;
    } else {
        q.setRPY(0, 0, heading_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        odom_msg.twist.twist.angular.z = angular_;
    }
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    
    odom_msg.twist.twist.linear.x = linear_;
    odom_msg.twist.twist.linear.y = 0.0;

    odom_msg.pose.covariance = {
        0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  1e3
    };

    odom_msg.twist.covariance = {
        0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  1e6,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  1e3
    };

    odom_publisher_->publish(odom_msg);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = current_time;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_footprint";

    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;

    tf.transform.rotation = odom_msg.pose.pose.orientation;
    
    if (publish_tf_) {
        tf_broadcaster_->sendTransform(tf);
    }
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

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::DiffOdometry)
