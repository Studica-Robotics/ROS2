#include "studica_control/diff_drive_odometry.h"

namespace studica_control {

// Odometry::Odometry(const rclcpp::NodeOptions &options) : Node("odometry_", options) {}

Odometry::Odometry(size_t velocity_rolling_window_size) :
timestamp_(0.0), 
x_(0.0), y_(0.0), 
heading_(0.0), 
linear_(0.0), 
angular_(0.0), 
wheel_separation_(0.0), 
wheel_radius_(0.0),
left_wheel_prev_pos_(0.0),
right_wheel_prev_pos_(0.0),
velocity_rolling_window_size_(velocity_rolling_window_size),
linear_accumulator_(velocity_rolling_window_size),
angular_accumulator_(velocity_rolling_window_size)
{}

void Odometry::init(const rclcpp::Time &time) {
    resetAccumulators();
    timestamp_ = time;
}

bool Odometry::update(double left_pos, double right_pos, const rclcpp::Time &time) {
    const double dt = time.seconds() - timestamp_.seconds();
    if (dt < 0.0001) return false;

    const double left_wheel_cur_pos = left_pos * wheel_radius_;
    const double right_wheel_cur_pos = right_pos * wheel_radius_;

    const double left_wheel_est_vel = left_wheel_cur_pos - left_wheel_prev_pos_;
    const double right_wheel_est_vel = right_wheel_cur_pos - right_wheel_prev_pos_;

    left_wheel_prev_pos_ = left_wheel_cur_pos;
    right_wheel_prev_pos_ = right_wheel_cur_pos;

    updateFromVelocity(left_wheel_est_vel, right_wheel_est_vel, time);

    return true;
}

bool Odometry::updateFromVelocity(double left_vel, double right_vel, const rclcpp::Time &time) {
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

void Odometry::updateOpenLoop(double linear, double angular, const rclcpp::Time &time) {
    linear_ = linear;
    angular_ = angular;

    const double dt = time.seconds() - timestamp_.seconds();
    timestamp_ = time;
    integrateExact(linear * dt, angular * dt);
}

void Odometry::resetOdometry() {
    x_ = 0.0;
    y_ = 0.0;
    heading_ = 0.0;
}

void Odometry::setWheelParams(double wheel_separation, double wheel_radius) {
    wheel_separation_ = wheel_separation;
    wheel_radius_ = wheel_radius;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size) {
    velocity_rolling_window_size_ = velocity_rolling_window_size;
    resetAccumulators();
}

void Odometry::integrateRungeKutta2(double linear, double angular) {
    const double direction = heading_ + angular * 0.5;

    x_ += linear * std::cos(direction);
    y_ += linear * std::sin(direction);
    heading_ += angular;
}

void Odometry::integrateExact(double linear, double angular) {
    if (fabs(angular) < 1e-6) integrateRungeKutta2(linear, angular);
    else {
        const double prev_heading = heading_;
        const double r = linear / angular;
        heading_ += angular;
        x_ += r * (std::sin(heading_) - std::sin(prev_heading));
        y_ += -r * (std::cos(heading_) - std::cos(prev_heading));
    }
}

void Odometry::resetAccumulators() {
    linear_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
    angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}


}