#ifndef DIFF_DRIVE_ODOMETRY_H
#define DIFF_DRIVE_ODOMETRY_H

#include <cmath>
#include <deque>

#include "rclcpp/time.hpp"

namespace studica_control {

class RollingMeanAccumulator {
public:
    explicit RollingMeanAccumulator(size_t window_size = 10) 
        : window_size_(window_size) {}

    void accumulate(double value) {
        values_.push_back(value);
        if (values_.size() > window_size_) {
            values_.pop_front();
        }
    }

    double getRollingMean() const {
        if (values_.empty()) return 0.0;
        double sum = 0.0;
        for (double value : values_) {
            sum += value;
        }
        return sum / values_.size();
    }

    void clear() {
        values_.clear();
    }

private:
    std::deque<double> values_;
    size_t window_size_;
};

class Odometry {
public:
    explicit Odometry(size_t velocity_rolling_window_size = 10);

    void init(const rclcpp::Time &time);
    bool update(double left_pos, double right_pos, const rclcpp::Time &time);
    bool updateFromVelocity(double left_vel, double right_vel, const rclcpp::Time &time);
    void updateOpenLoop(double linear, double angular, const rclcpp::Time &time);
    void resetOdometry();

    double getX() const { return x_; }
    double getY() const { return y_; }
    double getHeading() const { return heading_; }
    double getLinear() const { return linear_; }
    double getAngular() const { return angular_; }

    void setWheelParams(double wheel_separation, double wheel_radius);
    void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

private:
    void integrateRungeKutta2(double linear, double angular);
    void integrateExact(double linear, double angular);
    void resetAccumulators();

    rclcpp::Time timestamp_;
    double x_, y_, heading_;
    double linear_, angular_;
    double wheel_separation_, wheel_radius_;
    double left_wheel_prev_pos_, right_wheel_prev_pos_;
    size_t velocity_rolling_window_size_;
    RollingMeanAccumulator linear_accumulator_;
    RollingMeanAccumulator angular_accumulator_;
};

} // namespace studica_control

#endif // DIFF_DRIVE_ODOMETRY_H
