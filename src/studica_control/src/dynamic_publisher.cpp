#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <string>
#include <map>
# include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/imu.hpp>

class DynamicPublisher : public rclcpp::Node
{
public:
    DynamicPublisher(const std::string &publisher_name) : Node(publisher_name)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic_" + publisher_name, 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&DynamicPublisher::publish_message, this));
        count_ = 0;
        RCLCPP_INFO(this->get_logger(), "Publisher %s started.", publisher_name.c_str());
    }

    void publish_message()
    {
        auto message = std_msgs::msg::String();
        message.data = std::string("Message from ") + this->get_name() + std::string(": ") + std::to_string(count_++);
        // message.data = ("Message from %s: %s", this->get_name(), std::to_string(count_++));
        publisher_->publish(message);
    }

    void adjust_publishing_rate(std::chrono::milliseconds new_rate)
    {
        timer_->reset();
        timer_ = this->create_wall_timer(new_rate, std::bind(&DynamicPublisher::publish_message, this));
        RCLCPP_INFO(this->get_logger(), "Publishing rate adjusted for %s.", this->get_name());
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};