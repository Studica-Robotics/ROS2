// studica_control_node.cpp

#include "studica_control/studica_control_node.h"
#include <sensor_msgs/msg/imu.hpp>
// #include <studica_control/srv/control_imu.hpp>
// #include <studica_control/srv/control_motor.hpp>

#include "studica_control/imu_driver_node.h" 

// void log(string s) { RCLCPP_INFO(this->get_logger(), s); }


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <string>
#include <map>
# include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "studica_control/device.h"

class DynamicPublisher : public Device
// tell servo to change stuff 
// send commands to sensors?
{
public:
    DynamicPublisher(const std::string &name) : Device(name)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic_" + name, 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), 
            std::bind(&DynamicPublisher::publish_message, this));
        count_ = 0;
        RCLCPP_INFO(this->get_logger(), "Publisher %s started.", name.c_str());
    }

    void publish_message()
    {
        auto message = std_msgs::msg::String();
        message.data = std::string("Message from ") + this->get_name() + std::string(": ") + std::to_string(count_++);
        std::cout << this->get_name() << ": " << count_ << std::endl;
        // message.data = ("Message from %s: %s", this->get_name(), std::to_string(count_++));
        publisher_->publish(message);
    }

    void adjust_publishing_rate(std::chrono::milliseconds new_rate)
    {
        timer_->reset();
        timer_ = this->create_wall_timer(new_rate, std::bind(&DynamicPublisher::publish_message, this));
        RCLCPP_INFO(this->get_logger(), "Publishing rate adjusted for %s.", this->get_name());
    }
    
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) {
        if(std::string(params.c_str()) == "rate") {
            this->adjust_publishing_rate(std::chrono::milliseconds(500));
        }
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};



class StudicaControlServer : public rclcpp::Node {
    bool boolean = false;
public:
    StudicaControlServer() : Node("studica_control_server") { // CONSTRUCTOR
        dynamic_publisher_service_ = this->create_service<studica_control::srv::SetData>(
            "manage_dynamic_publisher", 
            std::bind(&StudicaControlServer::manage_service_callback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Dynamic publisher ready");
    }

private:
    // SERVICES
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr dynamic_publisher_service_;
    std::map<std::string, std::shared_ptr<Device>> component_map; // Store publisher objects
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;

    void manage_service_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response) {
        std::string action = std::string(request->command);
        std::string name = std::string(request->name);
        std::string params = std::string(request->params);

        if (component_map.find(name) == component_map.end() && action != "initialize")
        {
            response->success = false;
            response->message = name + " is not initialized.";
            RCLCPP_INFO(this->get_logger(), "%s is not initialized.", name.c_str());
            return;
        }

        if (action == "initialize")
        {
            if (component_map.find(name) != component_map.end()) {
                response->success = false;
                response->message = name + " is already running.";
                RCLCPP_INFO(this->get_logger(), "%s is already running.", name.c_str());
                return;
            }
            handle_initialize(request, response);
        }
        else if (action == "terminate")
        {
            handle_terminate(request, response);
        }
        else if (action == "cmd")
        {
            if (component_map.find(name) == component_map.end())
            {
                response->success = false;
                response->message = name + " is not initialized.";
                RCLCPP_INFO(this->get_logger(), "%s is not initialized.", name.c_str());
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Running cmd %s %s", name.c_str(), params.c_str());
            component_map[name]->cmd(params, response);
        }
        else
        {
            response->success = false;
            response->message = "No such action '" + action + "'";
        }
    }

    void handle_initialize(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                           std::shared_ptr<studica_control::srv::SetData::Response> response) {
        std::string name = request->name.c_str();
        std::string component = request->component.c_str();
        if (component == "publisher") {
            auto publisher_node = std::make_shared<DynamicPublisher>(name);
            component_map[name] = publisher_node;
            executor_->add_node(publisher_node);
            response->success = true;
            response->message = name + " started.";
            RCLCPP_INFO(this->get_logger(), "%s started.", name.c_str());
        }
        else if (component == "imu") {
            RCLCPP_INFO(this->get_logger(), "Initializing component: %s, name %s.", component, name.c_str());
            auto imu_node = std::make_shared<ImuDriver>();
            component_map[name] = imu_node;
            executor_->add_node(std::dynamic_pointer_cast<rclcpp::Node>(imu_node));
        }
        else {
            response->success = false;
            response->message = "No such component '" + component + "'";
            RCLCPP_INFO(this->get_logger(), "No such component '%s'", component);
        }
    }

    void handle_terminate(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                          std::shared_ptr<studica_control::srv::SetData::Response> response) {
        std::string name = request->name.c_str();
        component_map.erase(name);
        response->success = true;
        response->message = name + " stopped.";
        RCLCPP_INFO(this->get_logger(), "%s stopped.", name.c_str());
    }

    void handle_cmd(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                    std::shared_ptr<studica_control::srv::SetData::Response> response) {
        std::string name = request->name.c_str();
        component_map[name]->cmd(request->params.c_str(), response);
    }


public:
    void set_executor(const std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>& exec)
    {
        executor_ = exec;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_manager = std::make_shared<StudicaControlServer>();
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    node_manager->set_executor(executor);
    executor->add_node(node_manager);
    executor->spin();

    rclcpp::shutdown();
    return 0;
}

// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     auto imu_node = std::make_shared<ImuDriver>();
//     rclcpp::spin(imu_node);
//     rclcpp::shutdown();
//     return 0;
// }
