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

    // SERVICES
    //
    // - data services
    // - setters
    // - getters
    // - drivers

        dynamic_publisher_service_ = this->create_service<studica_control::srv::SetData>(
            "manage_dynamic_publisher", 
            std::bind(&StudicaControlServer::manage_dynamic_publisher_callback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Dynamic publisher ready");

        // set_data_service_ = this->create_service<studica_control::srv::SetBool>(
        //     "set_data", 
        //     std::bind(&DataServer::handle_set_data, this, std::placeholders::_1, std::placeholders::_2)
        // );

        // spawn_srv_ = nh_->create_service<turtlesim::srv::Spawn>("spawn", std::bind(&TurtleFrame::spawnCallback, this, std::placeholders::_1, std::placeholders::_2));

        // imu_service_ = this->create_service<studica_control::srv::ControlImu>(
        //     "control_imu", std::bind(&StudicaControlServer::handle_imu_service, this, std::placeholders::_1, std::placeholders::_2));
        // imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

        // motor_service_ = this->create_service<studica_control::srv::ControlMotor>(
            // "control_motor", std::bind(&StudicaControlServer::handle_motor_service, this, std::placeholders::_1, std::placeholders::_2));
        // log("Data server is ready and running...");
    }

private:
    // SERVICES
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr dynamic_publisher_service_;
    std::map<std::string, std::shared_ptr<Device>> component_map; // Store publisher objects
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;

    void handle_initialize(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                           std::shared_ptr<studica_control::srv::SetData::Response> response) {
        std::string name = request->name.c_str();
        if (component_map.find(name) != component_map.end()) {
            response->success = false;
            response->message = name + " is already running.";
            RCLCPP_INFO(this->get_logger(), "%s is already running.", name.c_str());
        } else {
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
    }

    void manage_dynamic_publisher_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response) {
        std::string action = std::string(request->command);
        std::string name = std::string(request->name);
        std::string params = std::string(request->params);

        if (action == "initialize")
        {
            // handle_initialize
            handle_initialize(request, response);
            // // Check if the publisher is already running
            // if (component_map.find(name) != component_map.end())
            // {
            //     response->success = false;
            //     response->message = name + " is already running.";
            //     RCLCPP_INFO(this->get_logger(), "%s is already running.", name.c_str());
            // }
            // else
            // {
            //     // Create and start a new publisher node
            //     RCLCPP_INFO(this->get_logger(), "Starting %s...", name.c_str());
            //     auto publisher_node = std::make_shared<DynamicPublisher>(name);
            //     component_map[name] = publisher_node;

            //     executor_->add_node(publisher_node);
                
            //     response->success = true;
            //     response->message = name + " started.";
            //     RCLCPP_INFO(this->get_logger(), "%s started.", name.c_str());
            // }
        }
        else if (action == "terminate")
        {
            // Stop the publisher if it's running
            if (component_map.find(name) != component_map.end())
            {
                RCLCPP_INFO(this->get_logger(), "Terminating and removing %s...", name.c_str());
                executor_->remove_node(component_map[name]);
                component_map.erase(name); // Remove the publisher from the map
                response->success = true;
                response->message = name + " stopped.";
                RCLCPP_INFO(this->get_logger(), "%s stopped.", name.c_str());
            }
            else
            {
                response->success = false;
                response->message = name + " does not exist.";
                RCLCPP_INFO(this->get_logger(), "%s does not exist.", name.c_str());
            }
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
;        }
    }

public:
    // Function to adjust publisher parameters dynamically
    void adjust_publisher_rate(const std::string &name, std::chrono::milliseconds new_rate)
    {
        if (component_map.find(name) != component_map.end())
        {
            // component_map[name]->adjust_publishing_rate(new_rate);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Publisher %s is not running. Cannot adjust parameters.", name.c_str());
        }
    }
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

    // Set the executor in the node manager
    node_manager->set_executor(executor);

    // Add the NodeManager to the executor
    executor->add_node(node_manager);

    // Adjust the publisher rate (can be based on user input or some logic)
    node_manager->adjust_publisher_rate("publisher_1", std::chrono::milliseconds(500)); // Adjust rate to 500ms

    // Run the executor
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
