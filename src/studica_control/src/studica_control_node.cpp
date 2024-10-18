// studica_control_node.cpp

#include "studica_control/studica_control_node.h"
#include <sensor_msgs/msg/imu.hpp>
// #include <studica_control/srv/control_imu.hpp>
// #include <studica_control/srv/control_motor.hpp>

#include "studica_control/device.h"
#include "studica_control/imu_driver_node.h" 
#include "studica_control/ultrasonic.h"
#include "studica_control/sharp_sensor_node.h"
#include "studica_control/analog_input.h"
#include "studica_control/titan.h"
#include "studica_control/cobra_sensor_node.h"
#include "studica_control/servo.h"
#include "studica_control/DIOPin.h"
#include "nodes/encoder_node.h"

// void log(string s) { RCLCPP_INFO(this->get_logger(), s); }

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <string>
#include <map>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <studica_control/msg/initialize_params.hpp>

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
        response->success = true; //CHANGE THIS
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
        // spin on feeding the watchdog
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&StudicaControlServer::spin, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    bool feeding = false;

    // SERVICES
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr dynamic_publisher_service_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::shared_ptr<VMXPi> vmx_;
    struct Component {
        std::string name;
        std::shared_ptr<Device> component;
        std::vector<int> pins;
    };
    std::map<std::string, Component> component_map; // Store publisher objects
#define IMU_PIN 600

    void manage_service_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response) {
        std::string action = std::string(request->command);
        std::string name = std::string(request->name);
        std::string params = std::string(request->params);

        if (action == "feed_watchdog")
        {
            feeding = true;
            response->success = true;
            response->message = "Watchdog fed.";
        } else if (action == "stop_watchdog") {
            feeding = false;
            response->success = true;
            response->message = "Watchdog stopped.";
        }

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
            if (component_map.find(name) == component_map.end()) {
                response->success = false;
                response->message = name + " is not initialized.";
                RCLCPP_INFO(this->get_logger(), "%s is not initialized.", name.c_str());
                return;
            }
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
            component_map[name].component->cmd(params, response);
        }
        else
        {
            response->success = false;
            response->message = "No such action '" + action + "'";
        }
    }

    bool is_name_initialized(const std::string &name) {
        return component_map.find(name) != component_map.end();
    }

    bool get_pin_state(int pin) {
        for (auto &component : component_map) {
            for (auto &component_pin : component.second.pins) {
                if (component_pin == pin) {
                    return true;
                }
            }
        }
        return false;
    }

    bool check_pin_is_available(int pin, std::shared_ptr<studica_control::srv::SetData::Response> response) {
        if (get_pin_state(pin)) {
            response->success = false;
            response->message = "Pin already in use.";
            RCLCPP_INFO(this->get_logger(), "Pin already in use.");
            return false;
        }
        return true;
    }
public:
    void handle_initialize(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                           std::shared_ptr<studica_control::srv::SetData::Response> response) {
        std::string name = request->name.c_str();
        std::string component = request->component.c_str();
        if (component == "publisher") {
            auto publisher_node = std::make_shared<DynamicPublisher>(name);
            component_map[name] = {name, publisher_node, {}};
            executor_->add_node(publisher_node);
            response->success = true;
            response->message = name + " started.";
            RCLCPP_INFO(this->get_logger(), "%s started.", name.c_str());
        }
        else if (component == "imu") {
            RCLCPP_INFO(this->get_logger(), "Initializing component: %s, name %s.", component.c_str(), name.c_str());
            // Checks
            if (!check_pin_is_available(IMU_PIN, response)) return;
            // Initialize
            auto imu_node = std::make_shared<ImuDriver>(vmx_);
            component_map[name] = {name, imu_node, {IMU_PIN}};
            executor_->add_node(std::dynamic_pointer_cast<rclcpp::Node>(imu_node));
            response->success = true;
            response->message = name + " started.";
        }
        else if (component == "ultrasonic") {
            RCLCPP_INFO(this->get_logger(), "Initializing component: %s, name %s.", component.c_str(), name.c_str());
            uint8_t ping = request->initparams.ping;
            uint8_t echo = request->initparams.echo;
            // Checks
            if (!check_pin_is_available(ping, response) || !check_pin_is_available(echo, response)) return;
            std::vector<std::pair<uint8_t, uint8_t>> valid_pairs = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {8, 9}, {10, 11}};
            if (std::find(valid_pairs.begin(), valid_pairs.end(), std::make_pair(ping, echo)) == valid_pairs.end()) {
                response->success = false;
                response->message = "Invalid pin pair (ping, echo).";
                RCLCPP_INFO(this->get_logger(), "Invalid pin pair.");
                return;
            }
            // Initialize
            auto ultrasonic_node = std::make_shared<UltrasonicDriver>(vmx_, std::string(name), ping, echo);
            component_map[name] = {name, ultrasonic_node, {ping, echo}};
            executor_->add_node(std::dynamic_pointer_cast<rclcpp::Node>(ultrasonic_node));
            response->success = true;
            response->message = name + " started.";
        } else if (component == "sharp") {
            RCLCPP_INFO(this->get_logger(), "Initializing component: %s, name %s.", component.c_str(), name.c_str());
            uint8_t ping = request->initparams.ping;
            
            // Validate ping values
            if (ping != 0 && ping != 1 && ping != 2 && ping != 3) {
                response->success = false;
                response->message = "Invalid ping value.";
                RCLCPP_ERROR(this->get_logger(), "Invalid ping value: %d. Allowed values are 0, 1, 2, or 3.", ping);
                return;
            }
            ping += 22; // True ping channel index
            
            // Check availability
            if (!check_pin_is_available(ping, response)) return;

            // Initialize analog input
            auto analog_input = std::make_shared<AnalogInput>(vmx_, ping);
            if (!analog_input->activate_channel()) {
                response->success = false;
                response->message = "Failed to activate analog input.";
                RCLCPP_ERROR(this->get_logger(), "Failed to activate analog input for ping: %d", ping);
                return;
            }

            // Initialize SharpSensor node with analog input
            auto sharp_node = std::make_shared<SharpSensor>(vmx_, name, analog_input);
            component_map[name] = {name, sharp_node, {ping}};
            executor_->add_node(std::dynamic_pointer_cast<rclcpp::Node>(sharp_node));

            response->success = true;
            response->message = "Sharp sensor initialized successfully.";
            RCLCPP_INFO(this->get_logger(), "Sharp sensor initialized successfully for ping: %d", ping);
        } else if (component == "cobra") {
            float vref = request->initparams.vref;
            uint8_t mux_ch = request->initparams.mux_ch;
            if (mux_ch > 3) {
                response->success = false;
                response->message = "Unavailable multiplexer channel.";
                RCLCPP_INFO(this->get_logger(), "Unavailable multiplexer channel %d. Allowed channels are 0 - 3.", mux_ch);
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Initializing component: %s, name %s.", component.c_str(), name.c_str());
            auto cobra_node = std::make_shared<CobraSensor>(vmx_, name, vref, mux_ch);
            component_map[name] = {name, cobra_node, {}};
            executor_->add_node(std::dynamic_pointer_cast<rclcpp::Node>(cobra_node));
        } else if (component == "dio") {
            RCLCPP_INFO(this->get_logger(), "Initializing component: %s, name %s.", component.c_str(), name.c_str());
            uint8_t pin = request->initparams.pin;
            // Checks
            if (!check_pin_is_available(pin, response)) return;
            // pinmode variable
            std::string params = request->params.c_str();
            DIOPin::PinMode pin_mode;
            if (params == "input") {
                pin_mode = DIOPin::PinMode::INPUT;
            } else if (params == "output") {
                pin_mode = DIOPin::PinMode::OUTPUT;
            } else {
                response->success = false;
                response->message = "Invalid pin mode.";
                RCLCPP_INFO(this->get_logger(), "Invalid pin mode. Allowed values are 'input' or 'output'.");
                return;
            }
            // Initialize
            auto dio_node = std::make_shared<DIOPin>(vmx_, pin, pin_mode);
            component_map[name] = {name, dio_node, {pin}};
            executor_->add_node(std::dynamic_pointer_cast<rclcpp::Node>(dio_node));
            response->success = true;
            response->message = name + " started.";
        } else if (component == "servo") {
            RCLCPP_INFO(this->get_logger(), "Initializing component: %s, name %s.", component.c_str(), name.c_str());
            uint8_t pin = request->initparams.pin;
            if (!check_pin_is_available(pin, response)) return;
            // Initialize
            string servo_type_str = request->initparams.servo_type;
            ServoType servo_type;
            int min;
            int max;
            if (servo_type_str == "standard") {
                servo_type = ServoType::Standard;
                min = -150;
                max = 150;
            } else if (servo_type_str == "continuous") {
                servo_type = ServoType::Continuous;
                min = -100;
                max = 100;
            } else if (servo_type_str == "linear") {
                servo_type = ServoType::Linear;
                min = 0;
                max = 100;
            } else {
                response->success = false;
                response->message = "Invalid servo type.";
                RCLCPP_INFO(this->get_logger(), "Invalid servo type. Allowed values are 'standard' or 'continuous'.");
                return;
            }
            auto servo_node = std::make_shared<Servo>(vmx_, pin, servo_type, min, max);
            component_map[name] = {name, servo_node, {pin}};
            executor_->add_node(std::dynamic_pointer_cast<rclcpp::Node>(servo_node));
            response->success = true;
            response->message = name + " started.";
        } else if (component == "encoder") {
            RCLCPP_INFO(this->get_logger(), "Initializing component: %s, name %s.", component.c_str(), name.c_str());
            VMXChannelIndex pin_a = request->initparams.port_a;
            VMXChannelIndex pin_b = request->initparams.port_b;
            // Checks
            if (!check_pin_is_available(pin_a, response)) return;
            if (!check_pin_is_available(pin_b, response)) return;
            // Initialize
            auto encoder_node = std::make_shared<studica_control::EncoderNode>("name", pin_a, pin_b, vmx_);
            component_map[name] = {name, encoder_node, {pin_a, pin_b}};
            executor_->add_node(std::dynamic_pointer_cast<rclcpp::Node>(encoder_node));
            response->success = true;
            response->message = name + " started.";
        } else if (component == "titan") {
            RCLCPP_INFO(this->get_logger(), "Initializing component: %s, name %s.", component.c_str(), name.c_str());
            uint8_t nEncoder = request->initparams.n_encoder; // 0, 1, 2, 3
            float distPerTick = request->initparams.dist_per_tick; // 0.0006830601
            float speed = request->initparams.speed; // (-1, 1) // 0.8
            uint8_t canID = request->initparams.can_id; // (0,64) // 45
            uint16_t motorFreq = request->initparams.motor_freq; // [0,20000] // 15600

            auto titan_node = std::make_shared<Titan>(vmx_, name, canID, motorFreq, nEncoder, distPerTick, speed); // canID, motorFrequency
            // std::this_thread::sleep_for(std::chrono::milliseconds((int64_t)(1.0 * 1000))); // req? why?
            component_map[name] = {name, titan_node, {}};
            executor_->add_node(std::dynamic_pointer_cast<rclcpp::Node>(titan_node));
            response->success = true;
            response->message = name + " started.";
        } else {
            response->success = false;
            response->message = "No such component '" + std::string(component) + "'";
            RCLCPP_INFO(this->get_logger(), "No such component '%s'", component.c_str());
        }
    }

    void handle_terminate(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                          std::shared_ptr<studica_control::srv::SetData::Response> response) {
        std::string name = request->name;
        
        // Shutdown and Destruct
        std::shared_ptr<Device> node = component_map[name].component;
        // node.shutdown();
        executor_->remove_node(node);
        // node.reset();
        // Remove from map
        component_map.erase(name);
        response->success = true;
        response->message = name + " terminated.";
        RCLCPP_INFO(this->get_logger(), "%s terminated.", name.c_str());
    }

    void handle_cmd(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                    std::shared_ptr<studica_control::srv::SetData::Response> response) {
        std::string name = request->name.c_str();
        component_map[name].component->cmd(request->params.c_str(), response);
    }


public:
    void set_executor(const std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>& exec)
    {
        executor_ = exec;
    }
    void set_hal(const std::shared_ptr<VMXPi>& vmx)
    {
        vmx_ = vmx;
    }


    void initialize_watchdog()
    {
        VMXErrorCode vmxerr;
        // vmx_->io.SetWatchdogManagedOutputs(true, true, true, &vmxerr);
        // vmx_->io.SetWatchdogTimeoutPeriodMS(250, &vmxerr);
        // vmx_->io.SetWatchdogEnabled(true, &vmxerr);
        if (vmx_->io.SetWatchdogManagedOutputs(true, true, true, &vmxerr))
        {
            RCLCPP_INFO(this->get_logger(), "Watchdog managed outputs set.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set managed outputs.");
            // DisplayVMXError(vmxerr);
        }
        if (vmx_->io.SetWatchdogTimeoutPeriodMS(250, &vmxerr))
        {
            RCLCPP_INFO(this->get_logger(), "Watchdog timeout period set to 250 milliseconds.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set timeout period.");
            // DisplayVMXError(vmxerr);
        }
        if (vmx_->io.SetWatchdogEnabled(true, &vmxerr))
        {
            RCLCPP_INFO(this->get_logger(), "Watchdog enabled.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to enable watchdog.");
            // DisplayVMXError(vmxerr);
        }
    }
    void feed_watchdog()
    {
        VMXErrorCode vmxerr;
        if (!vmx_->io.FeedWatchdog(&vmxerr))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to feed watchdog.");
            // DisplayVMXError(vmxerr);
        }
        else 
        {
            RCLCPP_INFO(this->get_logger(), "Watchdog fed.");
        }
    }

    void stop_feeding_watchdog()
    {
        feeding = false;
    }

    void start_feeding_watchdog()
    {
        feeding = true;
    }

    void spin() {
        if (feeding) {
            feed_watchdog();
        }
            bool fed;
            VMXErrorCode vmxerr;
// bool IOCXClient::get_io_watchdog_expired(bool& expired){}
            vmx_->io.GetWatchdogExpired(fed, &vmxerr);
            // RCLCPP_INFO(this->get_logger(), "Watchdog: %s", fed ? "EXPIRED" : "FED");
            // get component named "henry"
            if (component_map.find("sharp") != component_map.end()) {
                float distance = std::dynamic_pointer_cast<SharpSensor>(component_map["sharp"].component)->get_voltage();
                printf("Distance: %f\n", distance);
                if (component_map.find("henry") != component_map.end()) {
                    // component_map["henry"].component->set(distance);
                    std::dynamic_pointer_cast<Servo>(component_map["henry"].component)->SetAngle((int)distance*60 -150);
                }
            }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);


    bool realtime = true;
    uint8_t update_rate_hz = 50;
    auto vmx = std::make_shared<VMXPi>(realtime, update_rate_hz);

    auto node_manager = std::make_shared<StudicaControlServer>();
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    node_manager->set_hal(vmx);
    node_manager->set_executor(executor);
    // node_manager->initialize_watchdog();
    // node_manager->start_feeding_watchdog();

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

