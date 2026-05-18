#include "servo.hpp"
#include <chrono>
#include <thread>

#define SERVO_PIN 14
int main(int argc, char* argv[])
{
    VMXChannelIndex port = SERVO_PIN;
    studica_driver::Servo servo(port, studica_driver::ServoType::Continuous, -100, 100);
    servo.SetSpeed(100);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    servo.SetSpeed(-100);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    servo.SetSpeed(0);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return 0;
}