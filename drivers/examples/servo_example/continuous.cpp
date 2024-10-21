#include "servo.h"
#include <thread>
#include <chrono>

int main(int argc, char *argv[])
{
    VMXChannelIndex port = 14; // servo output on pin 13
    studica_driver::Servo servo(port, studica_driver::ServoType::Continuous, -100, 100);  
    servo.SetAngle(100);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    servo.SetAngle(-100);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    servo.SetAngle(0);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return 0;
}